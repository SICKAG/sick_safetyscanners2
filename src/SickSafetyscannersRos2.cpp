
#include <sick_safetyscanners2/SickSafetyscannersRos2.h>

namespace sick {

SickSafetyscannersRos2::SickSafetyscannersRos2()
  : Node("SickSafetyscannersRos2")
  // , m_initialised(false)
  , m_time_offset(0.0)
  , m_range_min(0.0)
  , m_range_max(100.0) // TODO read from typecode
  , m_angle_offset(-90.0)
  , m_use_pers_conf(false)
{
  std::cout << "Init ROS2 Node" << std::endl;

  // TODO read params!
  m_frame_id = "scan";

  m_laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);


  // Sensor IP and Port
  sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string("192.168.1.11");
  sick::types::port_t tcp_port{2122};

  // Prepare the CommSettings for Sensor streaming data
  sick::datastructure::CommSettings comm_settings;
  comm_settings.host_ip       = boost::asio::ip::address_v4::from_string("192.168.1.9");
  comm_settings.host_udp_port = 0;


  // Bind callback
  std::function<void(const sick::datastructure::Data&)> callback =
    std::bind(&SickSafetyscannersRos2::receiveUDPPaket, this, std::placeholders::_1);


  // Create a sensor instance
  m_device =
    std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, callback);

  // Start async receiving and processing of sensor data
  m_device->run();
  std::cout << "Running" << std::endl;
}

void SickSafetyscannersRos2::receiveUDPPaket(const sick::datastructure::Data& data)
{
  std::cout << "Received UDP Packet" << std::endl;
  std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;
  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
  {
    auto scan = createLaserScanMessage(data);
    // publish
    m_laser_scan_publisher->publish(scan);
  }
}

sensor_msgs::msg::LaserScan
SickSafetyscannersRos2::createLaserScanMessage(const sick::datastructure::Data& data)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = m_frame_id;
  scan.header.stamp    = now();
  // Add time offset (to account for network latency etc.)
  // scan.header.stamp += ros::Duration().fromSec(m_time_offset); TODO
  // TODO check why returned number of beams is misaligned to size of vector
  std::vector<sick::datastructure::ScanPoint> scan_points =
    data.getMeasurementDataPtr()->getScanPointsVector();
  uint32_t num_scan_points = scan_points.size();

  scan.angle_min = sick::degToRad(data.getDerivedValuesPtr()->getStartAngle() + m_angle_offset);
  double angle_max =
    sick::degToRad(data.getMeasurementDataPtr()
                     ->getScanPointsVector()
                     .at(data.getMeasurementDataPtr()->getScanPointsVector().size() - 1)
                     .getAngle() +
                   m_angle_offset);
  scan.angle_max       = angle_max;
  scan.angle_increment = sick::degToRad(data.getDerivedValuesPtr()->getAngularBeamResolution());
  boost::posix_time::microseconds time_increment =
    boost::posix_time::microseconds(data.getDerivedValuesPtr()->getInterbeamPeriod());
  scan.time_increment = time_increment.total_microseconds() * 1e-6;
  boost::posix_time::milliseconds scan_time =
    boost::posix_time::milliseconds(data.getDerivedValuesPtr()->getScanTime());
  scan.scan_time = scan_time.total_microseconds() * 1e-6;
  // TODO
  scan.range_min = m_range_min;
  scan.range_max = m_range_max;
  scan.ranges.resize(num_scan_points);
  scan.intensities.resize(num_scan_points);


  for (uint32_t i = 0; i < num_scan_points; ++i)
  {
    const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    // Filter for intensities
    if (m_min_intensities < static_cast<double>(scan_point.getReflectivity()))
    {
      scan.ranges[i] = static_cast<float>(scan_point.getDistance()) *
                       data.getDerivedValuesPtr()->getMultiplicationFactor() * 1e-3; // mm -> m
      // Set values close to/greater than max range to infinity according to REP 117
      // https://www.ros.org/reps/rep-0117.html
      if (scan.ranges[i] >= (0.999 * m_range_max))
      {
        scan.ranges[i] = std::numeric_limits<double>::infinity();
      }
    }
    else
    {
      scan.ranges[i] = std::numeric_limits<double>::infinity();
    }
    scan.intensities[i] = static_cast<float>(scan_point.getReflectivity());
  }
  return scan;
}

} // namespace sick
