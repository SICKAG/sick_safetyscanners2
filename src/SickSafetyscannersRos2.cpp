
#include <sick_safetyscanners2/SickSafetyscannersRos2.h>

namespace sick {

SickSafetyscannersRos2::SickSafetyscannersRos2()
  : Node("SickSafetyscannersRos2")
  , m_time_offset(0.0)
  , m_range_min(0.0)
  , m_range_max(0.0)
  , m_angle_offset(-90.0)
  , m_use_pers_conf(false)
{
  RCLCPP_INFO(this->get_logger(), "Initializing SickSafetyscannersRos2 Node");

  // read parameters!
  initialize_parameters();
  load_parameters();
  sick::types::port_t tcp_port{2122};

  // TODO reconfigure?
  // TODO diagnostics

  // TODO further publishers
  m_laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);


  // Bind callback
  std::function<void(const sick::datastructure::Data&)> callback =
    std::bind(&SickSafetyscannersRos2::receiveUDPPaket, this, std::placeholders::_1);


  // Create a sensor instance
  m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
    m_sensor_ip, tcp_port, m_communications_settings, callback);

  // Start async receiving and processing of sensor data
  m_device->run();
  RCLCPP_INFO(this->get_logger(), "Communication to Sensor set up");

  readTypeCodeSettings();

  if (m_use_pers_conf)
  {
    readPersistentConfig();
    m_device->changeSensorSettings(m_communications_settings);
  }

  m_msg_creator = std::make_unique<sick::MessageCreator>(m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset, m_min_intensities);
  RCLCPP_INFO(this->get_logger(), "Node Configured and running");
}

void SickSafetyscannersRos2::readTypeCodeSettings()
{
  RCLCPP_INFO(this->get_logger(), "Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(type_code);
  m_communications_settings.e_interface_type = type_code.getInterfaceType();
  m_range_min                                = 0.1;
  m_range_max                                = type_code.getMaxRange();
}

void SickSafetyscannersRos2::readPersistentConfig()
{
  RCLCPP_INFO(this->get_logger(), "Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(config_data);
  m_communications_settings.start_angle = config_data.getStartAngle();
  m_communications_settings.end_angle   = config_data.getEndAngle();
}

void SickSafetyscannersRos2::initialize_parameters()
{
  this->declare_parameter<std::string>("frame_id", "scan");
  this->declare_parameter<std::string>("sensor_ip", "192.168.1.11");
  this->declare_parameter<std::string>("host_ip", "192.168.1.9");
  this->declare_parameter<int>("host_udp_port", 0);
  this->declare_parameter<int>("channel", 0);
  this->declare_parameter<bool>("channel_enabled", true);
  this->declare_parameter<int>("skip", 0);
  this->declare_parameter<double>("angle_start", 0.0);
  this->declare_parameter<double>("angle_end", 0.0);
  this->declare_parameter<double>("time_offset", 0.0); // TODO
  this->declare_parameter<bool>("general_system_state", true);
  this->declare_parameter<bool>("derived_settings", true);
  this->declare_parameter<bool>("measurement_data", true);
  this->declare_parameter<bool>("intrusion_data", true);
  this->declare_parameter<bool>("application_io_data", true);
  this->declare_parameter<bool>("use_persistent_config", false);
  this->declare_parameter<float>("min_intensities", 0.f);
}

void SickSafetyscannersRos2::load_parameters()
{
  rclcpp::Logger node_logger = this->get_logger();

  this->get_parameter<std::string>("frame_id", m_frame_id);
  RCLCPP_INFO(node_logger, "frame_id: %s", m_frame_id.c_str());

  std::string sensor_ip; // TODO
  this->get_parameter<std::string>("sensor_ip", sensor_ip);
  RCLCPP_INFO(node_logger, "sensor_ip: %s", sensor_ip.c_str());
  m_sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip);

  std::string host_ip;
  this->get_parameter<std::string>("host_ip", host_ip);
  RCLCPP_INFO(node_logger, "host_ip: %s", host_ip.c_str());
  // TODO check if valid IP?
  m_communications_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip);

  int host_udp_port;
  this->get_parameter<int>("host_udp_port", host_udp_port);
  RCLCPP_INFO(node_logger, "host_udp_port: %i", host_udp_port);
  m_communications_settings.host_udp_port = host_udp_port;

  int channel;
  this->get_parameter<int>("channel", channel);
  RCLCPP_INFO(node_logger, "channel: %i", channel);
  m_communications_settings.channel = channel;

  bool enabled;
  this->get_parameter<bool>("channel_enabled", enabled);
  RCLCPP_INFO(node_logger, "channel_enabled: %s", btoa(enabled).c_str());
  m_communications_settings.enabled = enabled;

  int skip;
  this->get_parameter<int>("skip", skip);
  RCLCPP_INFO(node_logger, "skip: %i", skip);
  m_communications_settings.publishing_frequency = skipToPublishFrequency(skip);

  float angle_start;
  this->get_parameter<float>("angle_start", angle_start);
  RCLCPP_INFO(node_logger, "angle_start: %f", angle_start);

  float angle_end;
  this->get_parameter<float>("angle_end", angle_end);
  RCLCPP_INFO(node_logger, "angle_end: %f", angle_end);

  // Included check before calculations to prevent rounding errors while calculating
  if (angle_start == angle_end)
  {
    m_communications_settings.start_angle = sick::radToDeg(0);
    m_communications_settings.end_angle   = sick::radToDeg(0);
  }
  else
  {
    m_communications_settings.start_angle = sick::radToDeg(angle_start) - m_angle_offset;
    m_communications_settings.end_angle   = sick::radToDeg(angle_end) - m_angle_offset;
  }


  this->get_parameter<double>("time_offset", m_time_offset);
  RCLCPP_INFO(node_logger, "time_offset: %f", m_time_offset);

  // Features
  bool general_system_state;
  this->get_parameter<bool>("general_system_state", general_system_state);
  RCLCPP_INFO(node_logger, "general_system_state: %s", btoa(general_system_state).c_str());

  bool derived_settings;
  this->get_parameter<bool>("derived_settings", derived_settings);
  RCLCPP_INFO(node_logger, "derived_settings: %s", btoa(derived_settings).c_str());

  bool measurement_data;
  this->get_parameter<bool>("measurement_data", measurement_data);
  RCLCPP_INFO(node_logger, "measurement_data: %s", btoa(measurement_data).c_str());

  bool intrusion_data;
  this->get_parameter<bool>("intrusion_data", intrusion_data);
  RCLCPP_INFO(node_logger, "intrusion_data: %s", btoa(intrusion_data).c_str());

  bool application_io_data;
  this->get_parameter<bool>("application_io_data", application_io_data);
  RCLCPP_INFO(node_logger, "application_io_data: %s", btoa(application_io_data).c_str());

  m_communications_settings.features = sick::SensorDataFeatures::toFeatureFlags(
    general_system_state, derived_settings, measurement_data, intrusion_data, application_io_data);


  this->get_parameter<bool>("use_persistent_config", m_use_pers_conf);
  RCLCPP_INFO(node_logger, "use_persistent_config: %s", btoa(m_use_pers_conf).c_str());

  this->get_parameter<double>("min_intensities", m_min_intensities);
  RCLCPP_INFO(node_logger, "min_intensities: %f", m_min_intensities);
}

void SickSafetyscannersRos2::receiveUDPPaket(const sick::datastructure::Data& data)
{
  // std::cout << "Received UDP Packet" << std::endl;
  // std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() <<
  // std::endl;
  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
  {
    auto scan = createLaserScanMessage(data);
    //auto scan = m_msg_creator->createLaserScanMsg(data);
    //scan.header.stamp = now();
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
