#include<sick_safetyscanners2/utils/MessageCreator.h>

namespace sick {

  MessageCreator::MessageCreator(std::string frame_id, double time_offset, double range_min, double range_max, float angle_offset, double min_intensities)
    : m_frame_id(frame_id)
      , m_time_offset(time_offset)
      , m_range_min(range_min)
      , m_range_max(range_max)
      ,m_angle_offset(angle_offset)
      , m_min_intensities(min_intensities)
  {

  }


  sensor_msgs::msg::LaserScan MessageCreator::createLaserScanMsg( const sick::datastructure::Data& data )
  {

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = m_frame_id;
  std::cout << "test" << std::endl;
  //TODO
  //scan.header.stamp    = rclcpp::Node::now();
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

} //end namespace sick
