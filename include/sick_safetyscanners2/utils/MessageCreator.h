#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sick_safetyscanners2/utils/Conversions.h>
#include <sick_safetyscanners2_interfaces/msg/extended_laser_scan.hpp>
#include <sick_safetyscanners2_interfaces/msg/output_paths_msg.hpp>
#include <sick_safetyscanners_base/Types.h>
#include <sick_safetyscanners_base/datastructure/Data.h>

namespace sick {

class MessageCreator
{
public:
  MessageCreator(std::string frame_id,
                 double time_offset,
                 double range_min,
                 double range_max,
                 float angle_offset,
                 double min_intensities);


  sensor_msgs::msg::LaserScan createLaserScanMsg(const sick::datastructure::Data& data,
                                                 rclcpp::Time now);
  sick_safetyscanners2_interfaces::msg::OutputPathsMsg
  createOutputPathsMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::ExtendedLaserScan
  createExtendedLaserScanMsg(const sick::datastructure::Data& data, rclcpp::Time now);

private:
  std::vector<bool>
  getMedianReflectors(const std::vector<sick::datastructure::ScanPoint> scan_points);
  std::string m_frame_id;
  double m_time_offset;
  double m_range_min;
  double m_range_max;
  float m_angle_offset;
  double m_min_intensities = 0.0; /*!< min intensities for laser points */
};

} // end namespace sick
