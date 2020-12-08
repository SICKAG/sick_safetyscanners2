#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sick_safetyscanners2/utils/Conversions.h>
#include <sick_safetyscanners2_interfaces/msg/extended_laser_scan.hpp>
#include <sick_safetyscanners2_interfaces/msg/output_paths.hpp>
#include <sick_safetyscanners2_interfaces/msg/raw_micro_scan_data.hpp>
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
  sick_safetyscanners2_interfaces::msg::OutputPaths
  createOutputPathsMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::ExtendedLaserScan
  createExtendedLaserScanMsg(const sick::datastructure::Data& data, rclcpp::Time now);
  sick_safetyscanners2_interfaces::msg::RawMicroScanData
  createRawDataMsg(const sick::datastructure::Data& data);

private:
  std::vector<bool>
  getMedianReflectors(const std::vector<sick::datastructure::ScanPoint> scan_points);
  std::string m_frame_id;
  double m_time_offset;
  double m_range_min;
  double m_range_max;
  float m_angle_offset;
  double m_min_intensities = 0.0; /*!< min intensities for laser points */

  sick_safetyscanners2_interfaces::msg::DataHeader
  createDataHeaderMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::DerivedValues
  createDerivedValuesMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::GeneralSystemState
  createGeneralSystemStateMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::MeasurementData
  createMeasurementDataMsg(const sick::datastructure::Data& data);
  std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint>
  createScanPointMsgVector(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::IntrusionData
  createIntrusionDataMsg(const sick::datastructure::Data& data);
  std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum>
  createIntrusionDatumMsgVector(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::ApplicationData
  createApplicationDataMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::ApplicationInputs
  createApplicationInputsMsg(const sick::datastructure::Data& data);
  sick_safetyscanners2_interfaces::msg::ApplicationOutputs
  createApplicationOutputsMsg(const sick::datastructure::Data& data);
};

} // end namespace sick
