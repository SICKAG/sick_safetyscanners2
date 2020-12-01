#include <sick_safetyscanners_base/SickSafetyscanners.h>
#include <sick_safetyscanners_base/datastructure/Data.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace sick {


/*!
 * \brief Converts degrees to radians.
 * \param deg Degrees to convert.
 * \return To radians converted degrees.
 */
inline float degToRad(float deg)
{
  return deg * M_PI / 180.0f;
}

/*!
 * \brief Converts radians to degrees.
 * \param rad Input radians to convert
 * \return To degrees converted radians
 */
inline float radToDeg(float rad)
{
  return rad * 180.0f / M_PI;
}

/*!
 * \brief Converts a skip value into a "publish frequency" value
 * \param skip The number of scans to skip between each measured scan.  For a 25Hz laser, setting
 * 'skip' to 0 makes it publish at 25Hz, 'skip' to 1 makes it publish at 12.5Hz. \return "Publish
 * Frequency" ie. One out of every n_th scan will be published.  1 is publish every scan.  2 is
 * publish at half rate, and so on.
 */
inline uint16_t skipToPublishFrequency(int skip)
{
  return skip + 1;
}

class SickSafetyscannersRos2 : public rclcpp::Node
{
public:
  SickSafetyscannersRos2();

private:
  void receiveUDPPaket(const sick::datastructure::Data& data);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;

  std::unique_ptr<sick::AsyncSickSafetyScanner> m_device;
  sick::datastructure::CommSettings m_communications_settings;

  std::string m_frame_id;
  double m_time_offset;
  double m_range_min;
  double m_range_max;
  double m_frequency_tolerance      = 0.1;
  double m_expected_frequency       = 20.0;
  double m_timestamp_min_acceptable = -1.0;
  double m_timestamp_max_acceptable = 1.0;
  double m_min_intensities          = 0.0; /*!< min intensities for laser points */

  bool m_use_sick_angles;
  float m_angle_offset;
  bool m_use_pers_conf;

  // diagnostics?


  // dynamic reconfigure?


  sensor_msgs::msg::LaserScan createLaserScanMessage(const sick::datastructure::Data& data);
  // void createExtendedLaserScanMessage(const sick::datastructure::Data& data);
  // void createOutputPathsMessage(const sick::datastructure::Data& data);
  // void createRawDataMessage(const sick::datastructure::Data& data);

  // bool getFieldData();

  // void getMedianReflectors(const sick::datastructure::Data);
};
} // namespace sick
