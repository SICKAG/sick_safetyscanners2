#include <sick_safetyscanners2/utils/MessageCreator.h>

namespace sick {

MessageCreator::MessageCreator(std::string frame_id,
                               double time_offset,
                               double range_min,
                               double range_max,
                               float angle_offset,
                               double min_intensities)
  : m_frame_id(frame_id)
  , m_time_offset(time_offset)
  , m_range_min(range_min)
  , m_range_max(range_max)
  , m_angle_offset(angle_offset)
  , m_min_intensities(min_intensities)
{
}


sensor_msgs::msg::LaserScan
MessageCreator::createLaserScanMsg(const sick::datastructure::Data& data, rclcpp::Time now)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = m_frame_id;
  scan.header.stamp    = now;
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

sick_safetyscanners2_interfaces::msg::OutputPaths
MessageCreator::createOutputPathsMsg(const sick::datastructure::Data& data)
{
  sick_safetyscanners2_interfaces::msg::OutputPaths msg;

  std::shared_ptr<sick::datastructure::ApplicationData> app_data = data.getApplicationDataPtr();
  sick::datastructure::ApplicationOutputs outputs                = app_data->getOutputs();

  std::vector<bool> eval_out         = outputs.getEvalOutVector();
  std::vector<bool> eval_out_is_safe = outputs.getEvalOutIsSafeVector();
  std::vector<bool> eval_out_valid   = outputs.getEvalOutIsValidVector();

  std::vector<uint16_t> monitoring_case_numbers  = outputs.getMonitoringCaseVector();
  std::vector<bool> monitoring_case_number_flags = outputs.getMonitoringCaseFlagsVector();

  // Fix according to issue #46, however why this appears is not clear
  if (monitoring_case_number_flags.size() > 0)
  {
    msg.active_monitoring_case = monitoring_case_numbers.at(0);
  }
  else
  {
    msg.active_monitoring_case = 0;
  }

  for (size_t i = 0; i < eval_out.size(); i++)
  {
    msg.status.push_back(eval_out.at(i));
    msg.is_safe.push_back(eval_out_is_safe.at(i));
    msg.is_valid.push_back(eval_out_valid.at(i));
  }
  return msg;
}

sick_safetyscanners2_interfaces::msg::ExtendedLaserScan
MessageCreator::createExtendedLaserScanMsg(const sick::datastructure::Data& data, rclcpp::Time now)
{
  sensor_msgs::msg::LaserScan scan = createLaserScanMsg(data, now);
  sick_safetyscanners2_interfaces::msg::ExtendedLaserScan msg;
  msg.laser_scan = scan;

  std::vector<sick::datastructure::ScanPoint> scan_points =
    data.getMeasurementDataPtr()->getScanPointsVector();
  uint32_t num_scan_points = scan_points.size();


  msg.reflektor_status.resize(num_scan_points);
  msg.intrusion.resize(num_scan_points);
  msg.reflektor_median.resize(num_scan_points);
  std::vector<bool> medians = getMedianReflectors(scan_points);
  for (uint32_t i = 0; i < num_scan_points; ++i)
  {
    const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    msg.reflektor_status[i]                         = scan_point.getReflectorBit();
    msg.intrusion[i]                                = scan_point.getContaminationBit();
    msg.reflektor_median[i]                         = medians.at(i);
  }
  return msg;
}

std::vector<bool>
MessageCreator::getMedianReflectors(const std::vector<sick::datastructure::ScanPoint> scan_points)
{
  std::vector<bool> res;
  res.resize(scan_points.size());
  bool last = false;
  int start = -1;
  for (size_t i = 0; i < scan_points.size(); i++)
  {
    res.at(i) = false;
    if (!last && scan_points.at(i).getReflectorBit())
    {
      last  = true;
      start = i;
    }
    else if (last && (!scan_points.at(i).getReflectorBit() || i == scan_points.size() - 1))
    {
      last                              = false;
      res.at(start + ((i - start) / 2)) = true;
    }
  }

  return res;
}

sick_safetyscanners2_interfaces::msg::RawMicroScanData
MessageCreator::createRawDataMsg(const sick::datastructure::Data& data)
{
  sick_safetyscanners2_interfaces::msg::RawMicroScanData msg;

  msg.header               = createDataHeaderMsg(data);
  //msg.derived_values       = createDerivedValuesMsg(data);
  //msg.general_system_state = createGeneralSystemStateMsg(data);
  //msg.measurement_data     = createMeasurementDataMsg(data);
  //msg.intrusion_data       = createIntrusionDataMsg(data);
  //msg.application_data     = createApplicationDataMsg(data);

  return msg;
}


sick_safetyscanners2_interfaces::msg::DataHeader
MessageCreator::createDataHeaderMsg(const sick::datastructure::Data& data)
{
  sick_safetyscanners2_interfaces::msg::DataHeader msg;

  if (!data.getDataHeaderPtr()->isEmpty())
  {
    std::shared_ptr<sick::datastructure::DataHeader> data_header = data.getDataHeaderPtr();

    msg.version_version       = data_header->getVersionIndicator();
    msg.version_release       = data_header->getVersionRelease();
    msg.version_major_version = data_header->getVersionMajorVersion();
    msg.version_minor_version = data_header->getVersionMinorVersion();

    msg.scan_number     = data_header->getScanNumber();
    msg.sequence_number = data_header->getSequenceNumber();

    msg.serial_number_of_device       = data_header->getSerialNumberOfDevice();
    msg.serial_number_of_channel_plug = data_header->getSerialNumberOfSystemPlug();

    msg.channel_number = data_header->getChannelNumber();

    msg.timestamp_date = data_header->getTimestampDate();
    msg.timestamp_time = data_header->getTimestampTime();
  }
  return msg;
}

sick_safetyscanners2_interfaces::msg::DerivedValues
MessageCreator::createDerivedValuesMsg(const sick::datastructure::Data& data)
{
}

sick_safetyscanners2_interfaces::msg::GeneralSystemState
MessageCreator::createGeneralSystemStateMsg(const sick::datastructure::Data& data)
{
}

sick_safetyscanners2_interfaces::msg::MeasurementData
MessageCreator::createMeasurementDataMsg(const sick::datastructure::Data& data)
{
}

std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint>
MessageCreator::createScanPointMsgVector(const sick::datastructure::Data& data)
{
}

sick_safetyscanners2_interfaces::msg::IntrusionData
MessageCreator::createIntrusionDataMsg(const sick::datastructure::Data& data)
{
}

std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum>
MessageCreator::createIntrusionDatumMsgVector(const sick::datastructure::Data& data)
{
}

sick_safetyscanners2_interfaces::msg::ApplicationData
MessageCreator::createApplicationDataMsg(const sick::datastructure::Data& data)
{
}

sick_safetyscanners2_interfaces::msg::ApplicationInputs
MessageCreator::createApplicationInputsMsg(const sick::datastructure::Data& data)
{
}

sick_safetyscanners2_interfaces::msg::ApplicationOutputs
MessageCreator::createApplicationOutputsMsg(const sick::datastructure::Data& data)
{
}

} // end namespace sick
