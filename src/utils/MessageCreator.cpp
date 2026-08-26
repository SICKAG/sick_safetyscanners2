// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2020, SICK AG, Waldkirch
*  Copyright (C) 2020, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file MessageCreator.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2020-12-08
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/utils/MessageCreator.h>

namespace sick {

MessageCreator::MessageCreator(std::string frame_id, double time_offset,
                               double range_min, double range_max,
                               float angle_offset, double min_intensities)
    : m_frame_id(frame_id), m_time_offset(time_offset), m_range_min(range_min),
      m_range_max(range_max), m_angle_offset(angle_offset),
      m_min_intensities(min_intensities) {}

std::unique_ptr<sensor_msgs::msg::LaserScan>
MessageCreator::createLaserScanMsg(const sick::datastructure::Data &data,
                                   rclcpp::Time now) {
  std::unique_ptr<sensor_msgs::msg::LaserScan> scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = m_frame_id;
  scan->header.stamp = now + rclcpp::Duration::from_seconds(m_time_offset);
  // TODO check why returned number of beams is misaligned to size of vector
  std::vector<sick::datastructure::ScanPoint> scan_points =
      data.getMeasurementDataPtr()->getScanPointsVector();
  uint32_t num_scan_points = scan_points.size();

  scan->angle_min = sick::degToRad(data.getDerivedValuesPtr()->getStartAngle() +
                                    m_angle_offset);
  scan->angle_max = scan_points.empty() ? scan->angle_min
                                         : sick::degToRad(scan_points.back().getAngle() + m_angle_offset);
  scan->angle_increment =
      sick::degToRad(data.getDerivedValuesPtr()->getAngularBeamResolution());
  boost::posix_time::microseconds time_increment =
      boost::posix_time::microseconds(
          data.getDerivedValuesPtr()->getInterbeamPeriod());
  scan->time_increment = time_increment.total_microseconds() * 1e-6;
  boost::posix_time::milliseconds scan_time = boost::posix_time::milliseconds(
      data.getDerivedValuesPtr()->getScanTime());
  scan->scan_time = scan_time.total_microseconds() * 1e-6;
  // TODO
  scan->range_min = m_range_min;
  scan->range_max = m_range_max;
  scan->ranges.resize(num_scan_points);
  scan->intensities.resize(num_scan_points);

  for (uint32_t i = 0; i < num_scan_points; ++i) {
    const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    // Filter for intensities
    if (m_min_intensities < static_cast<double>(scan_point.getReflectivity())) {
      scan->ranges[i] = static_cast<float>(scan_point.getDistance()) *
                       data.getDerivedValuesPtr()->getMultiplicationFactor() *
                       1e-3; // mm -> m
      // Set values close to/greater than max range to infinity according to REP
      // 117 https://www.ros.org/reps/rep-0117.html
      if (scan->ranges[i] >= (0.999 * m_range_max)) {
        scan->ranges[i] = std::numeric_limits<double>::infinity();
      }
    } else {
      scan->ranges[i] = std::numeric_limits<double>::infinity();
    }
    scan->intensities[i] = static_cast<float>(scan_point.getReflectivity());
  }

  return scan;
}

std::unique_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths>
MessageCreator::createOutputPathsMsg(const sick::datastructure::Data &data) {
  std::unique_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths> msg = 
      std::make_unique<sick_safetyscanners2_interfaces::msg::OutputPaths>();

  std::shared_ptr<sick::datastructure::ApplicationData> app_data =
      data.getApplicationDataPtr();
  sick::datastructure::ApplicationOutputs outputs = app_data->getOutputs();

  std::vector<bool> eval_out = outputs.getEvalOutVector();
  std::vector<bool> eval_out_is_safe = outputs.getEvalOutIsSafeVector();
  std::vector<bool> eval_out_valid = outputs.getEvalOutIsValidVector();

  std::vector<uint16_t> monitoring_case_numbers =
      outputs.getMonitoringCaseVector();
  std::vector<bool> monitoring_case_number_flags =
      outputs.getMonitoringCaseFlagsVector();

  // Fix according to issue #46, however why this appears is not clear
  if (monitoring_case_number_flags.size() > 0) {
    msg->active_monitoring_case = monitoring_case_numbers.at(0);
  } else {
    msg->active_monitoring_case = 0;
  }

  for (size_t i = 0; i < eval_out.size(); i++) {
    msg->status.push_back(eval_out.at(i));
    msg->is_safe.push_back(eval_out_is_safe.at(i));
    msg->is_valid.push_back(eval_out_valid.at(i));
  }
  return msg;
}

std::unique_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>
MessageCreator::createExtendedLaserScanMsg(
    const sick::datastructure::Data &data, rclcpp::Time now) {
  std::unique_ptr<sensor_msgs::msg::LaserScan> scan = createLaserScanMsg(data, now);
  std::unique_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan> msg = std::make_unique<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>();
  msg->laser_scan = *scan;

  std::vector<sick::datastructure::ScanPoint> scan_points =
      data.getMeasurementDataPtr()->getScanPointsVector();
  uint32_t num_scan_points = scan_points.size();

  msg->reflektor_status.resize(num_scan_points);
  msg->intrusion.resize(num_scan_points);
  msg->reflektor_median.resize(num_scan_points);
  std::vector<bool> medians = getMedianReflectors(scan_points);
  for (uint32_t i = 0; i < num_scan_points; ++i) {
    const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    msg->reflektor_status[i] = scan_point.getReflectorBit();
    msg->intrusion[i] = scan_point.getContaminationBit();
    msg->reflektor_median[i] = medians.at(i);
  }
  return msg;
}

std::vector<bool> MessageCreator::getMedianReflectors(
    const std::vector<sick::datastructure::ScanPoint> scan_points) {
  std::vector<bool> res;
  res.resize(scan_points.size());
  bool last = false;
  int start = -1;
  for (size_t i = 0; i < scan_points.size(); i++) {
    res.at(i) = false;
    if (!last && scan_points.at(i).getReflectorBit()) {
      last = true;
      start = i;
    } else if (last && (!scan_points.at(i).getReflectorBit() ||
                        i == scan_points.size() - 1)) {
      last = false;
      res.at(start + ((i - start) / 2)) = true;
    }
  }

  return res;
}

std::unique_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData>
MessageCreator::createRawDataMsg(const sick::datastructure::Data &data) {
  std::unique_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData> msg =
      std::make_unique<sick_safetyscanners2_interfaces::msg::RawMicroScanData>();

  msg->header = createDataHeaderMsg(data);
  msg->derived_values = createDerivedValuesMsg(data);
  msg->general_system_state = createGeneralSystemStateMsg(data);
  msg->measurement_data = createMeasurementDataMsg(data);
  msg->intrusion_data = createIntrusionDataMsg(data);
  msg->application_data = createApplicationDataMsg(data);

  return msg;
}

sick_safetyscanners2_interfaces::msg::DataHeader
MessageCreator::createDataHeaderMsg(const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::DataHeader msg;

  if (!data.getDataHeaderPtr()->isEmpty()) {
    std::shared_ptr<sick::datastructure::DataHeader> data_header =
        data.getDataHeaderPtr();

    msg.version_version = data_header->getVersionIndicator();
    msg.version_release = data_header->getVersionRelease();
    msg.version_major_version = data_header->getVersionMajorVersion();
    msg.version_minor_version = data_header->getVersionMinorVersion();

    msg.scan_number = data_header->getScanNumber();
    msg.sequence_number = data_header->getSequenceNumber();

    msg.serial_number_of_device = data_header->getSerialNumberOfDevice();
    msg.serial_number_of_channel_plug =
        data_header->getSerialNumberOfSystemPlug();

    msg.channel_number = data_header->getChannelNumber();

    msg.timestamp_date = data_header->getTimestampDate();
    msg.timestamp_time = data_header->getTimestampTime();
  }
  return msg;
}

sick_safetyscanners2_interfaces::msg::DerivedValues
MessageCreator::createDerivedValuesMsg(const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::DerivedValues msg;

  if (!data.getDerivedValuesPtr()->isEmpty()) {
    std::shared_ptr<sick::datastructure::DerivedValues> derived_values =
        data.getDerivedValuesPtr();

    msg.multiplication_factor = derived_values->getMultiplicationFactor();
    msg.scan_time = derived_values->getScanTime();
    msg.interbeam_period = derived_values->getInterbeamPeriod();
    msg.number_of_beams = derived_values->getNumberOfBeams();
    msg.start_angle = derived_values->getStartAngle() + m_angle_offset;
    msg.angular_beam_resolution = derived_values->getAngularBeamResolution();
  }
  return msg;
}

sick_safetyscanners2_interfaces::msg::GeneralSystemState
MessageCreator::createGeneralSystemStateMsg(
    const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::GeneralSystemState msg;

  if (!data.getGeneralSystemStatePtr()->isEmpty()) {
    std::shared_ptr<sick::datastructure::GeneralSystemState>
        general_system_state = data.getGeneralSystemStatePtr();

    msg.run_mode_active = general_system_state->getRunModeActive();
    msg.standby_mode_active = general_system_state->getStandbyModeActive();
    msg.contamination_warning = general_system_state->getContaminationWarning();
    msg.contamination_error = general_system_state->getContaminationError();
    msg.reference_contour_status =
        general_system_state->getReferenceContourStatus();
    msg.manipulation_status = general_system_state->getManipulationStatus();

    std::vector<bool> safe_cut_off_path =
        general_system_state->getSafeCutOffPathVector();
    for (size_t i = 0; i < safe_cut_off_path.size(); i++) {
      msg.safe_cut_off_path.push_back(safe_cut_off_path.at(i));
    }

    std::vector<bool> non_safe_cut_off_path =
        general_system_state->getNonSafeCutOffPathVector();
    for (size_t i = 0; i < non_safe_cut_off_path.size(); i++) {
      msg.non_safe_cut_off_path.push_back(non_safe_cut_off_path.at(i));
    }

    std::vector<bool> reset_required_cut_off_path =
        general_system_state->getResetRequiredCutOffPathVector();
    for (size_t i = 0; i < reset_required_cut_off_path.size(); i++) {
      msg.reset_required_cut_off_path.push_back(
          reset_required_cut_off_path.at(i));
    }

    msg.current_monitoring_case_no_table_1 =
        general_system_state->getCurrentMonitoringCaseNoTable1();
    msg.current_monitoring_case_no_table_2 =
        general_system_state->getCurrentMonitoringCaseNoTable2();
    msg.current_monitoring_case_no_table_3 =
        general_system_state->getCurrentMonitoringCaseNoTable3();
    msg.current_monitoring_case_no_table_4 =
        general_system_state->getCurrentMonitoringCaseNoTable4();

    msg.application_error = general_system_state->getApplicationError();
    msg.device_error = general_system_state->getDeviceError();
  }
  return msg;
}

sick_safetyscanners2_interfaces::msg::MeasurementData
MessageCreator::createMeasurementDataMsg(
    const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::MeasurementData msg;

  if (!data.getMeasurementDataPtr()->isEmpty()) {
    msg.number_of_beams = data.getMeasurementDataPtr()->getNumberOfBeams();
    msg.scan_points = createScanPointMsgVector(data);
  }
  return msg;
}

std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint>
MessageCreator::createScanPointMsgVector(
    const sick::datastructure::Data &data) {
  std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint> msg_vector;

  std::shared_ptr<sick::datastructure::MeasurementData> measurement_data =
      data.getMeasurementDataPtr();
  std::vector<sick::datastructure::ScanPoint> scan_points =
      measurement_data->getScanPointsVector();
  // uint32_t num_points                                     =
  // measurement_data->getNumberOfBeams();
  uint32_t num_points = scan_points.size();
  for (uint32_t i = 0; i < num_points; i++) {
    sick::datastructure::ScanPoint scan_point = scan_points.at(i);
    sick_safetyscanners2_interfaces::msg::ScanPoint msg;
    msg.distance = scan_point.getDistance();
    msg.reflectivity = scan_point.getReflectivity();
    msg.angle = scan_point.getAngle() + m_angle_offset;
    msg.valid = scan_point.getValidBit();
    msg.infinite = scan_point.getInfiniteBit();
    msg.glare = scan_point.getGlareBit();
    msg.reflector = scan_point.getReflectorBit();
    msg.contamination_warning = scan_point.getContaminationWarningBit();
    msg.contamination = scan_point.getContaminationBit();

    msg_vector.push_back(msg);
  }
  return msg_vector;
}

sick_safetyscanners2_interfaces::msg::IntrusionData
MessageCreator::createIntrusionDataMsg(const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::IntrusionData msg;

  if (!data.getIntrusionDataPtr()->isEmpty()) {
    msg.data = createIntrusionDatumMsgVector(data);
  }
  return msg;
}

std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum>
MessageCreator::createIntrusionDatumMsgVector(
    const sick::datastructure::Data &data) {
  std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum> msg_vector;

  std::shared_ptr<sick::datastructure::IntrusionData> intrusion_data =
      data.getIntrusionDataPtr();
  std::vector<sick::datastructure::IntrusionDatum> intrusion_datums =
      intrusion_data->getIntrusionDataVector();

  for (size_t i = 0; i < intrusion_datums.size(); i++) {
    sick_safetyscanners2_interfaces::msg::IntrusionDatum msg;
    sick::datastructure::IntrusionDatum intrusion_datum =
        intrusion_datums.at(i);
    msg.size = intrusion_datum.getSize();
    std::vector<bool> flags = intrusion_datum.getFlagsVector();
    for (size_t j = 0; j < flags.size(); j++) {
      msg.flags.push_back(flags.at(j));
    }
    msg_vector.push_back(msg);
  }
  return msg_vector;
}

sick_safetyscanners2_interfaces::msg::ApplicationData
MessageCreator::createApplicationDataMsg(
    const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::ApplicationData msg;

  if (!data.getApplicationDataPtr()->isEmpty()) {
    msg.inputs = createApplicationInputsMsg(data);
    msg.outputs = createApplicationOutputsMsg(data);
  }
  return msg;
}

sick_safetyscanners2_interfaces::msg::ApplicationInputs
MessageCreator::createApplicationInputsMsg(
    const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::ApplicationInputs msg;

  std::shared_ptr<sick::datastructure::ApplicationData> app_data =
      data.getApplicationDataPtr();
  sick::datastructure::ApplicationInputs inputs = app_data->getInputs();
  std::vector<bool> unsafe_inputs = inputs.getUnsafeInputsInputSourcesVector();
  std::vector<bool> unsafe_inputs_flags = inputs.getUnsafeInputsFlagsVector();
  for (size_t i = 0; i < unsafe_inputs.size(); i++) {
    msg.unsafe_inputs_input_sources.push_back(unsafe_inputs.at(i));
    msg.unsafe_inputs_flags.push_back(unsafe_inputs_flags.at(i));
  }
  std::vector<uint16_t> monitoring_case = inputs.getMonitoringCasevector();
  std::vector<bool> monitoring_case_flags =
      inputs.getMonitoringCaseFlagsVector();
  for (size_t i = 0; i < monitoring_case.size(); i++) {
    msg.monitoring_case_number_inputs.push_back(monitoring_case.at(i));
    msg.monitoring_case_number_inputs_flags.push_back(
        monitoring_case_flags.at(i));
  }
  msg.linear_velocity_inputs_velocity_0 = inputs.getVelocity0();
  msg.linear_velocity_inputs_velocity_0_transmitted_safely =
      inputs.getVelocity0TransmittedSafely();
  msg.linear_velocity_inputs_velocity_0_valid = inputs.getVelocity0Valid();
  msg.linear_velocity_inputs_velocity_1 = inputs.getVelocity1();
  msg.linear_velocity_inputs_velocity_1_transmitted_safely =
      inputs.getVelocity1TransmittedSafely();
  msg.linear_velocity_inputs_velocity_1_valid = inputs.getVelocity1Valid();

  msg.sleep_mode_input = inputs.getSleepModeInput();

  return msg;
}

sick_safetyscanners2_interfaces::msg::ApplicationOutputs
MessageCreator::createApplicationOutputsMsg(
    const sick::datastructure::Data &data) {
  sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg;

  std::shared_ptr<sick::datastructure::ApplicationData> app_data =
      data.getApplicationDataPtr();
  sick::datastructure::ApplicationOutputs outputs = app_data->getOutputs();

  std::vector<bool> eval_out = outputs.getEvalOutVector();
  std::vector<bool> eval_out_is_safe = outputs.getEvalOutIsSafeVector();
  std::vector<bool> eval_out_valid = outputs.getEvalOutIsValidVector();
  for (size_t i = 0; i < eval_out.size(); i++) {
    msg.evaluation_path_outputs_eval_out.push_back(eval_out.at(i));
    msg.evaluation_path_outputs_is_safe.push_back(eval_out_is_safe.at(i));
    msg.evaluation_path_outputs_is_valid.push_back(eval_out_valid.at(i));
  }

  std::vector<uint16_t> monitoring_case = outputs.getMonitoringCaseVector();
  std::vector<bool> monitoring_case_flags =
      outputs.getMonitoringCaseFlagsVector();
  for (size_t i = 0; i < monitoring_case.size(); i++) {
    msg.monitoring_case_number_outputs.push_back(monitoring_case.at(i));
    msg.monitoring_case_number_outputs_flags.push_back(
        monitoring_case_flags.at(i));
  }

  msg.sleep_mode_output = outputs.getSleepModeOutput();
  msg.sleep_mode_output_valid = outputs.getFlagsSleepModeOutputIsValid();

  msg.error_flag_contamination_warning =
      outputs.getHostErrorFlagContaminationWarning();
  msg.error_flag_contamination_error =
      outputs.getHostErrorFlagContaminationError();
  msg.error_flag_manipulation_error =
      outputs.getHostErrorFlagManipulationError();
  msg.error_flag_glare = outputs.getHostErrorFlagGlare();
  msg.error_flag_reference_contour_intruded =
      outputs.getHostErrorFlagReferenceContourIntruded();
  msg.error_flag_critical_error = outputs.getHostErrorFlagCriticalError();
  msg.error_flags_are_valid = outputs.getFlagsHostErrorFlagsAreValid();

  msg.linear_velocity_outputs_velocity_0 = outputs.getVelocity0();
  msg.linear_velocity_outputs_velocity_0_transmitted_safely =
      outputs.getVelocity0TransmittedSafely();
  msg.linear_velocity_outputs_velocity_0_valid = outputs.getVelocity0Valid();
  msg.linear_velocity_outputs_velocity_1 = outputs.getVelocity1();
  msg.linear_velocity_outputs_velocity_1_transmitted_safely =
      outputs.getVelocity1TransmittedSafely();
  msg.linear_velocity_outputs_velocity_1_valid = outputs.getVelocity1Valid();

  std::vector<int16_t> resulting_velocities =
      outputs.getResultingVelocityVector();
  std::vector<bool> resulting_velocities_flags =
      outputs.getResultingVelocityIsValidVector();

  for (size_t i = 0; i < resulting_velocities.size(); i++) {
    msg.resulting_velocity.push_back(resulting_velocities.at(i));
    msg.resulting_velocity_flags.push_back(resulting_velocities_flags.at(i));
  }

  return msg;
}

} // end namespace sick
