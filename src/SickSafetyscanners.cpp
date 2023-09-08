// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2023, Lowpad, Bleskensgraaf, Netherlands*
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
 * \file SickSafetyscanners.h
 *
 * \author  Rein Appeldoorn <rein.appeldoorn@lowpad.com>
 * \date    2023-09-07
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/SickSafetyscanners.hpp>

namespace sick {
rcl_interfaces::msg::SetParametersResult SickSafetyscanners::parametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  bool update_sensor_config = false;

  for (const auto &param : parameters) {
    std::stringstream ss;
    ss << "{" << param.get_name() << ", " << param.value_to_string() << "}";
    RCLCPP_INFO(getLogger(), "Got parameter: '%s'", ss.str().c_str());

    if (param.get_name() == "frame_id") {
      m_config.m_frame_id = param.value_to_string();
    } else if (param.get_name() == "host_ip") {
      m_config.m_communications_settings.host_ip =
          boost::asio::ip::address_v4::from_string(param.value_to_string());
      update_sensor_config = true;
    } else if (param.get_name() == "host_udp_port") {
      m_config.m_communications_settings.host_udp_port = param.as_int();
      update_sensor_config = true;
    } else if (param.get_name() == "channel") {
      m_config.m_communications_settings.channel = param.as_int();
      update_sensor_config = true;
    } else if (param.get_name() == "channel_enabled") {
      m_config.m_communications_settings.enabled = param.as_bool();
      update_sensor_config = true;
    } else if (param.get_name() == "skip") {
      m_config.m_communications_settings.publishing_frequency =
          skipToPublishFrequency(param.as_int());
      update_sensor_config = true;
    } else if (param.get_name() == "angle_start") {
      m_config.m_communications_settings.start_angle =
          sick::radToDeg(param.as_double()) - m_config.m_angle_offset;
      update_sensor_config = true;
    } else if (param.get_name() == "angle_end") {
      m_config.m_communications_settings.end_angle =
          sick::radToDeg(param.as_double()) - m_config.m_angle_offset;
      update_sensor_config = true;
    } else if (param.get_name() == "time_offset") {
      m_config.m_time_offset = param.as_double();
    } else if (param.get_name() == "general_system_state") {
      // TODO improve
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 0)) |
          (param.as_bool() << 0);
      update_sensor_config = true;
    } else if (param.get_name() == "derived_settings") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 1)) |
          (param.as_bool() << 1);
      update_sensor_config = true;
    } else if (param.get_name() == "measurement_data") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 2)) |
          (param.as_bool() << 2);
      update_sensor_config = true;
    } else if (param.get_name() == "intrusion_data") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 3)) |
          (param.as_bool() << 3);
      update_sensor_config = true;
    } else if (param.get_name() == "application_io_data") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 4)) |
          (param.as_bool() << 4);
      update_sensor_config = true;
    } else if (param.get_name() == "min_intensities") {
      m_config.m_min_intensities = param.as_double();
    } else {
      throw std::runtime_error("Parameter is not dynamic reconfigurable");
    }
  }

  if (update_sensor_config) {
    m_device->changeSensorSettings(m_config.m_communications_settings);
  }

  m_config.setupMsgCreator();

  return result;
}

void SickSafetyscanners::setupCommunication(
    std::function<void(const sick::datastructure::Data &)> callback) {
  // Create a sensor instance
  if (m_config.m_communications_settings.host_ip.is_multicast()) {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
        m_config.m_sensor_ip, m_config.m_tcp_port,
        m_config.m_communications_settings, m_config.m_interface_ip, callback);
  } else {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
        m_config.m_sensor_ip, m_config.m_tcp_port,
        m_config.m_communications_settings, callback);
  }

  RCLCPP_INFO(getLogger(), "Communication to Sensor set up");

  // Read sensor specific configurations
  readTypeCodeSettings();

  if (m_config.m_use_pers_conf) {
    readPersistentConfig();
  }

  m_config.setupMsgCreator();
}

void SickSafetyscanners::startCommunication() {
  // Start async receiving and processing of sensor data
  m_device->run();
  m_device->changeSensorSettings(m_config.m_communications_settings);
}

void SickSafetyscanners::stopCommunication() { m_device->stop(); }

bool SickSafetyscanners::getFieldData(
    const std::shared_ptr<
        sick_safetyscanners2_interfaces::srv::FieldData::Request>
        request,
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response>
        response) {
  // Suppress warning of unused request variable due to empty request fields
  (void)request;

  std::vector<sick::datastructure::FieldData> fields;
  m_device->requestFieldData(fields);

  for (size_t i = 0; i < fields.size(); i++) {
    sick::datastructure::FieldData field = fields.at(i);
    sick_safetyscanners2_interfaces::msg::Field field_msg;

    field_msg.start_angle =
        degToRad(field.getStartAngle() + m_config.m_angle_offset);
    field_msg.angular_resolution = degToRad(field.getAngularBeamResolution());
    field_msg.protective_field = field.getIsProtectiveField();

    std::vector<uint16_t> ranges = field.getBeamDistances();
    for (size_t j = 0; j < ranges.size(); j++) {
      field_msg.ranges.push_back(static_cast<float>(ranges.at(j)) * 1e-3);
    }

    response->fields.push_back(field_msg);
  }

  datastructure::DeviceName device_name;
  m_device->requestDeviceName(device_name);
  response->device_name = device_name.getDeviceName();

  std::vector<sick::datastructure::MonitoringCaseData> monitoring_cases;
  m_device->requestMonitoringCases(monitoring_cases);

  for (const auto &monitoring_case : monitoring_cases) {
    sick_safetyscanners2_interfaces::msg::MonitoringCase monitoring_case_msg;

    monitoring_case_msg.monitoring_case_number =
        monitoring_case.getMonitoringCaseNumber();
    std::vector<uint16_t> mon_fields = monitoring_case.getFieldIndices();
    std::vector<bool> mon_fields_valid = monitoring_case.getFieldsValid();
    for (size_t j = 0; j < mon_fields.size(); j++) {
      monitoring_case_msg.fields.push_back(mon_fields.at(j));
      monitoring_case_msg.fields_valid.push_back(mon_fields_valid.at(j));
    }
    response->monitoring_cases.push_back(monitoring_case_msg);
  }

  return true;
}

void SickSafetyscanners::readTypeCodeSettings() {
  RCLCPP_INFO(getLogger(), "Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(type_code);
  m_config.m_communications_settings.e_interface_type =
      type_code.getInterfaceType();
  m_config.m_range_min = 0.1;
  m_config.m_range_max = type_code.getMaxRange();
}

void SickSafetyscanners::readPersistentConfig() {
  RCLCPP_INFO(getLogger(), "Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(config_data);
  m_config.m_communications_settings.start_angle = config_data.getStartAngle();
  m_config.m_communications_settings.end_angle = config_data.getEndAngle();
}
} // namespace sick
