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
 * \file SickSafetyscannersRos.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2020-12-08
 */
//----------------------------------------------------------------------

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

  // Dynamic Parameter Change client
  m_parameters_client   = std::make_shared<rclcpp::AsyncParametersClient>(this);
  m_parameter_event_sub = m_parameters_client->on_parameter_event(
    std::bind(&SickSafetyscannersRos2::onParameterEventCallback, this, std::placeholders::_1));
  // TODO reconfigure?
  // TODO diagnostics

  // init publishers and services
  m_laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  m_extended_laser_scan_publisher =
    this->create_publisher<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>("extended_scan",
                                                                                    1);
  m_output_paths_publisher =
    this->create_publisher<sick_safetyscanners2_interfaces::msg::OutputPaths>("output_paths", 1);
  m_raw_data_publisher =
    this->create_publisher<sick_safetyscanners2_interfaces::msg::RawMicroScanData>("raw_data", 1);

  m_field_data_service = this->create_service<sick_safetyscanners2_interfaces::srv::FieldData>(
    "field_data",
    std::bind(
      &SickSafetyscannersRos2::getFieldData, this, std::placeholders::_1, std::placeholders::_2));

  // Bind callback
  std::function<void(const sick::datastructure::Data&)> callback =
    std::bind(&SickSafetyscannersRos2::receiveUDPPaket, this, std::placeholders::_1);


  // Create a sensor instance
  m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
    m_sensor_ip, tcp_port, m_communications_settings, callback);

  RCLCPP_INFO(this->get_logger(), "Communication to Sensor set up");

  // Read sensor specific configurations
  readTypeCodeSettings();

  if (m_use_pers_conf)
  {
    readPersistentConfig();
    m_device->changeSensorSettings(m_communications_settings);
  }

  // Start async receiving and processing of sensor data
  m_device->run();
  m_msg_creator = std::make_unique<sick::MessageCreator>(
    m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset, m_min_intensities);
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

void SickSafetyscannersRos2::onParameterEventCallback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
  std::stringstream ss;
  ss << "\nParameter event:\n new parameters:";
  for (auto& new_parameter : event->new_parameters)
  {
    ss << "\n  " << new_parameter.name;
  }
  ss << "\n changed parameters:";
  for (auto& changed_parameter : event->changed_parameters)
  {
    ss << "\n  " << changed_parameter.name;
  }
  ss << "\n deleted parameters:";
  for (auto& deleted_parameter : event->deleted_parameters)
  {
    ss << "\n  " << deleted_parameter.name;
  }
  ss << "\n";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  load_parameters();
  m_device->changeSensorSettings(m_communications_settings);
  m_msg_creator = std::make_unique<sick::MessageCreator>(
    m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset, m_min_intensities);

  RCLCPP_INFO(this->get_logger(), "Updated sensor settings");
}

void SickSafetyscannersRos2::receiveUDPPaket(const sick::datastructure::Data& data)
{
  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
  {
    auto scan = m_msg_creator->createLaserScanMsg(data, this->now());
    m_laser_scan_publisher->publish(scan);

    sick_safetyscanners2_interfaces::msg::ExtendedLaserScan extended_scan =
      m_msg_creator->createExtendedLaserScanMsg(data, this->now());

    m_extended_laser_scan_publisher->publish(extended_scan);

    auto output_paths = m_msg_creator->createOutputPathsMsg(data);
    m_output_paths_publisher->publish(output_paths);
  }

  auto raw_msg = m_msg_creator->createRawDataMsg(data);
  m_raw_data_publisher->publish(raw_msg);
}


bool SickSafetyscannersRos2::getFieldData(
  const std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Request> request,
  std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response> response)
{
  // Suppress warning of unused request variable due to empty request fields
  (void)request;

  std::vector<sick::datastructure::FieldData> fields;
  m_device->requestFieldData(fields);

  for (size_t i = 0; i < fields.size(); i++)
  {
    sick::datastructure::FieldData field = fields.at(i);
    sick_safetyscanners2_interfaces::msg::Field field_msg;

    field_msg.start_angle        = degToRad(field.getStartAngle() + m_angle_offset);
    field_msg.angular_resolution = degToRad(field.getAngularBeamResolution());
    field_msg.protective_field   = field.getIsProtectiveField();

    std::vector<uint16_t> ranges = field.getBeamDistances();
    for (size_t j = 0; j < ranges.size(); j++)
    {
      field_msg.ranges.push_back(static_cast<float>(ranges.at(j)) * 1e-3);
    }

    response->fields.push_back(field_msg);
  }

  datastructure::DeviceName device_name;
  m_device->requestDeviceName(device_name);
  response->device_name = device_name.getDeviceName();


  std::vector<sick::datastructure::MonitoringCaseData> monitoring_cases;
  m_device->requestMonitoringCases(monitoring_cases);

  for (size_t i = 0; i < monitoring_cases.size(); i++)
  {
    sick::datastructure::MonitoringCaseData monitoring_case_data = monitoring_cases.at(i);
    sick_safetyscanners2_interfaces::msg::MonitoringCase monitoring_case_msg;

    monitoring_case_msg.monitoring_case_number = monitoring_case_data.getMonitoringCaseNumber();
    std::vector<uint16_t> mon_fields           = monitoring_case_data.getFieldIndices();
    std::vector<bool> mon_fields_valid         = monitoring_case_data.getFieldsValid();
    for (size_t j = 0; j < mon_fields.size(); j++)
    {
      monitoring_case_msg.fields.push_back(mon_fields.at(j));
      monitoring_case_msg.fields_valid.push_back(mon_fields_valid.at(j));
    }
    response->monitoring_cases.push_back(monitoring_case_msg);
  }

  return true;
}


} // namespace sick
