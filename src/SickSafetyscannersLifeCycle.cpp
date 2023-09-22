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
 * \file SickSafetyscannersLifeCycle.cpp
 *
 * \authors  Soma gallai<soma.gallai@cm-robotics.com>  Erwin Lejeune <erwin.lejeune@cm-robotics.com>
 * \date    2021-05-27
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/SickSafetyscannersLifeCycle.hpp>

#include "rclcpp_components/register_node_macro.hpp"

namespace sick {


SickSafetyscannersLifeCycle::SickSafetyscannersLifeCycle(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("SickSafetyscannersLifecycle", options)
  , m_time_offset(0.0)
  , m_range_min(0.0)
  , m_range_max(0.0)
  , m_angle_offset(-90.0)
  , m_use_pers_conf(false)
{
  RCLCPP_INFO(this->get_logger(), "Initializing SickSafetyscannersLifeCycle ");

  // read parameters!
  initialize_parameters();
  load_parameters();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_configure()...");
  sick::types::port_t tcp_port{2122};

  // Dynamic Parameter Change client
  m_param_callback = add_on_set_parameters_callback(
    std::bind(&SickSafetyscannersLifeCycle::parametersCallback, this, std::placeholders::_1));

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

  m_config_metadata_service =
    this->create_service<sick_safetyscanners2_interfaces::srv::ConfigMetadata>("config_metadata",
    std::bind(&SickSafetyscannersLifeCycle::getConfigMetadata,
      this, std::placeholders::_1, std::placeholders::_2));
  m_field_data_service = this->create_service<sick_safetyscanners2_interfaces::srv::FieldData>(
    "field_data",
    std::bind(&SickSafetyscannersLifeCycle::getFieldData,
              this,
              std::placeholders::_1,
              std::placeholders::_2));
  m_application_name_service =
    this->create_service<sick_safetyscanners2_interfaces::srv::ApplicationName>("application_name",
    std::bind(&SickSafetyscannersLifeCycle::getApplicationName,
      this, std::placeholders::_1, std::placeholders::_2));
  m_type_code_service =
    this->create_service<sick_safetyscanners2_interfaces::srv::TypeCode>("type_code",
    std::bind(&SickSafetyscannersLifeCycle::getTypeCode,
      this, std::placeholders::_1, std::placeholders::_2));


  // Bind callback
  std::function<void(const sick::datastructure::Data&)> callback =
    std::bind(&SickSafetyscannersLifeCycle::receiveUDPPaket, this, std::placeholders::_1);

  // Create a sensor instance
  if (m_communications_settings.host_ip.is_multicast())
  {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
      m_sensor_ip, tcp_port, m_communications_settings, m_interface_ip, callback);
  }
  else
  {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
      m_sensor_ip, tcp_port, m_communications_settings, callback);
  }

  RCLCPP_INFO(this->get_logger(), "Communication to Sensor set up");

  // Read sensor specific configurations
  readTypeCodeSettings();

  if (m_use_pers_conf)
  {
    readPersistentConfig();
  }
  m_msg_creator = std::make_unique<sick::MessageCreator>(
    m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset, m_min_intensities);

  RCLCPP_INFO(this->get_logger(), "Node Configured");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_activate()...");

  m_laser_scan_publisher->on_activate();
  m_extended_laser_scan_publisher->on_activate();
  m_output_paths_publisher->on_activate();
  m_raw_data_publisher->on_activate();

  // Start async receiving and processing of sensor data
  m_device->run();
  m_device->changeSensorSettings(m_communications_settings);

  RCLCPP_INFO(this->get_logger(), "Node activated, device is running...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate()...");
  m_device->stop();
  m_laser_scan_publisher->on_deactivate();
  m_extended_laser_scan_publisher->on_deactivate();
  m_output_paths_publisher->on_deactivate();
  m_raw_data_publisher->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Node deactivated, device stopped...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup()...");
  m_laser_scan_publisher.reset();
  m_extended_laser_scan_publisher.reset();
  m_output_paths_publisher.reset();
  m_raw_data_publisher.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SickSafetyscannersLifeCycle::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown()...");
  m_laser_scan_publisher.reset();
  m_extended_laser_scan_publisher.reset();
  m_output_paths_publisher.reset();
  m_raw_data_publisher.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SickSafetyscannersLifeCycle::readTypeCodeSettings()
{
  RCLCPP_INFO(this->get_logger(), "Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(type_code);
  m_communications_settings.e_interface_type = type_code.getInterfaceType();
  m_range_min                                = 0.1;
  m_range_max                                = type_code.getMaxRange();
}

void SickSafetyscannersLifeCycle::readPersistentConfig()
{
  RCLCPP_INFO(this->get_logger(), "Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(config_data);
  m_communications_settings.start_angle = config_data.getStartAngle();
  m_communications_settings.end_angle   = config_data.getEndAngle();
}

void SickSafetyscannersLifeCycle::initialize_parameters()
{
  this->declare_parameter<std::string>("frame_id", "scan");
  this->declare_parameter<std::string>("sensor_ip", "192.168.1.11");
  this->declare_parameter<std::string>("host_ip", "192.168.1.9");
  this->declare_parameter<std::string>("interface_ip", "0.0.0.0");
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

void SickSafetyscannersLifeCycle::load_parameters()
{
  rclcpp::Logger node_logger = this->get_logger();

  this->get_parameter<std::string>("frame_id", m_frame_id);
  RCLCPP_INFO(node_logger, "frame_id: %s", m_frame_id.c_str());

  std::string sensor_ip;
  this->get_parameter<std::string>("sensor_ip", sensor_ip);
  RCLCPP_INFO(node_logger, "sensor_ip: %s", sensor_ip.c_str());
  m_sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip);

  std::string interface_ip;
  this->get_parameter<std::string>("interface_ip", interface_ip);
  RCLCPP_INFO(node_logger, "interface_ip: %s", interface_ip.c_str());
  m_interface_ip = boost::asio::ip::address_v4::from_string(interface_ip);

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

rcl_interfaces::msg::SetParametersResult
SickSafetyscannersLifeCycle::parametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful         = true;
  result.reason             = "";
  bool update_sensor_config = false;

  for (auto param : parameters)
  {
    {
      std::stringstream ss;
      ss << "{" << param.get_name() << ", " << param.value_to_string() << "}";
      RCLCPP_INFO(this->get_logger(), "Got parameter: '%s'", ss.str().c_str());

      if (!param.get_name().compare("frame_id"))
      {
        m_frame_id = param.value_to_string();
      }
      else if (!param.get_name().compare("host_ip"))
      {
        m_communications_settings.host_ip =
          boost::asio::ip::address_v4::from_string(param.value_to_string());
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("host_udp_port"))
      {
        m_communications_settings.host_udp_port = param.as_int();
        update_sensor_config                    = true;
      }
      else if (!param.get_name().compare("channel"))
      {
        m_communications_settings.channel = param.as_int();
        update_sensor_config              = true;
      }
      else if (!param.get_name().compare("channel_enabled"))
      {
        m_communications_settings.enabled = param.as_bool();
        update_sensor_config              = true;
      }
      else if (!param.get_name().compare("skip"))
      {
        m_communications_settings.publishing_frequency = skipToPublishFrequency(param.as_int());
        update_sensor_config                           = true;
      }
      else if (!param.get_name().compare("angle_start"))
      {
        // TODO cleanup
        double angle_start = param.as_double();
        double angle_end;
        this->get_parameter<double>("angle_end", angle_end);
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
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("angle_end"))
      {
        // TODO cleanup
        double angle_end = param.as_double();
        double angle_start;
        this->get_parameter<double>("angle_start", angle_start);
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
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("time_offset"))
      {
        m_time_offset = param.as_double();
      }
      else if (!param.get_name().compare("general_system_state"))
      {
        // TODO improve
        m_communications_settings.features =
          (m_communications_settings.features & ~(1UL << 0)) | (param.as_bool() << 0);
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("derived_settings"))
      {
        m_communications_settings.features =
          (m_communications_settings.features & ~(1UL << 1)) | (param.as_bool() << 1);
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("measurement_data"))
      {
        m_communications_settings.features =
          (m_communications_settings.features & ~(1UL << 2)) | (param.as_bool() << 2);
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("intrusion_data"))
      {
        m_communications_settings.features =
          (m_communications_settings.features & ~(1UL << 3)) | (param.as_bool() << 3);
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("application_io_data"))
      {
        m_communications_settings.features =
          (m_communications_settings.features & ~(1UL << 4)) | (param.as_bool() << 4);
        update_sensor_config = true;
      }
      else if (!param.get_name().compare("min_intensities"))
      {
        m_min_intensities = param.as_double();
      }
      else
      {
        result.successful = false;
        result.reason     = "Parameter is not dynamic reconfigurable";
        RCLCPP_WARN(this->get_logger(),
                    "Parameter %s not dynamically reconfigurable",
                    param.get_name().c_str());
      }
    }
  }
  if (update_sensor_config)
  {
    m_device->changeSensorSettings(m_communications_settings);
  }
  m_msg_creator = std::make_unique<sick::MessageCreator>(
    m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset, m_min_intensities);
  return result;
}


void SickSafetyscannersLifeCycle::receiveUDPPaket(const sick::datastructure::Data& data)
{
  if (!m_msg_creator)
  {
    RCLCPP_WARN(get_logger(),
                "Received UDPP packet before all objects were instantiated, ignoring this packet.");
    return;
  }

  if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty() &&
      m_msg_creator)
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

bool SickSafetyscannersLifeCycle::getConfigMetadata(
  const std::shared_ptr<sick_safetyscanners2_interfaces::srv::ConfigMetadata::Request> request,
  std::shared_ptr<sick_safetyscanners2_interfaces::srv::ConfigMetadata::Response> response)
{
  // Suppress warning of unused request variable due to empty request fields
  (void)request;

  auto metadata = sick::datastructure::ConfigMetadata();
  m_device->requestConfigMetadata(metadata);
  response->version_c_version = metadata.getVersionCVersion();
  response->major_version_number = metadata.getVersionMajorVersionNumber();
  response->minor_version_number = metadata.getVersionMinorVersionNumber();
  response->release_version_number = metadata.getVersionReleaseNumber();
  response->modification_time_date = metadata.getModificationTimeDate();
  response->modification_time_time = metadata.getModificationTimeTime();
  response->transfer_time_date = metadata.getTransferTimeDate();
  response->transfer_time_time = metadata.getTransferTimeTime();
  response->app_checksum = metadata.getAppChecksum();
  response->overall_checksum = metadata.getOverallChecksum();
  response->integrity_hash = metadata.getIntegrityHash();
  return true;
}

bool SickSafetyscannersLifeCycle::getFieldData(
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

bool SickSafetyscannersLifeCycle::getApplicationName(
  const std::shared_ptr<sick_safetyscanners2_interfaces::srv::ApplicationName::Request> request,
  std::shared_ptr<sick_safetyscanners2_interfaces::srv::ApplicationName::Response> response)
{
  // Suppress warning of unused request variable due to empty request fields
  (void)request;

  auto app_name = sick::datastructure::ApplicationName();
  m_device->requestApplicationName(app_name);
  response->version_c_version = app_name.getVersionCVersion();
  response->major_version_number = app_name.getVersionMajorVersionNumber();
  response->minor_version_number = app_name.getVersionMinorVersionNumber();
  response->release_version_number = app_name.getVersionReleaseNumber();
  response->name_length = app_name.getNameLength();
  response->application_name = app_name.getApplicationName();
  
  return true;
}

bool SickSafetyscannersLifeCycle::getTypeCode(
  const std::shared_ptr<sick_safetyscanners2_interfaces::srv::TypeCode::Request> request,
  std::shared_ptr<sick_safetyscanners2_interfaces::srv::TypeCode::Response> response)
{
  // Suppress warning of unused request variable due to empty request fields
  (void)request;

  auto type_code = sick::datastructure::TypeCode();
  m_device->requestTypeCode(type_code);
  response->type_code = type_code.getTypeCode();
  response->interface_type = type_code.getInterfaceType();
  response->max_range = type_code.getMaxRange();  
  
  return true;
}


} // namespace sick

RCLCPP_COMPONENTS_REGISTER_NODE(sick::SickSafetyscannersLifeCycle)
