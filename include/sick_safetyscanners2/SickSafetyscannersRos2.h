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
 * \file SickSafetyscannersRos2.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2020-12-08
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS2_SICKSAFETYSCANNERSROS2_H
#define SICK_SAFETYSCANNERS2_SICKSAFETYSCANNERSROS2_H

#include <sick_safetyscanners_base/SickSafetyscanners.h>
#include <sick_safetyscanners_base/Types.h>
#include <sick_safetyscanners_base/datastructure/Data.h>

#include <sick_safetyscanners2_interfaces/msg/extended_laser_scan.hpp>
#include <sick_safetyscanners2_interfaces/msg/output_paths.hpp>
#include <sick_safetyscanners2_interfaces/srv/field_data.hpp>

#include <sick_safetyscanners2/utils/Conversions.h>
#include <sick_safetyscanners2/utils/MessageCreator.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace sick {

class SickSafetyscannersRos2 : public rclcpp::Node
{
public:
  SickSafetyscannersRos2();

private:
  void receiveUDPPaket(const sick::datastructure::Data& data);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;
  rclcpp::Publisher<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>::SharedPtr
    m_extended_laser_scan_publisher;
  rclcpp::Publisher<sick_safetyscanners2_interfaces::msg::OutputPaths>::SharedPtr
    m_output_paths_publisher;
  rclcpp::Publisher<sick_safetyscanners2_interfaces::msg::RawMicroScanData>::SharedPtr
    m_raw_data_publisher;
  rclcpp::Service<sick_safetyscanners2_interfaces::srv::FieldData>::SharedPtr m_field_data_service;

  std::unique_ptr<sick::AsyncSickSafetyScanner> m_device;
  sick::datastructure::CommSettings m_communications_settings;

  std::unique_ptr<sick::MessageCreator> m_msg_creator;

  boost::asio::ip::address_v4 m_sensor_ip;
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

  void initialize_parameters();
  void load_parameters();


  // bool getFieldData();
  bool getFieldData(
    const std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Request> request,
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response> response);
  void readTypeCodeSettings();
  void readPersistentConfig();

  // void getMedianReflectors(const sick::datastructure::Data);
};
} // namespace sick

#endif // SICK_SAFETYSCANNERS2_SICKSAFETYSCANNERSROS2_H
