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
    : Node("SickSafetyscannersRos2") {
  RCLCPP_INFO(this->get_logger(), "Initializing SickSafetyscannersRos2 Node");

  // read parameters!
  initializeParameters(*this);
  loadParameters(*this);

  // init publishers and services
  m_laser_scan_publisher =
      this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  m_extended_laser_scan_publisher = this->create_publisher<
      sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>("extended_scan",
                                                               1);
  m_output_paths_publisher =
      this->create_publisher<sick_safetyscanners2_interfaces::msg::OutputPaths>(
          "output_paths", 1);
  m_raw_data_publisher = this->create_publisher<
      sick_safetyscanners2_interfaces::msg::RawMicroScanData>("raw_data", 1);

  m_field_data_service =
      this->create_service<sick_safetyscanners2_interfaces::srv::FieldData>(
          "field_data",
          std::bind(&SickSafetyscannersRos2::getFieldData, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Dynamic Parameter Change client
  m_param_callback = add_on_set_parameters_callback(
      std::bind(&SickSafetyscannersRos2::parametersCallback, this,
                std::placeholders::_1));

  setupCommunication(std::bind(&SickSafetyscannersRos2::receiveUDPPaket, this,
                               std::placeholders::_1));

  startCommunication(this, m_laser_scan_publisher);

  RCLCPP_INFO(this->get_logger(), "Node Configured and running");
}

void SickSafetyscannersRos2::receiveUDPPaket(
    const sick::datastructure::Data &data) {
  if (!m_config.m_msg_creator) {
    RCLCPP_WARN(get_logger(), "Received UDPP packet before all objects were "
                              "instantiated, ignoring this packet.");
    return;
  }

  if (!data.getMeasurementDataPtr()->isEmpty() &&
      !data.getDerivedValuesPtr()->isEmpty()) {
    auto scan = m_config.m_msg_creator->createLaserScanMsg(data, this->now());
    m_diagnosed_laser_scan_publisher->publish(scan);

    sick_safetyscanners2_interfaces::msg::ExtendedLaserScan extended_scan =
        m_config.m_msg_creator->createExtendedLaserScanMsg(data, this->now());

    m_extended_laser_scan_publisher->publish(extended_scan);

    auto output_paths = m_config.m_msg_creator->createOutputPathsMsg(data);
    m_output_paths_publisher->publish(output_paths);
  }

  m_last_raw_msg = m_config.m_msg_creator->createRawDataMsg(data);
  m_raw_data_publisher->publish(m_last_raw_msg);
}
} // namespace sick
