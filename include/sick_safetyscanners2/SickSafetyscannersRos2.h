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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>

#include "./SickSafetyscanners.hpp"

namespace sick {

class SickSafetyscannersRos2 : public rclcpp::Node, public SickSafetyscanners {
public:
  /*!
   * \brief Constructor of the ROS2 Node handling the Communication of the Sick
   * Safetyscanner
   */
  SickSafetyscannersRos2();

private:
  // Publishers
  rclcpp::Publisher<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>::
      SharedPtr m_extended_laser_scan_publisher;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      m_laser_scan_publisher;
  rclcpp::Publisher<sick_safetyscanners2_interfaces::msg::OutputPaths>::
      SharedPtr m_output_paths_publisher;
  rclcpp::Publisher<sick_safetyscanners2_interfaces::msg::RawMicroScanData>::
      SharedPtr m_raw_data_publisher;

  // Services
  rclcpp::Service<sick_safetyscanners2_interfaces::srv::FieldData>::SharedPtr
      m_field_data_service;

  // Callback function passed to the device for handling the received packages
  void receiveUDPPaket(const sick::datastructure::Data &data);
};
} // namespace sick

#endif // SICK_SAFETYSCANNERS2_SICKSAFETYSCANNERSROS2_H
