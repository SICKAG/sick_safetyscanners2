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
 * \file MessageCreator.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2020-12-08
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS2_UTILS_MESSAGECREATOR_H
#define SICK_SAFETYSCANNERS2_UTILS_MESSAGECREATOR_H

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

#endif // SICK_SAFETYSCANNERS2_UTILS_MESSAGECREATOR_H
