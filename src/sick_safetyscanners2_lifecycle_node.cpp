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
 * \file sick_safetyscanners2_lifecycle_node.cpp
 *
 * \authors Soma gallai<soma.gallai@cm-robotics.com>  Erwin Lejeune
 * <erwin.lejeune@cm-robotics.com> \date    2021-05-27
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/SickSafetyscannersLifeCycle.hpp>

#include <rclcpp/rclcpp.hpp>

#include <cstdio>

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<sick::SickSafetyscannersLifeCycle>(options);
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
