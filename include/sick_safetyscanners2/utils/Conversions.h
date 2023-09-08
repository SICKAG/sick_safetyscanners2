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
 * \file Conversions.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2020-12-09
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS2_UTILS_CONVERSIONS_H
#define SICK_SAFETYSCANNERS2_UTILS_CONVERSIONS_H

#include <sick_safetyscanners_base/Types.h>

namespace sick {

/*!
 * \brief Converts a boolean to a string containing true or false.
 * \param x boolean to convert to a string
 * \returns A string containing true or false.
 */
inline std::string btoa(bool x) { return ((x) ? "true" : "false"); }

/*!
 * \brief Converts degrees to radians.
 * \param deg Degrees to convert.
 * \return To radians converted degrees.
 */
inline float degToRad(float deg) { return deg * M_PI / 180.0f; }

/*!
 * \brief Converts radians to degrees.
 * \param rad Input radians to convert
 * \return To degrees converted radians
 */
inline float radToDeg(float rad) { return rad * 180.0f / M_PI; }

/*!
 * \brief Converts a skip value into a "publish frequency" value
 * \param skip The number of scans to skip between each measured scan.  For a
 * 25Hz laser, setting 'skip' to 0 makes it publish at 25Hz, 'skip' to 1 makes
 * it publish at 12.5Hz. \return "Publish Frequency" ie. One out of every n_th
 * scan will be published.  1 is publish every scan.  2 is publish at half rate,
 * and so on.
 */
inline uint16_t skipToPublishFrequency(int skip) { return skip + 1; }

} // End namespace sick
#endif // SICK_SAFETYSCANNERS2_UTILS_CONVERSIONS_H
