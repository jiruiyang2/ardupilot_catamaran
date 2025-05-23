/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

/*
  define common vehicle build types.
  Also note that code needs to support other APM_BUILD_DIRECTORY
  values for example sketches
 */
// @LoggerEnum: APM_BUILD
#define APM_BUILD_Rover      1
#define APM_BUILD_ArduCopter     2
#define APM_BUILD_ArduPlane      3
#define APM_BUILD_AntennaTracker 4
#define APM_BUILD_UNKNOWN        5
#define APM_BUILD_Replay         6
#define APM_BUILD_ArduSub        7
#define APM_BUILD_iofirmware     8
#define APM_BUILD_AP_Periph      9
#define APM_BUILD_AP_DAL_Standalone 10
#define APM_BUILD_AP_Bootloader  11
#define APM_BUILD_Blimp      12
#define APM_BUILD_Heli       13
// @LoggerEnumEnd

#ifdef APM_BUILD_DIRECTORY
/*
  using this macro catches cases where we try to check vehicle type on
  build systems that don't support it
 */
#define APM_BUILD_TYPE(type) ((type) == APM_BUILD_DIRECTORY)

/*
  Copter and heli share a lot of code. This macro makes it easier to check for both
*/
#define APM_BUILD_COPTER_OR_HELI (APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Heli))

#else
#ifndef APM_BUILD_TYPE
#define APM_BUILD_TYPE(type) 0
#endif
#ifndef APM_BUILD_COPTER_OR_HELI
#define APM_BUILD_COPTER_OR_HELI 0
#endif
#endif
