// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file was last modified by Michael Artuerhof
// (michael.artuerhof@gmail.com), on 2024-05-26.


/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ARTEBOT_CONTROL__VISIBILITY_CONTROL_H_
#define ARTEBOT_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ARTEBOT_CONTROL_EXPORT __attribute__((dllexport))
#define ARTEBOT_CONTROL_IMPORT __attribute__((dllimport))
#else
#define ARTEBOT_CONTROL_EXPORT __declspec(dllexport)
#define ARTEBOT_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef ARTEBOT_CONTROL_BUILDING_DLL
#define ARTEBOT_CONTROL_PUBLIC ARTEBOT_CONTROL_EXPORT
#else
#define ARTEBOT_CONTROL_PUBLIC ARTEBOT_CONTROL_IMPORT
#endif
#define ARTEBOT_CONTROL_PUBLIC_TYPE ARTEBOT_CONTROL_PUBLIC
#define ARTEBOT_CONTROL_LOCAL
#else
#define ARTEBOT_CONTROL_EXPORT __attribute__((visibility("default")))
#define ARTEBOT_CONTROL_IMPORT
#if __GNUC__ >= 4
#define ARTEBOT_CONTROL_PUBLIC __attribute__((visibility("default")))
#define ARTEBOT_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define ARTEBOT_CONTROL_PUBLIC
#define ARTEBOT_CONTROL_LOCAL
#endif
#define ARTEBOT_CONTROL_PUBLIC_TYPE
#endif

#endif  // ARTEBOT_CONTROL__VISIBILITY_CONTROL_H_
