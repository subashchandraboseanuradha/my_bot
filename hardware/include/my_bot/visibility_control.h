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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef MY_BOT__VISIBILITY_CONTROL_H_
#define MY_BOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MY_BOT_EXPORT __attribute__((dllexport))
#define MY_BOT_IMPORT __attribute__((dllimport))
#else
#define MY_BOT_EXPORT __declspec(dllexport)
#define MY_BOT_IMPORT __declspec(dllimport)
#endif
#ifdef MY_BOT_BUILDING_DLL
#define MY_BOT_PUBLIC MY_BOT_EXPORT
#else
#define MY_BOT_PUBLIC MY_BOT_IMPORT
#endif
#define MY_BOT_PUBLIC_TYPE MY_BOT_PUBLIC
#define MY_BOT_LOCAL
#else
#define MY_BOT_EXPORT __attribute__((visibility("default")))
#define MY_BOT_IMPORT
#if __GNUC__ >= 4
#define MY_BOT_PUBLIC __attribute__((visibility("default")))
#define MY_BOT_LOCAL __attribute__((visibility("hidden")))
#else
#define MY_BOT_PUBLIC
#define MY_BOT_LOCAL
#endif
#define MY_BOT_PUBLIC_TYPE
#endif

#endif  // MY_BOT__VISIBILITY_CONTROL_H_