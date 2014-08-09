/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef __HAPTIX_COMM_HELPERS_HH_INCLUDED__
#define __HAPTIX_COMM_HELPERS_HH_INCLUDED__

/** \def HAPTIX_VISIBLE
 * Use to represent "symbol visible" if supported
 */

/** \def HAPTIX_HIDDEN
 * Use to represent "symbol hidden" if supported
 */

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define HAPTIX_VISIBLE __attribute__ ((dllexport))
    #else
      #define HAPTIX_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define HAPTIX_VISIBLE __attribute__ ((dllimport))
    #else
      #define HAPTIX_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define HAPTIX_HIDDEN
#else
  #if __GNUC__ >= 4
    #define HAPTIX_VISIBLE __attribute__ ((visibility ("default")))
    #define HAPTIX_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define HAPTIX_VISIBLE
    #define HAPTIX_HIDDEN
  #endif
#endif

#endif
