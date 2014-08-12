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

#ifndef APL_CONTROL_INTERFACE
#define APL_CONTROL_INTERFACE

typedef enum
{
  unknown_link     =  0,
  WristDev         =  1,
  WristShell       =  2,
  MaleForearm      =  3,
  IndDistal        =  4,
  IndMedial        =  5,
  IndMetaCarpal    =  6,
  IndProximal      =  7,
  LittleDistal     =  8,
  LittleMedial     =  9,
  LittleMetaCarpal = 10,
  LittleProximal   = 12,
  MidDistal        = 12,
  MidMedial        = 13,
  MidMetaCarpal    = 14,
  MidProximal      = 15,
  Palm             = 16,
  PlanetaryAsm     = 17,
  RingDistal       = 18,
  RingMedial       = 19,
  RingMetaCarpal   = 20,
  RingProximal     = 21,
  ThDistal         = 22,
  ThProximal1      = 23,
  ThProximal2      = 24,
  num_links
} AplLinkId;

typedef enum
{
  unknown_joint   =  0,
  wrist_rot       =  1,
  wrist_dev       =  2,
  wrist_fe        =  3,
  index_dip       =  4,
  index_mcp       =  5,
  index_pip       =  6,
  little_dip      =  7,
  little_mcp      =  8,
  little_pip      =  9,
  middle_dip      = 10,
  middle_mcp      = 11,
  middle_pip      = 12,
  thumb_cmc_ab_ad = 13,
  little_ab_ad    = 14,
  ring_ab_ad      = 15,
  middle_ab_ad    = 16,
  index_ab_ad     = 17,
  thumb_cmc_fe    = 18,
  ring_dip        = 19,
  ring_mcp        = 20,
  ring_pip        = 21,
  thumb_mcp       = 22,
  thumb_dip       = 23,
  num_joints
} AplJointId;

typedef enum
{
  unknown_actuator    =  0,
  wrist_rot_act       =  1,
  wrist_dev_act       =  2,
  wrist_fe_act        =  3,
  index_dip_act       =  4,
  index_mcp_act       =  5,
  index_pip_act       =  6,
  little_dip_act      =  7,
  little_mcp_act      =  8,
  little_pip_act      =  9,
  middle_dip_act      = 10,
  middle_mcp_act      = 11,
  middle_pip_act      = 12,
  thumb_cmc_ab_ad_act = 13,
  little_ab_ad_act    = 14,
  ring_ab_ad_act      = 15,
  middle_ab_ad_act    = 16,
  index_ab_ad_act     = 17,
  thumb_cmc_fe_act    = 18,
  ring_dip_act        = 19,
  ring_mcp_act        = 20,
  ring_pip_act        = 21,
  thumb_mcp_act       = 22,
  thumb_dip_act       = 23,
  num_actuators
} AplActuatorId;

struct JointState
{
  float position;
  float velocity;
  float effort;
};

struct JointCommand
{
  float position;
  float velocity;
  float effort;

  float kp_position;
  float ki_position;
  // float kd_position;
  float kp_velocity;
  float force;
};

struct AplRobotState
{
  struct JointState state[num_joints];
};

struct AplRobotCommand
{
  struct JointCommand command[num_joints];
};

#endif
