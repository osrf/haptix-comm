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
  ACI_WristDev         =  0,
  ACI_WristShell       =  1,
  ACI_MaleForearm      =  2,
  ACI_IndDistal        =  3,
  ACI_IndMedial        =  4,
  ACI_IndMetaCarpal    =  5,
  ACI_IndProximal      =  6,
  ACI_LittleDistal     =  7,
  ACI_LittleMedial     =  8,
  ACI_LittleMetaCarpal =  9,
  ACI_LittleProximal   = 10,
  ACI_MidDistal        = 12,
  ACI_MidMedial        = 12,
  ACI_MidMetaCarpal    = 13,
  ACI_MidProximal      = 14,
  ACI_Palm             = 15,
  ACI_PlanetaryAsm     = 16,
  ACI_RingDistal       = 17,
  ACI_RingMedial       = 18,
  ACI_RingMetaCarpal   = 19,
  ACI_RingProximal     = 20,
  ACI_ThDistal         = 21,
  ACI_ThProximal1      = 22,
  ACI_ThProximal2      = 23,
  ACI_num_links        = 24
} AplLinkId;

typedef enum
{
  ACI_wrist_rot       =  0,
  ACI_wrist_dev       =  1,
  ACI_wrist_fe        =  2,
  ACI_index_dip       =  3,
  ACI_index_mcp       =  4,
  ACI_index_pip       =  5,
  ACI_little_dip      =  6,
  ACI_little_mcp      =  7,
  ACI_little_pip      =  8,
  ACI_middle_dip      =  9,
  ACI_middle_mcp      = 10,
  ACI_middle_pip      = 11,
  ACI_thumb_cmc_ab_ad = 12,
  ACI_little_ab_ad    = 13,
  ACI_ring_ab_ad      = 14,
  ACI_middle_ab_ad    = 15,
  ACI_index_ab_ad     = 16,
  ACI_thumb_cmc_fe    = 17,
  ACI_ring_dip        = 18,
  ACI_ring_mcp        = 19,
  ACI_ring_pip        = 20,
  ACI_thumb_mcp       = 21,
  ACI_thumb_dip       = 22,
  ACI_num_joints      = 23
} AplJointId;

typedef enum
{
  ACI_wrist_rot_act       =  0,
  ACI_wrist_dev_act       =  1,
  ACI_wrist_fe_act        =  2,
  ACI_index_dip_act       =  3,
  ACI_index_mcp_act       =  4,
  ACI_index_pip_act       =  5,
  ACI_little_dip_act      =  6,
  ACI_little_mcp_act      =  7,
  ACI_little_pip_act      =  8,
  ACI_middle_dip_act      =  9,
  ACI_middle_mcp_act      = 10,
  ACI_middle_pip_act      = 11,
  ACI_thumb_cmc_ab_ad_act = 12,
  ACI_little_ab_ad_act    = 13,
  ACI_ring_ab_ad_act      = 14,
  ACI_middle_ab_ad_act    = 15,
  ACI_index_ab_ad_act     = 16,
  ACI_thumb_cmc_fe_act    = 17,
  ACI_ring_dip_act        = 18,
  ACI_ring_mcp_act        = 19,
  ACI_ring_pip_act        = 20,
  ACI_thumb_mcp_act       = 21,
  ACI_thumb_dip_act       = 22,
  ACI_num_actuators       = 23
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
  struct JointState state[ACI_num_joints];
};

struct AplRobotCommand
{
  struct JointCommand command[ACI_num_joints];
};

#endif
