/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "haptix/comm/haptix.h"
#include "haptix/comm/haptix_sim.h"
#include "haptix/comm/haptix_sim_utils.h"
#include "msg/hxContactPoint.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxInt.pb.h"
#include "msg/hxModel.pb.h"
#include "msg/hxParam.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxString.pb.h"
#include "msg/hxTime.pb.h"
#include "msg/hxTransform.pb.h"
#include "msg/hxVector3.pb.h"

extern "C" {
  //////////////////////////////////////////////////
  hxResult hxs_siminfo(hxSimInfo *_siminfo)
  {
    const std::string service = "/haptix/gazebo/hxs_siminfo";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxSimInfo rep;
    return hxs_call(service, __func__, req, rep, _siminfo, hxs_convertSimInfo);
  }

  //////////////////////////////////////////////////
  hxResult hxs_camera_transform(hxTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_camera_transform";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxTransform rep;
    return hxs_call(service, __func__, req, rep, _transform,
      hxs_convertTransform);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_camera_transform(const hxTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_set_camera_transform";
    haptix::comm::msgs::hxTransform req;
    haptix::comm::msgs::hxEmpty rep;

    if (!hxs_convertTransform(_transform, &req))
      return hxERROR;

    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_contacts(const char *_model, hxContactPoints *_contact)
  {
    const std::string service = "/haptix/gazebo/hxs_contacts";
    haptix::comm::msgs::hxString req;
    req.set_data(_model);
    haptix::comm::msgs::hxContactPoint_V rep;
    return hxs_call(service, __func__, req, rep, _contact,
      hxs_convertContactPoints);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_state(const hxModel *_model)
  {
    if (!_model)
    {
      std::cerr << "hxs_state() error: model is NULL" << std::endl;
      return hxERROR;
    }

    const std::string service = "/haptix/gazebo/hxs_set_state";
    haptix::comm::msgs::hxModel req;
    haptix::comm::msgs::hxEmpty rep;
    if (!hxs_convertModel(_model, &req))
      return hxERROR;

    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_add_model(const char *_urdf, const char *_name,
    float _x, float _y, float _z, float _roll, float _pitch, float _yaw,
    bool _gravity, hxModel *_model)
  {
    const std::string service = "/haptix/gazebo/hxs_add_model";
    haptix::comm::msgs::hxParam req;
    haptix::comm::msgs::hxModel rep;
    req.set_name(std::string(_name));
    req.mutable_vector3()->set_x(_x);
    req.mutable_vector3()->set_y(_y);
    req.mutable_vector3()->set_z(_z);
    req.mutable_orientation()->set_roll(_roll);
    req.mutable_orientation()->set_pitch(_pitch);
    req.mutable_orientation()->set_yaw(_yaw);
    req.set_string_value(_urdf);
    req.set_gravity(_gravity);

    return hxs_call(service, __func__, req, rep, _model, hxs_convertModel);
  }

  //////////////////////////////////////////////////
  hxResult hxs_remove_model(const char *_name)
  {
    const std::string service = "/haptix/gazebo/hxs_remove_model";
    haptix::comm::msgs::hxString req;
    req.set_data(_name);
    haptix::comm::msgs::hxEmpty rep;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_transform(const char *_name, const hxTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_model_transform";
    haptix::comm::msgs::hxParam req;
    haptix::comm::msgs::hxEmpty rep;
    req.set_name(_name);
    if (!hxs_convertTransform(_transform, req.mutable_transform()))
      return hxERROR;
    return hxs_call(service, __func__, req, rep);
  }
  //////////////////////////////////////////////////
  hxResult hxs_model_gravity(const char *_name, int *_gravity)
  {
    const std::string service = "/haptix/gazebo/hxs_model_gravity";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxInt rep;
    req.set_data(_name);
    return hxs_call(service, __func__, req, rep, _gravity, hxs_convertScalar);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_gravity(const char *_name, const int _gravity)
  {
    const std::string service = "/haptix/gazebo/hxs_set_model_gravity";
    haptix::comm::msgs::hxParam req;
    haptix::comm::msgs::hxEmpty rep;
    req.set_name(_name);
    req.set_int_value(_gravity);

    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_linear_velocity(const char *_name, const hxVector3 *_linvel)
  {
    const std::string service = "/haptix/gazebo/hxs_linear_velocity";
    haptix::comm::msgs::hxParam req;
    haptix::comm::msgs::hxEmpty rep;
    req.set_name(_name);
    if (!hxs_convertVector3(_linvel, req.mutable_vector3()))
      return hxERROR;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_angular_velocity(const char *_name, const hxVector3 *_angvel)
  {
    const std::string service = "/haptix/gazebo/hxs_angular_velocity";
    haptix::comm::msgs::hxParam req;
    haptix::comm::msgs::hxEmpty rep;
    req.set_name(_name);
    if (!hxs_convertVector3(_angvel, req.mutable_vector3()))
      return hxERROR;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_force(const char *_modelName, const char *_linkName,
      const hxVector3 *_force, const float _duration)
  {
    const std::string service = "/haptix/gazebo/hxs_force";
    haptix::comm::msgs::hxParam req;
    // Bit of a hack
    req.set_name(_modelName);
    req.set_string_value(_linkName);
    req.set_float_value(_duration);
    if (!hxs_convertVector3(_force, req.mutable_vector3()))
      return hxERROR;
    haptix::comm::msgs::hxEmpty rep;
    // TODO: send strings
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_torque(const char *_modelName, const char *_linkName,
      const hxVector3 *_torque, const float _duration)
  {
    const std::string service = "/haptix/gazebo/hxs_torque";
    haptix::comm::msgs::hxParam req;
    req.set_name(_modelName);
    req.set_string_value(_linkName);
    req.set_float_value(_duration);
    if (!hxs_convertVector3(_torque, req.mutable_vector3()))
      return hxERROR;
    haptix::comm::msgs::hxEmpty rep;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset(int _resetLimbPose)
  {
    const std::string service = "/haptix/gazebo/hxs_reset";
    haptix::comm::msgs::hxInt req;
    haptix::comm::msgs::hxEmpty rep;
    req.set_data(_resetLimbPose);
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset_timer()
  {
    const std::string service = "/haptix/gazebo/hxs_reset_timer";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxEmpty rep;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_timer()
  {
    const std::string service = "/haptix/gazebo/hxs_start_timer";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxEmpty rep;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_timer()
  {
    const std::string service = "/haptix/gazebo/hxs_stop_timer";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxEmpty rep;
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_timer(hxTime *_time)
  {
    const std::string service = "/haptix/gazebo/hxs_timer";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxTime rep;
    return hxs_call(service, __func__, req, rep, _time, hxs_convertTime);
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_logging(const char *_filename)
  {
    const std::string service = "/haptix/gazebo/hxs_start_logging";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxEmpty rep;

    req.set_data(std::string(_filename));
    return hxs_call(service, __func__, req, rep);
  }

  //////////////////////////////////////////////////
  hxResult hxs_is_logging(int *_result)
  {
    const std::string service = "/haptix/gazebo/hxs_is_logging";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxInt rep;
    return hxs_call(service, __func__, req, rep, _result, hxs_convertScalar);
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_logging()
  {
    const std::string service = "/haptix/gazebo/hxs_stop_logging";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxEmpty rep;
    return hxs_call(service, __func__, req, rep);
  }
}
