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
  hxResult hxs_sim_info(hxsSimInfo *_siminfo)
  {
    const std::string service = "/haptix/gazebo/hxs_sim_info";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxSimInfo rep;
    return hxs_call(service, __func__, req, rep, _siminfo, hxs_convertSimInfo);
  }

  //////////////////////////////////////////////////
  hxResult hxs_camera_transform(hxsTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_camera_transform";
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxTransform rep;
    return hxs_call(service, __func__, req, rep, _transform,
      hxs_convertTransform);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_camera_transform(const hxsTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_set_camera_transform";
    haptix::comm::msgs::hxTransform req;

    if (!hxs_convertTransform(_transform, &req))
      return hxERROR;

    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_contacts(const char *_model, hxsContactPoints *_contact)
  {
    const std::string service = "/haptix/gazebo/hxs_contacts";
    haptix::comm::msgs::hxString req;
    req.set_data(_model);
    haptix::comm::msgs::hxContactPoint_V rep;
    return hxs_call(service, __func__, req, rep, _contact,
      hxs_convertContactPoints);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_joint_state(const char *_model, const char *_joint,
    float _pos, float _vel)
  {
    if (!_model || !_joint)
    {
      std::cerr << "hxs_state() error: required string is NULL" << std::endl;
      return hxERROR;
    }

    const std::string service = "/haptix/gazebo/hxs_set_model_joint_state";
    haptix::comm::msgs::hxModel req;
    req.set_name(_model);
    req.add_joints();
    req.mutable_joints(0)->set_name(_joint);
    req.mutable_joints(0)->set_pos(_pos);
    req.mutable_joints(0)->set_vel(_vel);

    // Set empty required fields
    req.set_id(0);
    req.set_gravity_mode(0);
    hxsTransform tf;
    hxsWrench wrench;
    hxs_convertTransform(&tf, req.mutable_transform());
    hxs_convertWrench(&wrench,
      req.mutable_joints(0)->mutable_wrench_reactive());
    req.mutable_joints(0)->set_torque_motor(0);

    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_link_state(const char *_model, const char *_link,
      const hxsTransform *_transform, const hxsVector3 *_lin_vel,
      const hxsVector3 *_ang_vel)
  {
    if (!_model || !_link)
    {
      std::cerr << "hxs_state() error: required string is NULL" << std::endl;
      return hxERROR;
    }
    if (!_transform || !_lin_vel || !_ang_vel)
    {
      std::cerr << "hxs_state() error: required struct is NULL" << std::endl;
      return hxERROR;
    }

    const std::string service = "/haptix/gazebo/hxs_set_model_link_state";
    haptix::comm::msgs::hxModel req;
    req.set_name(_model);
    req.add_links();
    req.mutable_links(0)->set_name(_link);
    if (!hxs_convertTransform(_transform,
      req.mutable_links(0)->mutable_transform()))
    {
      return hxERROR;
    }
    if (!hxs_convertVector3(_lin_vel, req.mutable_links(0)->mutable_lin_vel()))
    {
      return hxERROR;
    }
    if (!hxs_convertVector3(_ang_vel, req.mutable_links(0)->mutable_ang_vel()))
    {
      return hxERROR;
    }
    req.set_id(0);
    req.set_gravity_mode(0);
    hxsTransform tf;
    hxsVector3 v;
    hxs_convertTransform(&tf, req.mutable_transform());
    hxs_convertVector3(&v, req.mutable_links(0)->mutable_lin_acc());
    hxs_convertVector3(&v, req.mutable_links(0)->mutable_ang_acc());

    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_add_model(const char *_sdf, const char *_name,
    float _x, float _y, float _z, float _roll, float _pitch, float _yaw,
    int _gravity_mode, hxsModel *_model)
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
    req.set_string_value(_sdf);
    req.set_gravity_mode(_gravity_mode);

    return hxs_call(service, __func__, req, rep, _model, hxs_convertModel);
  }

  //////////////////////////////////////////////////
  hxResult hxs_remove_model(const char *_name)
  {
    const std::string service = "/haptix/gazebo/hxs_remove_model";
    haptix::comm::msgs::hxString req;
    req.set_data(_name);
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_transform(const char *_name,
    const hxsTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_set_model_transform";
    haptix::comm::msgs::hxParam req;
    req.set_name(_name);
    if (!hxs_convertTransform(_transform, req.mutable_transform()))
      return hxERROR;
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_transform(const char *_name, hxsTransform *_transform)
  {
    const std::string service = "/haptix/gazebo/hxs_model_transform";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxTransform rep;
    req.set_data(_name);

    return hxs_call(service, __func__, req, rep, _transform,
      hxs_convertTransform);
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_gravity_mode(const char *_name, int *_gravity_mode)
  {
    const std::string service = "/haptix/gazebo/hxs_model_gravity_mode";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxInt rep;
    req.set_data(_name);
    return hxs_call(service, __func__, req, rep, _gravity_mode,
      hxs_convertScalar);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_gravity_mode(const char *_name,
    const int _gravity_mode)
  {
    const std::string service = "/haptix/gazebo/hxs_set_model_gravity_mode";
    haptix::comm::msgs::hxParam req;
    req.set_name(_name);
    req.set_gravity_mode(_gravity_mode);

    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_linear_velocity(const char *_name, hxsVector3 *_velocity)
  {
    const std::string service = "/haptix/gazebo/hxs_linear_velocity";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxVector3 rep;
    req.set_data(_name);
    return hxs_call(service, __func__, req, rep, _velocity, hxs_convertVector3);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_linear_velocity(const char *_name,
    const hxsVector3 *_lin_vel)
  {
    const std::string service = "/haptix/gazebo/hxs_set_linear_velocity";
    haptix::comm::msgs::hxParam req;
    req.set_name(_name);
    if (!hxs_convertVector3(_lin_vel, req.mutable_vector3()))
      return hxERROR;
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_angular_velocity(const char *_name, hxsVector3 *_ang_vel)
  {
    const std::string service = "/haptix/gazebo/hxs_angular_velocity";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxVector3 rep;
    req.set_data(_name);
    return hxs_call(service, __func__, req, rep, _ang_vel, hxs_convertVector3);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_angular_velocity(const char *_name,
    const hxsVector3 *_ang_vel)
  {
    const std::string service = "/haptix/gazebo/hxs_set_angular_velocity";
    haptix::comm::msgs::hxParam req;
    req.set_name(_name);
    if (!hxs_convertVector3(_ang_vel, req.mutable_vector3()))
      return hxERROR;
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_apply_force(const char *_modelName, const char *_linkName,
      const hxsVector3 *_force, const float _duration)
  {
    const std::string service = "/haptix/gazebo/hxs_apply_force";
    haptix::comm::msgs::hxParam req;
    req.set_name(_modelName);
    req.set_string_value(_linkName);
    req.set_float_value(_duration);
    if (!hxs_convertVector3(_force, req.mutable_vector3()))
      return hxERROR;
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_apply_torque(const char *_modelName, const char *_linkName,
      const hxsVector3 *_torque, const float _duration)
  {
    const std::string service = "/haptix/gazebo/hxs_apply_torque";
    haptix::comm::msgs::hxParam req;
    req.set_name(_modelName);
    req.set_string_value(_linkName);
    req.set_float_value(_duration);
    if (!hxs_convertVector3(_torque, req.mutable_vector3()))
      return hxERROR;
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_apply_wrench(const char *_modelName, const char *_linkName,
      const hxsWrench *_wrench, const float _duration)
  {
    const std::string service = "/haptix/gazebo/hxs_apply_wrench";
    haptix::comm::msgs::hxParam req;
    req.set_name(_modelName);
    req.set_string_value(_linkName);
    req.set_float_value(_duration);
    if (!hxs_convertWrench(_wrench, req.mutable_wrench()))
      return hxERROR;
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset(int _resetLimbPose)
  {
    const std::string service = "/haptix/gazebo/hxs_reset";
    haptix::comm::msgs::hxInt req;
    req.set_data(_resetLimbPose);
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_logging(const char *_filename)
  {
    const std::string service = "/haptix/gazebo/hxs_start_logging";
    haptix::comm::msgs::hxString req;

    req.set_data(std::string(_filename));
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_is_logging(int *_result)
  {
    const std::string service = "/haptix/gazebo/hxs_is_logging";
    haptix::comm::msgs::hxInt rep;
    return hxs_call(service, __func__, haptix::comm::msgs::hxEmpty(), rep,
      _result, hxs_convertScalar);
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_logging()
  {
    const std::string service = "/haptix/gazebo/hxs_stop_logging";
    return hxs_call(service, __func__, haptix::comm::msgs::hxEmpty(),
      haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_color(const char *_model, const hxsColor *_color)
  {
    const std::string service = "/haptix/gazebo/hxs_set_model_color";
    haptix::comm::msgs::hxParam req;

    req.set_name(_model);
    req.mutable_color()->set_r(_color->r);
    req.mutable_color()->set_g(_color->g);
    req.mutable_color()->set_b(_color->b);
    req.mutable_color()->set_alpha(_color->alpha);
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_color(const char *_model, hxsColor *_color)
  {
    const std::string service = "/haptix/gazebo/hxs_model_color";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxColor rep;

    req.set_data(_model);
    return hxs_call(service, __func__, req, rep, _color, hxs_convertColor);
  }

  //////////////////////////////////////////////////
  hxResult hxs_set_model_collide_mode(const char *_model,
    const hxsCollideMode *_collide_mode)
  {
    const std::string service = "/haptix/gazebo/hxs_set_model_collide_mode";
    haptix::comm::msgs::hxParam req;

    if (!_collide_mode)
    {
      printf("hxs_set_model_collide_mode() error: NULL collide mode\n");
      return hxERROR;
    }

    req.set_name(_model);
    switch (*_collide_mode)
    {
      case hxsNOCOLLIDE:
        req.mutable_collide_mode()->set_mode(
          haptix::comm::msgs::hxCollideMode::hxsNOCOLLIDE);
        break;
      case hxsDETECTIONONLY:
        req.mutable_collide_mode()->set_mode(
          haptix::comm::msgs::hxCollideMode::hxsDETECTIONONLY);
        break;
      case hxsCOLLIDE:
        req.mutable_collide_mode()->set_mode(
          haptix::comm::msgs::hxCollideMode::hxsCOLLIDE);
        break;
      default:
        printf("hxs_set_model_collide_mode() Unknown collide_mode [%d]\n",
          *_collide_mode);
        return hxERROR;
    }
    return hxs_call(service, __func__, req, haptix::comm::msgs::hxEmpty());
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_collide_mode(const char *_model,
    hxsCollideMode *_collide_mode)
  {
    const std::string service = "/haptix/gazebo/hxs_model_collide_mode";
    haptix::comm::msgs::hxString req;
    haptix::comm::msgs::hxCollideMode rep;

    req.set_data(_model);
    return hxs_call(service, __func__, req, rep, _collide_mode,
      hxs_convertCollisionMode);
  }
}
