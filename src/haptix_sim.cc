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

#include <ignition/transport.hh>
#include "haptix/comm/haptix.h"
#include "haptix/comm/haptix_sim.h"
#include "msg/hxCamera.pb.h"
#include "msg/hxContact.pb.h"
#include "msg/hxEmpty.pb.h"
#include "msg/hxJoint.pb.h"
#include "msg/hxLink.pb.h"
#include "msg/hxModel.pb.h"
#include "msg/hxQuaternion.pb.h"
#include "msg/hxSimInfo.pb.h"
#include "msg/hxTransform.pb.h"
#include "msg/hxVector3.pb.h"

extern "C" {

  /// \brief Timeout used for the service requests (ms.).
  static unsigned int kTimeout = 1000;

  /// \brief ignition transport node.
  ignition::transport::Node *haptixSimNode = NULL;

  /// \brief Error string, to be retrieved by hx_last_result()
  std::string hxs_lastResult;
  std::mutex hxs_lastResultLock;

  //////////////////////////////////////////////////
  /// \brief Private function that creates an Ignition Transport node
  /// or return a pointer to it if has been already created.
  static ignition::transport::Node *getHxNodeInstance()
  {
    if (!haptixSimNode)
      haptixSimNode = new ignition::transport::Node();

    return haptixSimNode;
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxVector3 message to a
  /// C struct hxVector3.
  static void convertVector3(const haptix::comm::msgs::hxVector3 _in,
    hxVector3 *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxVector3));

    _out->x = _in.x();
    _out->y = _in.y();
    _out->z = _in.z();
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxQuaternion message to a
  /// C struct hxQuaternion.
  static void convertQuaternion(const haptix::comm::msgs::hxQuaternion _in,
    hxQuaternion *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxQuaternion));

    _out->w = _in.w();
    _out->x = _in.x();
    _out->y = _in.y();
    _out->z = _in.z();
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxQuaternion message to a
  /// C struct hxQuaternion.
  static void convertTransform(const haptix::comm::msgs::hxTransform _in,
    hxTransform *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxTransform));

    convertVector3(_in.pos(), &(_out->pos));
    convertQuaternion(_in.orient(), &(_out->orient));
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxJoint message to a
  /// C struct hxJoint.
  static void convertJoint(const haptix::comm::msgs::hxJoint _in, hxJoint *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxJoint));

    _out->pos = _in.pos();
    _out->vel = _in.vel();
    _out->acc = _in.acc();
    _out->torque_motor = _in.torque_motor();
    _out->torque_passive = _in.torque_passive();
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxLink message to a
  /// C struct hxLink.
  static void convertLink(const haptix::comm::msgs::hxLink _in, hxLink *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxLink));

    convertTransform(_in.transform(), &(_out->transform));
    convertVector3(_in.linvel(), &(_out->linvel));
    convertVector3(_in.angvel(), &(_out->angvel));
    convertVector3(_in.linacc(), &(_out->linacc));
    convertVector3(_in.angacc(), &(_out->angacc));
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxModel message to a
  /// C struct hxModel.
  static void convertModel(const haptix::comm::msgs::hxModel _in, hxModel *_out)
  {
    // Deallocate the memory for the links.
    for (int i = 0; i < _out->link_count; ++i)
      free(_out->links[i]);
    free(_out->links);

    // Deallocate the memory for the joints.
    for (int i = 0; i < _out->joint_count; ++i)
      free(_out->joints[i]);
    free(_out->joints);

    // Initialize the C struct.
    memset(_out, 0, sizeof(hxModel));

    convertTransform(_in.transform(), &(_out->transform));
    _out->is_static = _in.is_static();
    _out->id = _in.id();
    _out->link_count = _in.links_size();
    _out->joint_count = _in.joints_size();

    // Allocate memory for the link pointers.
    _out->links = NULL;
    _out->links =
      static_cast<hxLink**>(malloc((_out->link_count) * sizeof(hxLink*)));

    // Allocate memory for the links and fill them.
    for (int i = 0; i < _out->link_count; ++i)
    {
      _out->links[i] = static_cast<hxLink*>(malloc(sizeof(hxLink)));
      convertLink(_in.links(i), _out->links[i]);
    }

    // Allocate memory for the joint pointers.
    _out->joints = NULL;
    _out->joints =
      static_cast<hxJoint**>(malloc((_out->joint_count) * sizeof(hxJoint*)));

    // Allocate memory for the joints.
    for (int i = 0; i < _out->joint_count; ++i)
    {
      _out->joints[i] = static_cast<hxJoint*>(malloc(sizeof(hxJoint)));
      convertJoint(_in.joints(i), _out->joints[i]);
    }
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxContact message to a
  /// C struct hxContact.
  /*static void convertContact(const haptix::comm::msgs::hxContact _in,
    hxContact *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxContact));

    _out->contactCount = _in.body1_size();

    for (int i = 0; i < _out->contactCount; ++i)
    {
      _out->body1[i] = _in.body1(i);
      _out->body2[i] = _in.body2(i);
      convertVector3(_in.point(i), &(_out->point[i]));
      convertVector3(_in.normal(i), &(_out->normal[i]));
      convertVector3(_in.tangent1(i), &(_out->tangent1[i]));
      convertVector3(_in.tangent2(i), &(_out->tangent2[i]));
      _out->distance[i] = _in.distance(i);
      convertVector3(_in.velocity(i), &(_out->velocity[i]));
      convertVector3(_in.force(i), &(_out->force[i]));
    }
  }*/

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxCamera message to a
  /// C struct hxCamera.
  static void convertCamera(const haptix::comm::msgs::hxCamera _in,
    hxCamera *_out)
  {
    // Initialize the C struct.
    memset(_out, 0, sizeof(hxCamera));

    convertTransform(_in.transform(), &(_out->transform));
  }

  //////////////////////////////////////////////////
  /// \brief Private function that converts a protobuf hxSimInfo message to a
  /// C struct hxSimInfo.
  static void convertSimInfo(const haptix::comm::msgs::hxSimInfo _in,
    hxSimInfo *_out)
  {
    // Deallocate the memory for the models first.
    for (int i = 0; i < _out->modelCount; ++i)
    {
      for (int j = 0; j < _out->models[i]->link_count; ++j)
        free(_out->models[i]->links[j]);
      free(_out->models[i]->links);

      for (int j = 0; j < _out->models[i]->joint_count; ++j)
        free(_out->models[i]->joints[j]);
      free(_out->models[i]->joints);

      free(_out->models[i]);
    }
    free(_out->models);

    // Initialize the C struct.
    memset(_out, 0, sizeof(hxSimInfo));

    _out->modelCount = _in.models_size();
    _out->models = NULL;

    // Allocate memory for the model pointers.
    _out->models =
      static_cast<hxModel**>(malloc((_out->modelCount) * sizeof(hxModel*)));

    // Allocate memory for the models and fill them.
    for (int i = 0; i < _out->modelCount; ++i)
    {
      _out->models[i] = static_cast<hxModel*>(malloc(sizeof(hxModel)));\
      convertModel(_in.models(i), _out->models[i]);
    }

    // Fill the camera.
    convertCamera(_in.camera(), _out->camera);
  }

  //////////////////////////////////////////////////
  hxResult hxs_siminfo(hxSimInfo * _siminfo)
  {
    haptix::comm::msgs::hxEmpty req;
    haptix::comm::msgs::hxSimInfo rep;
    bool result;
    ignition::transport::Node *hxNode = getHxNodeInstance();

    // Request the service.
    std::string service = "/haptix/gazebo/siminfo";
    bool executed = hxNode->Request(service, req, kTimeout, rep, result);

    if (executed)
    {
      if (result)
      {
        // Fill the struct with the response.
        convertSimInfo(rep, _siminfo);

        return hxOK;
      }
      else
      {
        std::lock_guard<std::mutex> lock(hxs_lastResultLock);
        hxs_lastResult = "hxs_siminfo() Service call failed.";
        std::cerr << hxs_lastResult << std::endl;
      }
    }
    else
    {
      std::lock_guard<std::mutex> lock(hxs_lastResultLock);
      hxs_lastResult = "hxs_siminfo() Service call timed out.";
      std::cerr << hxs_lastResult << std::endl;
    }

    return hxERROR;
  }

  //////////////////////////////////////////////////
  hxResult hxs_camera(hxCamera * /*_camera*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_camera_transform(hxTransform /*_transform*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_contacts(hxContact * /*_contact*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_jacobian(const hxLink * /*_link*/, const float * /*_point*/,
                        float * /*_jacobian*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_state(const hxModel * /*_model*/, const hxJoint * /*_joint*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxModel *hxs_add_model(const char * /*_urdf*/, float /*_x*/, float /*_y*/,
                         float /*_z*/, float /*_roll*/, float /*_pitch*/,
                         float /*_yaw*/)
  {
    return 0;
  }

  //////////////////////////////////////////////////
  hxResult hxs_remove_model_id(int /*_id*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_remove_model(const hxModel * /*_model*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_model_transform(int /*_id*/, hxTransform /*_transform*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_linear_velocity(int /*_id*/, hxVector3 /*_velocity*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_angular_velocity(int /*_id*/, hxVector3 /*_velocity*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_linear_accel(int /*_id*/, hxVector3 /*_accel*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_angular_accel(int /*_id*/, hxVector3 /*_accel*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_force(const hxLink * /*_link*/, hxVector3 /*_force*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_torque(const hxLink * /*_link*/, hxVector3 /*_torque*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset(int /*_resetLimbPose*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_reset_timer()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_timer()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_timer()
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  hxResult hxs_start_logging(const char * /*_filename*/)
  {
    return hxOK;
  }

  //////////////////////////////////////////////////
  int hxs_is_logging()
  {
    return 0;
  }

  //////////////////////////////////////////////////
  hxResult hxs_stop_logging()
  {
    return hxOK;
  }
}
