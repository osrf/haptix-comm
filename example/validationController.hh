/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef _HAPTIX_VALIDATION_CONTROLLER_
#define _HAPTIX_VALIDATION_CONTROLLER_

#include <mutex>
#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Timer.hh>
#include <ignition/transport/Node.hh>

  /// \brief Labels for states.
  static std::string kReadyState     = "controller_ready";
  static std::string kInitCondsState = "controller_init_conds";
  static std::string kRunningState   = "controller_running";
  static std::string kEndState       = "controller_end";

// Forward declarations.
class ValidationController;

namespace msgs
{
  class GzString;
}

/// \brief State pattern used for the state machine.
class State
{
  /// \brief Class constructor.
  /// \param[in] _name Name of the state.
  /// \param[in] _plugin Pointer to the model plugin.
  public: State(const std::string &_name,
                ValidationController &_plugin);

  /// \brief Update the state.
  public: void Update();

  /// \brief Called once before changing to a different state.
  public: void Teardown();

  /// \brief Equal to operator.
  /// \param[in] _state The state to compare against.
  /// \return true if the state has the same name.
  public: bool operator ==(const State &_state) const
  {
    return this->name == _state.name;
  }

  /// \brief Not equal to operator
  /// \param[in] _state The state to compare against
  /// \return true if the state doesn't have the same name.
  public: bool operator !=(const State &_state) const
  {
    return !(*this == _state);
  }

  /// \brief Return the current feedback value.
  protected: std::string GazeboState() const;

  /// \brief ToDo.
  private: virtual void DoInitialize();

  /// \brief ToDo.
  private: virtual void DoUpdate();

  /// \brief ToDo.
  private: virtual void DoTeardown();

  /// \brief ToDo.
  private: virtual void DoOnState();

  /// \brief Initialize the state. Called once after a pause duration after
  /// entering state.
  private: void Initialize();

  /// \brief Callback that updates the feedback member variable.
  private: void OnState(const gazebo::msgs::GzString &_msg);

   /// \brief Publish the current state.
  private: void PublishState();

  /// \brief Name of the state.
  public: const std::string name;

  /// \brief Pointer to the validation plugin.
  protected: ValidationController &plugin;

  /// \brief Timer to measure time in the current state.
  protected: gazebo::common::Timer timer;

  /// \brief Has initialized
  protected: bool initialized = false;

  /// \brief Last feedback message.
  private: std::string gazeboState;

  /// \brief Mutex to protect the feedback message.
  private: mutable std::mutex mutex;

  /// \brief Last time the we published the state.
  private: gazebo::common::Time lastPublication;

  /// \brief Elapsed time between state publications.
  private: gazebo::common::Time publicationInterval;

  /// \brief Transport node.
  protected: ignition::transport::Node node;
};

/// \brief State that handles the "ready" state.
class ReadyState : public State
{
  // Use class constructor from base class.
  using State::State;

  // Documentation inherited.
  public: virtual void DoInitialize();

  // Documentation inherited
  public: virtual void DoOnState();
};

/// \brief State that handles the "initConds" state.
class InitCondsState : public State
{
  // Use class constructor from base class.
  using State::State;

  // Documentation inherited.
  public: virtual void DoInitialize();

  // Documentation inherited.
  public: virtual void DoUpdate();
};

/// \brief State that handles the "running" state.
class RunningState : public State
{
  // Use class constructor from base class.
  using State::State;

  // Documentation inherited.
  public: virtual void DoInitialize();

  // Documentation inherited.
  public: virtual void DoUpdate();
};

/// \brief State that handles the "running" state.
class EndState : public State
{
  // Use class constructor from base class.
  using State::State;

  // Documentation inherited.
  public: virtual void DoInitialize();

  // Documentation inherited
  public: virtual void DoOnState();
};


/// Example SDF:
///       <plugin name="actuator_plugin" filename="libActuatorPlugin.so">
///        <actuator>
///          <name>actuator_0</name> <!-- optional -->
///          <joint>JOINT_0</joint> <!-- name of joint to actuate -->
///          <index>0</index> <!-- needed for multi-DOF joints -->
///          <type>electric_motor</type> <!-- motor model type -->
///          <power>20</power> <!-- parameters for motor model -->
///          <max_velocity>6</max_velocity>
///          <max_torque>10.0</max_torque>
///        </actuator>
///      </plugin>
///    </model>
///
/// Required fields:
/// - name
/// - joint
/// - index (can be 0 in most cases)
/// - type: current options are electric_motor, velocity_limiter or null
/// Required for motor model electric_motor:
/// - power
/// - max_velocity
/// - max_torque
/// Required for motor model velocity_limiter:
/// - max_velocity
/// - max_torque
/// \brief ToDo
class GAZEBO_VISIBLE ValidationController
{
  /// \brief Class constructor.
  public: ValidationController();

  /// \brief Class destructor.
  public: ~ValidationController();

  /// \brief Callback on world update event.
  public: void Start();

  /// \brief ToDo.
  public: void ChangeState(State &_newState);

  /// \brief State machine states.
  public: std::unique_ptr<ReadyState> readyState;
  public: std::unique_ptr<InitCondsState> initCondsState;
  public: std::unique_ptr<RunningState> runningState;
  public: std::unique_ptr<EndState> endState;

  /// \brief Pointer to the current state.
  public: State *currentState;
};

#endif
