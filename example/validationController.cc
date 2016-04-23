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

#include <mutex>
#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/transport/Node.hh>

/// \brief Labels for states.
static std::string kReadyState  = "controller_ready";
static std::string kInitialConditionsState  = "controller_initial_conditions";

// Forward declarations.
class ValidationController;

/// \brief State pattern used for the state machine.
class State
{
  /// \brief Class constructor.
  /// \param[in] _name Name of the state.
  /// \param[in] _plugin Pointer to the model plugin.
  public: State(const std::string &_name,
                ValidationController &_plugin)
    : name(_name),
      plugin(_plugin),
      publicationInterval(1.0)
  {
    // This is the topic where we receive the state from Gazebo.
    this->node.Subscribe("/gazebo/validation/state", &State::OnState, this);

    // Advertise controller feedback.
    this->node.Advertise<gazebo::msgs::GzString>("/gazebo/validation/feedback");
  }

  /// \brief Update the state.
  public: void Update()
  {
    //std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->initialized)
      this->Initialize();

    std::cout << this->name << std::endl;
    std::cout << "State::Update()" << std::endl;
    std::cout << timer.GetElapsed() << std::endl;

    this->DoUpdate();

    // Time to publish the current state?
    auto now = gazebo::common::Time::GetWallTime();
    if (now - this->lastPublication >= this->publicationInterval)
      this->PublishState();
  }

  /// \brief Called once before changing to a different state.
  public: void Teardown()
  {
    //std::lock_guard<std::mutex> lock(this->mutex);
    this->initialized = false;

    std::cout << "State::Teardown()" << std::endl;
    this->DoTeardown();

    this->PublishState();
  }

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

  /// \brief Return the current Gazebo state value.
  protected: std::string GazeboState() const
  {
    //std::lock_guard<std::mutex> lock(this->mutex);
    return this->gazeboState;
  }

  /// \brief ToDo.
  private: virtual void DoInitialize(){};

  /// \brief ToDo.
  private: virtual void DoUpdate(){};

  /// \brief ToDo.
  private: virtual void DoTeardown(){};

  /// \brief ToDo.
  private: virtual void DoOnState(){};

  /// \brief Initialize the state. Called once after a pause duration after
  /// entering state.
  private: void Initialize()
  {
    std::cout << "State::Initialize()" << std::endl;
    this->initialized = true;
    this->timer.Reset();
    this->timer.Start();
    this->DoInitialize();
  }

  /// \brief Callback that updates the Gazebo state member variable.
  private: void OnState(const gazebo::msgs::GzString &_msg)
  {
    //std::lock_guard<std::mutex> lock(this->mutex);
    this->gazeboState = _msg.data();

    // Only if we're the active state.
    if (this->initialized)
      this->DoOnState();
  }

   /// \brief Publish the current state.
  private: void PublishState()
  {
    gazebo::msgs::GzString msg;
    msg.set_data(this->name);
    this->node.Publish("/gazebo/validation/feedback", msg);
    this->lastPublication = gazebo::common::Time::GetWallTime();
  }

  /// \brief Name of the state.
  public: const std::string name;

  /// \brief Pointer to the validation plugin.
  protected: ValidationController &plugin;

  /// \brief Timer to measure time in the current state.
  protected: gazebo::common::Timer timer;

  /// \brief Has initialized
  protected: bool initialized = false;

  /// \brief Last gazebo state message.
  private: std::string gazeboState;

  /// \brief Mutex to protect the gazebo state message.
  private: mutable std::mutex mutex;

  /// \brief Last time the we published the state.
  private: gazebo::common::Time lastPublication;

  /// \brief Elapsed time between state publications.
  private: gazebo::common::Time publicationInterval;

  /// \brief Transport node.
  protected: ignition::transport::Node node;
};

/// \brief State that handles the "init" state.
class ReadyState : public State
{
  // Use class constructor from base class.
  using State::State;

  // Documentation inherited
  public: virtual void DoOnState()
  {
    if (this->GazeboState() == "gazebo_waiting_initial_conditions")
      this->plugin.SetState(*this->plugin.setupState);

    std::cout << "InitiState::DoFeedback()" << std::endl;
  };
};

/// \brief State that handles the "initial_conditions" state.
class InitialConditionsState : public State
{
  // Use class constructor from base class.
  using State::State;

  public: virtual void DoInitialize()
  {
    // Set the initial conditions.
  }

  public: virtual void DoUpdate()
  {
    // Check when the initial conditions are ready.
  }

  // Documentation inherited
  public: virtual void DoOnState()
  {
    if (this->GazeboState() == "gazebo_waiting_initial_conditions")
      this->plugin.SetState(*this->plugin.setupState);

    std::cout << "InitiState::DoFeedback()" << std::endl;
  };
};

class ValidationController
{
  /// \brief ToDo
  public: ValidationController()
  : readyState(new ReadyState(kReadyState, *this)),
    currentState(readyState.get())
  {
  }

  /// \brief ToDo
  public: ~ValidationController(){};

  /// \brief ToDo
  public: void Start()
  {
    while (true)
    {
      if (this->currentState)
        this->currentState->Update();

      // 1000 Hz.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  /// \brief "ready" state.
  public: std::unique_ptr<ReadyState> readyState;

  /// \brief Pointer to the current game state.
  public: State *currentState;
};

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ValidationController controller;
  controller.Start();

  return 0;
}
