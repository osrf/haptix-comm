/*
  HAPTIX User API, Part 1

  This file defines data structures and API functions
  available both in simulation and on physical robots.
*/

/// \file haptix.h
/// \brief Structures and functions for the primary HAPTIX API.

#ifndef __HAPTIX_API_HAPTIX_H__
#define __HAPTIX_API_HAPTIX_H__

#ifdef __cplusplus
extern "C" {
#endif

// ---------- constants ----------

/// \brief Maximum number of motors.
/// Defines the maximum number of motors across any particular robot.
/// It is used when allocating sensor and command objects related to the motors.
/// The number of motors for a particular robot is defined in #_hxRobotInfo.
#define hxMAXMOTOR              32

/// \brief Maximum number of joints.
/// Defines the maximum number of joints across any particular robot.
/// It is used when allocating sensor properties related to the joints.
/// number of joints for a particular robot is defined in #_hxRobotInfo.
#define hxMAXJOINT              32

/// \brief Maximum number of contact sensors.
/// Defines the maximum number of contact sensors across any particular
/// robot.
/// It is used when allocating an hxSensor.contact object.
/// The number of contact sensors for a particular robot is defined in
/// #_hxRobotInfo.
#define hxMAXCONTACTSENSOR      32

/// \brief Maximum number of IMUs.
/// Defines the maximum number of inertial measurement units across any
/// particular robot.
/// It is used when allocating sensor objects related to IMUs.
/// The number of IMUs for a particular robot is defined in #_hxRobotInfo.
#define hxMAXIMU                32

/// \brief API return codes.
/// These return codes are used to specify the outcome of a haptix-comm
/// function (e.g. success or failure).
typedef enum
{
  /// success
  hxOK = 0,
  /// failure
  hxERROR
} hxResult;

// ---------- data structures ----------

/// \brief A representation of time
struct _hxTime
{
  /// Seconds
  int sec;

  /// Nanoseconds
  int nsec;
};

/// \def hxTime
/// \brief Time representation.
typedef struct _hxTime hxTime;

/// \brief Device information.
/// This data structure specifies inherent properties of the robot that
/// do not change during simulation (for
/// example, the number of joints in the robot arm).
///
/// It can be retrieved by calling hx_robotinfo(hxRobotInfo*).
struct _hxRobotInfo
{
  /// \brief Number of motors.
  /// Motors are commanded through filling an hxCommand struct and calling
  /// hx_update(int, const hxCommand*, hxSensor*).
  ///
  /// The number of motors is less than or equal to the number of
  /// joints. For example, one motor may control several joints through
  /// kinematic joint coupling.
  int motor_count;

  /// \brief Number of hinge joints.
  /// The joints are passive and are moved as a side effect of commanding
  /// the motors. A joint may correspond directly to the movement of a motor,
  /// or it may be commanded indirectly through joint coupling.
  ///
  /// The number of joints is greater than or equal to the number of
  /// motors.
  int joint_count;

  /// \brief Number of contact sensors.
  /// A contact sensor measures the magnitude of the force on that sensor.
  int contact_sensor_count;

  /// \brief Number of IMUs (inertial measurement units).
  /// An IMU or inertial measurement unit measures the 3-dimensional linear
  /// acceleration vector and the 3-dimensional angular velocity vector
  /// experienced by the sensor.
  int imu_count;

  /// \brief Minimum and maximum motor angles (rad).
  /// An m by 2 array representing the angular limits of each motor in the
  /// robot, where m is the maximum number of motors. Each 1x2 row of the array
  /// corresponds to a motor. The first entry in the row is the lower limit
  /// of the motor. The second entry is the upper limit of the motor.
  float motor_limit[hxMAXMOTOR][2];

  /// \brief Minimum and maximum joint angles (rad).
  /// An n by 2 array representing the angular limits of each joint in the
  /// robot, where n is the maximum number of joints. Each 1x2 row of the array
  /// corresponds to a joint. The first entry in the row is the lower limit
  /// of the joint. The second entry is the upper limit of the joint.
  float joint_limit[hxMAXJOINT][2];

  /// \brief Hz rate at which the robot will update.
  float update_rate;
};

/// \def hxRobotInfo
/// \brief Robot information.
typedef struct _hxRobotInfo hxRobotInfo;

/// \brief Sensor data.
/// This data structure specifies the sensor information gained in a simulation
/// update.
/// It is an output of the function
/// hx_update(int, const hxCommand*, hxSensor*).
struct _hxSensor
{
  /// \brief Timestamp.
  /// Time at which the sensor reading was taken. See #hxTime for data format.
  hxTime time_stamp;

  /// \brief Motor position (rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the angular positions for each motor.
  /// The ordering of these motor values is consistent across the different
  /// motor-related properties of #hxSensor.
  ///
  /// These values cannot exceed the minimum and maximum values specified in
  /// hxRobotInfo::limit.
  float motor_pos[hxMAXMOTOR];

  /// \brief Motor velocity (rad/s).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the angular velocity for
  /// each motor. The ordering of
  /// these motor values is consistent across the different motor-related
  /// properties of hxSensor.
  float motor_vel[hxMAXMOTOR];

  /// \brief Torque applied by embedded controller (Nm).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the torque for each motor.
  /// The ordering of
  /// these motor values is consistent across the different motor-related
  /// properties of #hxSensor.
  float motor_torque[hxMAXMOTOR];

  /// \brief Joint position (rad).
  /// An array of floats of size #hxMAXJOINT. Entries 0 through
  /// hxRobotInfo::njoint-1 contain the angular position for each joint.
  /// The ordering of these joint values is consistent with
  /// hxSensor::joint_vel.
  float joint_pos[hxMAXJOINT];

  /// \brief Joint velocity (rad/s).
  /// An array of floats of size #hxMAXJOINT.
  /// Entries 0 through hxRobotInfo::njoint-1 contain the angular position
  /// for each joint.
  /// The ordering of these joint values is consistent with
  /// hxSensor::joint_pos.
  float joint_vel[hxMAXJOINT];

  /// \brief Contact normal force (N).
  /// An array of floats of size #hxMAXCONTACTSENSOR. Entries 0 through
  /// hxRobotInfo::ncontactsensor contain the contact magnitude for each
  /// contact sensor.
  float contact[hxMAXCONTACTSENSOR];

  /// \brief 3D accelerometer output (m/s^2).
  /// An array of floats of size #hxMAXIMUx3 where each row is a 3-dimensional
  /// vector of accelerometer output, which comprises the vector difference
  /// (a-g), where a is the linear acceleration and g is the gravity vector.
  /// This measurement is expressed in a body-fixed frame.
  /// The entries of each row are measured in
  /// meters per second squared and ordered (x, y, z).
  /// Entries 0 through hxRobotInfo::nimu-1 contain the acceleration vectors
  /// for each IMU.\n
  /// The ordering of these IMU values is consistent with
  /// hxSensor::imu_angular_vel.
  float imu_linear_acc[hxMAXIMU][3];

  /// \brief 3D angular velocity (rad/s).
  /// An array of floats of size #hxMAXIMUx3 where each row is a 3-dimensional
  /// angular velocity vector.
  /// This measurement is expressed in a body-fixed frame.
  /// The entries of each row are measured in
  /// radians per second and ordered (x, y, z).
  /// Entries 0 through hxRobotInfo::nimu-1 contain the velocity vectors
  /// for each IMU.
  ///
  /// The ordering of these IMU values is consistent with
  /// hxSensor::imu_linear_acc.
  float imu_angular_vel[hxMAXIMU][3];

  /// \brief 3D orientation quaternion.
  /// This value is set to a unit value of (1, 0, 0, 0), until the IMUs
  /// on the hardware is known to provide an orientation estimate.
  /// The expected coordinate ordering of quaternions is (w, x, y, z).
  float imu_orientation[hxMAXIMU][4]
};

/// \def hxSensor
/// \brief Sensor data.
typedef struct _hxSensor hxSensor;

/// \brief Motor command data.
/// This data structure specifies the next request to be send to the simulated
/// limb model.
///
/// It is an input of the function
/// hx_update(int, const hxCommand*, hxSensor*).
struct _hxCommand
{
  /// \brief Target reference positions (rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the desired angular positions for each
  /// motor.
  /// \sa ref_pos_count
  float ref_pos[hxMAXMOTOR];

  /// \brief A value <= 0 disables ref_pos, any other value enables
  /// ref_pos.
  /// \sa ref_pos
  int ref_pos_enabled;

  /// \brief Target reference velocities (rad/s).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the desired angular velocities for each
  /// motor.
  /// \sa ref_vel_max_count
  float ref_vel_max[hxMAXMOTOR];

  /// \brief A value <= 0 disables ref_vel_max, any other value enables
  /// ref_vel_max.
  /// \sa ref_vel_max
  int ref_vel_max_enabled;

  /// \brief Target position feedback gains (Nm/rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the position gain that will be
  /// applied during the update phase of the model controller.
  /// \sa gain_pos_count
  float gain_pos[hxMAXMOTOR];

  /// \brief A value <= 0 disables gain_pos, any other value enables
  /// gain_pos.
  /// \sa gain_pos
  int gain_pos_enabled;

  /// \brief Target velocity feedback gains (Nms/rad).
  /// An array of floats of size #hxMAXMOTOR. Entries 0 through
  /// hxRobotInfo::nmotors-1 contain the velocity gain that will be
  /// applied during the update phase of the model controller.
  /// \sa gain_vel_count
  float gain_vel[hxMAXMOTOR];

  /// \brief A value <= 0 disables gain_vel, any other value enables
  /// gain_vel.
  /// \sa gain_vel
  int gain_vel_enabled;
};

/// \def hxCommand
/// \brief Motor commands.
typedef struct _hxCommand hxCommand;

// ---------- API functions ----------

/// \brief Connect to a robot or simulator. Use this function
/// if you want to connect to a specific host and port.
///
/// \param[in] _host When connecting to a simulator, use _host to specify
/// the machine that is running the simulator. Use a value of NULL to
/// perform automatic discovery.
/// \param[in] _port When connecting to a simulator, use _port to specify
/// what port the simulator is running on. If _host is NULL, then _port
/// is ignored.
/// \return 'hxOK' if the connection succeed or an error code otherwise.
hxResult hx_connect(const char *_host, int _port);

/// \brief Close connection to a robot or simulator.
///
/// \return 'hxOK' if the disconnection succeed or an error code otherwise.
hxResult hx_close();

/// \brief Get information for a specified robot or simulator.
/// \param[out] _robotInfo Device information requested. See #_hxRobotInfo
/// for a list of available fields.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_robot_info(hxRobotInfo *_robotinfo);

/// \brief A non-blocking function that sends a command to the robot and
/// receives the latest sensor information. This function may be used as
/// frequently as desired, but the robot is only guaranteed to update at
/// a minimum of 50Hz.
/// \param[in] _command New command to be sent. See #_hxCommand for the full
/// description of fields contained in a command request.
/// \param[out] _sensor Sensor data received after the update. See #_hxSensor
/// for the full description of fields contained the state response.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_update(const hxCommand *_command, hxSensor *_sensor);

/// \brief Similar to hx_update, this function reads the latest sensor
/// information from the robot in a non-blocking manner. This function
/// may be called as often as desired, but the robot is only guaranteeed to
/// update at a minimum of 50Hz.
/// Return sensor data.
/// \param[out] _sensor Sensor data received after the update.
/// \return 'hxOK' if the operation succeed or an error code otherwise.
hxResult hx_read_sensors(hxSensor *_sensor);

/// \brief Return a string that describes the last result.
/// \sa hxResult.
/// \return String that describes the last result.
const char *hx_last_result();

#ifdef __cplusplus
}
#endif
#endif
