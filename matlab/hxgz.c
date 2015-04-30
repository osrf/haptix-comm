#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mex.h"
#include "haptix/comm/haptix.h"
#include "haptix/comm/haptix_sim.h"

// Simulation/hardware functions
void hxgz_connect (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[]);
void hxgz_close (int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]);
void hxgz_read_sensors (int nlhs, mxArray *plhs[],
                        int nrhs, const mxArray *prhs[]);
void hxgz_robot_info (int nlhs, mxArray *plhs[],
                      int nrhs, const mxArray *prhs[]);
void hxgz_update (int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);

// Simulation-specific functions
void hxgzs_sim_info (int nlhs, mxArray *plhs[],
                    int nrhs, const mxArray *prhs[]);
void hxgzs_camera_transform (int nlhs, mxArray *plhs[],
                             int nrhs, const mxArray *prhs[]);
void hxgzs_set_camera_transform (int nlhs, mxArray *plhs[],
                                 int nrhs, const mxArray *prhs[]);
void hxgzs_contacts (int nlhs, mxArray *plhs[],
                     int nrhs, const mxArray *prhs[]);
void hxgzs_set_model_joint_state (int nlhs, mxArray *plhs[],
                                  int nrhs, const mxArray *prhs[]);
void hxgzs_set_model_link_state (int nlhs, mxArray *plhs[],
                                 int nrhs, const mxArray *prhs[]);
void hxgzs_add_model (int nlhs, mxArray *plhs[],
                      int nrhs, const mxArray *prhs[]);
void hxgzs_remove_model (int nlhs, mxArray *plhs[],
                         int nrhs, const mxArray *prhs[]);
void hxgzs_model_transform (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[]);
void hxgzs_model_gravity_mode (int nlhs, mxArray *plhs[],
                               int nrhs, const mxArray *prhs[]);
void hxgzs_set_model_gravity_mode (int nlhs, mxArray *plhs[],
                                   int nrhs, const mxArray *prhs[]);
void hxgzs_linear_velocity (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[]);
void hxgzs_angular_velocity (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[]);
void hxgzs_force (int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);
void hxgzs_torque (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[]);
void hxgzs_wrench (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[]);
void hxgzs_reset (int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);
void hxgzs_reset_timer (int nlhs, mxArray *plhs[],
                        int nrhs, const mxArray *prhs[]);
void hxgzs_start_timer (int nlhs, mxArray *plhs[],
                        int nrhs, const mxArray *prhs[]);
void hxgzs_stop_timer (int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray *prhs[]);
void hxgzs_timer (int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]);
void hxgzs_start_logging (int nlhs, mxArray *plhs[],
                          int nrhs, const mxArray *prhs[]);
void hxgzs_is_logging (int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray *prhs[]);
void hxgzs_stop_logging (int nlhs, mxArray *plhs[],
                         int nrhs, const mxArray *prhs[]);
void hxgzs_set_model_color (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[]);
void hxgzs_model_color (int nlhs, mxArray *plhs[],
                        int nrhs, const mxArray *prhs[]);
void hxgzs_set_model_collide_mode (int nlhs, mxArray *plhs[],
                                   int nrhs, const mxArray *prhs[]);
void hxgzs_model_collide_mode (int nlhs, mxArray *plhs[],
                               int nrhs, const mxArray *prhs[]);

// Data structure conversion helpers
//
// haptix-comm types -> MATLAB arrays
mxArray* sensor_to_matlab (const hxSensor* hs);
mxArray* vector3_to_matlab (const hxsVector3* h);
mxArray* quaternion_to_matlab (const hxsQuaternion* h);
mxArray* transform_to_matlab (const hxsTransform* h);
mxArray* link_to_matlab (const hxsLink* h);
mxArray* wrench_to_matlab (const hxsWrench* h);
mxArray* joint_to_matlab (const hxsJoint* h);
mxArray* model_to_matlab (const hxsModel* h);
mxArray* contactpoints_to_matlab (const hxsContactPoints* h);
mxArray* color_to_matlab (const hxsColor* h);
//
// MATLAB arrays -> haptix-comm types
hxsVector3 matlab_to_vector3 (const mxArray* m);
hxsQuaternion matlab_to_quaternion (const mxArray* m);
hxsTransform matlab_to_transform (const mxArray* m);
hxsWrench matlab_to_wrench (const mxArray* m);
hxsColor matlab_to_color (const mxArray* m);


// Global info instance, for later reference
static hxRobotInfo g_info;
static int g_info_valid = 0;

void
mexFunction (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  // Get the robot info, so that we know how many rows to fill in later in
  // returned sensor data.
  if (!g_info_valid)
  {
    if (hx_robot_info(&g_info) != hxOK)
      mexErrMsgIdAndTxt("HAPTIX:hxgz", "Failed to get robot info");
    if (g_info.motor_count > hxMAXMOTOR)
      g_info.motor_count = hxMAXMOTOR;
    if (g_info.joint_count > hxMAXJOINT)
      g_info.joint_count = hxMAXJOINT;
    if (g_info.contact_sensor_count > hxMAXCONTACTSENSOR)
      g_info.contact_sensor_count = hxMAXCONTACTSENSOR;
    if (g_info.imu_count > hxMAXIMU)
      g_info.imu_count = hxMAXIMU;
    g_info_valid = 1;
  }

  // Dispatch to the appropriate function based on the first argument
  char funcName[128];
  if (nrhs < 1)
    mexErrMsgIdAndTxt("HAPTIX:hxgz", "Expects >= 1 argument");
  if (mxGetString(prhs[0], funcName, sizeof(funcName)))
    mexErrMsgIdAndTxt("HAPTIX:hxgz", "Failed to determine function name");
  if (!strcmp(funcName, "connect"))
    hxgz_connect(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "robot_info"))
    hxgz_robot_info(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "update"))
    hxgz_update(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "read_sensors"))
    hxgz_read_sensors(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "close"))
    hxgz_close(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "sim_info"))
    hxgzs_sim_info(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "camera_transform"))
    hxgzs_camera_transform(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "set_camera_transform"))
    hxgzs_set_camera_transform(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "contacts"))
    hxgzs_contacts(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "set_model_joint_state"))
    hxgzs_set_model_joint_state(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "set_model_link_state"))
    hxgzs_set_model_link_state(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "add_model"))
    hxgzs_add_model(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "remove_model"))
    hxgzs_remove_model(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "model_transform"))
    hxgzs_model_transform(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "model_gravity_mode"))
    hxgzs_model_gravity_mode(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "set_model_gravity_mode"))
    hxgzs_set_model_gravity_mode(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "linear_velocity"))
    hxgzs_linear_velocity(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "angular_velocity"))
    hxgzs_angular_velocity(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "force"))
    hxgzs_force(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "torque"))
    hxgzs_torque(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "wrench"))
    hxgzs_wrench(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "reset"))
    hxgzs_reset(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "reset_timer"))
    hxgzs_reset_timer(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "start_timer"))
    hxgzs_start_timer(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "stop_timer"))
    hxgzs_stop_timer(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "start_logging"))
    hxgzs_start_logging(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "is_logging"))
    hxgzs_is_logging(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "stop_logging"))
    hxgzs_stop_logging(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "set_model_color"))
    hxgzs_set_model_color(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "model_color"))
    hxgzs_model_color(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "set_model_collide_mode"))
    hxgzs_set_model_collide_mode(nlhs, plhs, nrhs-1, prhs+1);
  else if (!strcmp(funcName, "model_collide_mode"))
    hxgzs_model_collide_mode(nlhs, plhs, nrhs-1, prhs+1);
  else
    mexErrMsgIdAndTxt("HAPTIX:hxgz", "Unknown command");
}

void
hxgz_connect (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  // no-op
}

void
hxgz_close (int nlhs, mxArray *plhs[],
            int nrhs, const mxArray *prhs[])
{
  if (hx_close() != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hx_close", hx_last_result());
}

void
hxgz_read_sensors (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
  hxSensor sensor;

  // Request robot information.
  if (hx_read_sensors(&sensor) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hx_read_sensors", hx_last_result());

  mxArray* s = sensor_to_matlab(&sensor);

  // Set the output arguments.
  plhs[0] = s;
}

void
hxgz_robot_info (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  hxRobotInfo robotInfo;
  int i;

  // Request robot information.
  if (hx_robot_info(&robotInfo) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hx_robot_info", hx_last_result());

  // Create a Matlab structure array.
  const char *keys[] = {"motor_count",
                        "joint_count",
                        "contact_sensor_count",
                        "imu_count",
                        "motor_limit",
                        "joint_limit",
                        "update_rate"};
  mxArray *s = mxCreateStructMatrix (1, 1, 7, keys);

  // Create the empty mxArrays for the structure array.
  mxArray *motorCountArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *jointCountArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *contactSensorCountArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *imuCountArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *motorLimitArray = mxCreateDoubleMatrix(
      robotInfo.motor_count, 2, mxREAL);
  mxArray *jointLimitArray = mxCreateDoubleMatrix(
      robotInfo.joint_count, 2, mxREAL);
  mxArray *updateRateArray = mxCreateDoubleMatrix(1, 1, mxREAL);

  // Get the pointers to the data.
  double *motorCountData = mxGetPr(motorCountArray);
  double *jointCountData = mxGetPr(jointCountArray);
  double *contactSensorCountData = mxGetPr(contactSensorCountArray);
  double *imuCountData = mxGetPr(imuCountArray);
  double *motorLimitData = mxGetPr(motorLimitArray);
  double *jointLimitData = mxGetPr(jointLimitArray);
  double *updateRateData = mxGetPr(updateRateArray);

  // Fill the nmotor field.
  motorCountData[0] = robotInfo.motor_count;

  // Fill the njoint field.
  jointCountData[0] = robotInfo.joint_count;

  // Fill the ncontactsensor field.
  contactSensorCountData[0] = robotInfo.contact_sensor_count;

  // Fill the nIMU field.
  imuCountData[0] = robotInfo.imu_count;

  // Fill the motor limits.
  for (i = 0; i < robotInfo.motor_count; ++i)
  {
    motorLimitData[i] = robotInfo.motor_limit[i][0];
    motorLimitData[i + robotInfo.motor_count] = robotInfo.motor_limit[i][1];
  }

  // Fill the joint limits.
  for (i = 0; i < robotInfo.joint_count; ++i)
  {
    jointLimitData[i] = robotInfo.joint_limit[i][0];
    jointLimitData[i + robotInfo.joint_count] = robotInfo.joint_limit[i][1];
  }

  // Fill the update_rate field
  updateRateData[0] = robotInfo.update_rate;

  // Set the structure array fields.
  mxSetField(s, 0, "motor_count", motorCountArray);
  mxSetField(s, 0, "joint_count", jointCountArray);
  mxSetField(s, 0, "contact_sensor_count", contactSensorCountArray);
  mxSetField(s, 0, "imu_count", imuCountArray);
  mxSetField(s, 0, "joint_limit", jointLimitArray);
  mxSetField(s, 0, "motor_limit", motorLimitArray);
  mxSetField(s, 0, "update_rate", updateRateArray);

  // Set the output arguments.
  plhs[0] = s;
}

void
hxgz_update (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  hxCommand cmd;
  hxSensor sensor;
  int i;
  mxArray *v;
  const mwSize *cmdSize;
  double *data;

  // Sanity check: Verify that the first input argument is a struct.
  if (nrhs != 1 || !mxIsStruct(prhs[0]))
    mexErrMsgIdAndTxt("HAPTIX:hx_update", "Expects struct");

  // Sanity check: Verify that the struct has fields:
  // ref_pos, ref_vel_max, gain_pos and gain_vel, plus the
  // *_enabled flag for each one.
  if (mxGetNumberOfFields(prhs[0]) != 8)
    mexErrMsgIdAndTxt("HAPTIX:hx_update", "Expects 8 fields");

  // Set the hxCommand struct.
  v = mxGetField(prhs[0], 0, "ref_pos");
  data = mxGetPr(v);
  cmdSize = mxGetDimensions(v);
  for (i = 0; (i < cmdSize[1]) && (i < hxMAXMOTOR); ++i)
    cmd.ref_pos[i] = data[i];

  v = mxGetField(prhs[0], 0, "ref_pos_enabled");
  data = mxGetPr(v);
  cmd.ref_pos_enabled = data[0];

  v = mxGetField(prhs[0], 0, "ref_vel_max");
  data = mxGetPr(v);
  cmdSize = mxGetDimensions(v);
  for (i = 0; (i < cmdSize[1]) && (i < hxMAXMOTOR); ++i)
    cmd.ref_vel_max[i] = data[i];

  v = mxGetField(prhs[0], 0, "ref_vel_max_enabled");
  data = mxGetPr(v);
  cmd.ref_vel_max_enabled = data[0];

  v = mxGetField(prhs[0], 0, "gain_pos");
  data = mxGetPr(v);
  cmdSize = mxGetDimensions(v);
  for (i = 0; (i < cmdSize[1]) && (i < hxMAXMOTOR); ++i)
    cmd.gain_pos[i] = data[i];

  v = mxGetField(prhs[0], 0, "gain_pos_enabled");
  data = mxGetPr(v);
  cmd.gain_pos_enabled = data[0];

  v = mxGetField(prhs[0], 0, "gain_vel");
  data = mxGetPr(v);
  cmdSize = mxGetDimensions(v);
  for (i = 0; (i < cmdSize[1]) && (i < hxMAXMOTOR); ++i)
    cmd.gain_vel[i] = data[i];

  v = mxGetField(prhs[0], 0, "gain_vel_enabled");
  data = mxGetPr(v);
  cmd.gain_vel_enabled = data[0];

  // Request robot information.
  if (hx_update(&cmd, &sensor) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hx_update", hx_last_result());

  mxArray* s = sensor_to_matlab(&sensor);

  // Set the output arguments.
  plhs[0] = s;
}

mxArray*
sensor_to_matlab (const hxSensor* hs)
{
  int i;

  // Create a Matlab structure array.
  const char *keys[] = {"time_stamp",
                        "motor_pos",
                        "motor_vel",
			"motor_torque",
			"joint_pos",
			"joint_vel",
			"contact",
			"imu_linear_acc",
			"imu_angular_vel",
			"imu_orientation"};
  mxArray *s = mxCreateStructMatrix (1, 1, 10, keys);

  // Create the empty mxArrays for the structure array.
  mxArray *timeStampArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *motorPosArray = mxCreateDoubleMatrix(g_info.motor_count, 1, mxREAL);
  mxArray *motorVelArray = mxCreateDoubleMatrix(g_info.motor_count, 1, mxREAL);
  mxArray *motorTorqueArray = mxCreateDoubleMatrix(g_info.motor_count, 1, mxREAL);
  mxArray *jointPosArray = mxCreateDoubleMatrix(g_info.joint_count, 1, mxREAL);
  mxArray *jointVelArray = mxCreateDoubleMatrix(g_info.joint_count, 1, mxREAL);
  mxArray *contactArray = mxCreateDoubleMatrix(g_info.contact_sensor_count, 1, mxREAL);
  mxArray *imuLinAccArray = mxCreateDoubleMatrix(g_info.imu_count, 3, mxREAL);
  mxArray *imuAngVelArray = mxCreateDoubleMatrix(g_info.imu_count, 3, mxREAL);
  mxArray *imuOrientationArray = mxCreateDoubleMatrix(g_info.imu_count, 4, mxREAL);

  // Get the pointers to the data.
  double *timeStampData = mxGetPr(timeStampArray);
  double *motorPosData = mxGetPr(motorPosArray);
  double *motorVelData = mxGetPr(motorVelArray);
  double *motorTorqueData = mxGetPr(motorTorqueArray);
  double *jointPosData = mxGetPr(jointPosArray);
  double *jointVelData = mxGetPr(jointVelArray);
  double *contactData = mxGetPr(contactArray);
  double *imuLinAccData = mxGetPr(imuLinAccArray);
  double *imuAngVelData = mxGetPr(imuAngVelArray);
  double *imuOrientationData = mxGetPr(imuOrientationArray);

  timeStampData[0] = hs->time_stamp.sec + hs->time_stamp.nsec/1e9;

  // Fill the motor fields.
  for (i = 0; i < g_info.motor_count; ++i)
  {
    motorPosData[i] = (double)hs->motor_pos[i];
    motorVelData[i] = (double)hs->motor_vel[i];
    motorTorqueData[i] = (double)hs->motor_torque[i];
  }

  // Fill the joint fields.
  for (i = 0; i < g_info.joint_count; ++i)
  {
    jointPosData[i] = (double)hs->joint_pos[i];
    jointVelData[i] = (double)hs->joint_vel[i];
  }

  // Fill the contact sensor field.
  for (i = 0; i < g_info.contact_sensor_count; ++i)
    contactData[i] = (double)hs->contact[i];

  // Fill the IMU fields.
  for (i = 0; i < g_info.imu_count; ++i)
  {
    imuLinAccData[i] = (double)hs->imu_linear_acc[i][0];
    imuLinAccData[i + g_info.imu_count] = (double)hs->imu_linear_acc[i][1];
    imuLinAccData[i + 2 * g_info.imu_count] = (double)hs->imu_linear_acc[i][2];

    imuAngVelData[i] = (double)hs->imu_angular_vel[i][0];
    imuAngVelData[i + g_info.imu_count] = (double)hs->imu_angular_vel[i][1];
    imuAngVelData[i + 2 * g_info.imu_count] = (double)hs->imu_angular_vel[i][2];

    imuOrientationData[i] = (double)hs->imu_orientation[i][0];
    imuOrientationData[i + g_info.imu_count] = (double)hs->imu_orientation[i][1];
    imuOrientationData[i + 2 * g_info.imu_count] = (double)hs->imu_orientation[i][2];
    imuOrientationData[i + 3 * g_info.imu_count] = (double)hs->imu_orientation[i][3];
  }

  // Set the structure array fields.
  mxSetField(s, 0, "time_stamp", timeStampArray);
  mxSetField(s, 0, "motor_pos", motorPosArray);
  mxSetField(s, 0, "motor_vel", motorVelArray);
  mxSetField(s, 0, "motor_torque", motorTorqueArray);
  mxSetField(s, 0, "joint_pos", jointPosArray);
  mxSetField(s, 0, "joint_vel", jointVelArray);
  mxSetField(s, 0, "contact", contactArray);
  mxSetField(s, 0, "imu_linear_acc", imuLinAccArray);
  mxSetField(s, 0, "imu_angular_vel", imuAngVelArray);
  mxSetField(s, 0, "imu_orientation", imuOrientationArray);

  return s;
}

mxArray*
vector3_to_matlab (const hxsVector3* h)
{
  mxArray *array = mxCreateDoubleMatrix(3, 1, mxREAL);
  double *data = mxGetPr(array);
  data[0] = h->x;
  data[1] = h->y;
  data[2] = h->z;

  return array;
}

mxArray*
quaternion_to_matlab (const hxsQuaternion* h)
{
  mxArray *array = mxCreateDoubleMatrix(4, 1, mxREAL);
  double *data = mxGetPr(array);
  data[0] = h->w;
  data[1] = h->x;
  data[2] = h->y;
  data[3] = h->z;

  return array;
}

mxArray*
transform_to_matlab (const hxsTransform* h)
{
  const char *keys[] = {"pos", "orient"};
  mxArray *s = mxCreateStructMatrix(1, 1, 2, keys);

  mxArray *posArray = vector3_to_matlab(&(h->pos));
  mxArray *orientArray = quaternion_to_matlab(&(h->orient));

  mxSetField(s, 0, "pos", posArray);
  mxSetField(s, 0, "orient", orientArray);

  return s;
}

mxArray*
link_to_matlab (const hxsLink* h)
{
  const char *keys[] = {"name",
                        "transform",
                        "lin_vel",
                        "ang_vel",
                        "lin_acc",
                        "ang_acc"};
  mxArray *s = mxCreateStructMatrix(1, 1, 6, keys);

  mxArray* nameArray = mxCreateString(h->name);
  mxArray* transformArray = transform_to_matlab(&(h->transform));
  mxArray* linVelArray = vector3_to_matlab(&(h->lin_vel));
  mxArray* angVelArray = vector3_to_matlab(&(h->ang_vel));
  mxArray* linAccArray = vector3_to_matlab(&(h->lin_acc));
  mxArray* angAccArray = vector3_to_matlab(&(h->ang_acc));

  mxSetField(s, 0, "name", nameArray);
  mxSetField(s, 0, "transform", transformArray);
  mxSetField(s, 0, "lin_vel", linVelArray);
  mxSetField(s, 0, "ang_vel", angVelArray);
  mxSetField(s, 0, "lin_acc", linAccArray);
  mxSetField(s, 0, "ang_acc", angAccArray);

  return s;
}

mxArray*
wrench_to_matlab (const hxsWrench* h)
{
  const char *keys[] = {"force", "torque"};
  mxArray *s = mxCreateStructMatrix(1, 1, 2, keys);

  mxArray *forceArray = vector3_to_matlab(&(h->force));
  mxArray *torqueArray = vector3_to_matlab(&(h->torque));

  mxSetField(s, 0, "force", forceArray);
  mxSetField(s, 0, "torque", torqueArray);

  return s;
}

mxArray*
joint_to_matlab (const hxsJoint* h)
{
  const char *keys[] = {"name",
                        "pos",
                        "vel",
                        "acc",
                        "torque_motor",
                        "wrench_reactive"};
  mxArray *s = mxCreateStructMatrix(1, 1, 6, keys);

  mxArray *nameArray = mxCreateString(h->name);
  mxArray *posArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *velArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *accArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *torqueMotorArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *wrenchReactiveArray = wrench_to_matlab(&(h->wrench_reactive));

  double* posData = mxGetPr(posArray);
  double* velData = mxGetPr(velArray);
  double* accData = mxGetPr(accArray);
  double* torqueMotorData = mxGetPr(torqueMotorArray);

  posData[0] = h->pos;
  velData[0] = h->vel;
  accData[0] = h->acc;
  torqueMotorData[0] = h->torque_motor;

  mxSetField(s, 0, "name", nameArray);
  mxSetField(s, 0, "pos", posArray);
  mxSetField(s, 0, "vel", velArray);
  mxSetField(s, 0, "acc", accArray);
  mxSetField(s, 0, "torque_motor", torqueMotorArray);
  mxSetField(s, 0, "wrench_reactive", wrenchReactiveArray);

  return s;
}

mxArray*
model_to_matlab (const hxsModel* h)
{
  const char *keys[] = {"name",
                        "transform",
                        "id",
                        "links",
                        "joints",
                        "gravity_mode"};
  mxArray *s = mxCreateStructMatrix(1, 1, 6, keys);

  mxArray *nameArray = mxCreateString(h->name);
  mxArray *transformArray = transform_to_matlab(&(h->transform));
  mxArray *idArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  // TODO: Look for a way to build an array of structures without having to
  // create this extra layers of field names.  E.g., have the "links" field be
  // an array of link structures, so that you can access each one as:
  //   links(42)
  // instead of
  //   links(42).link
  // I looked around a bit but could not find a way to do that.
  const char *linksKeys[] = {"link"};
  mxArray *linksArray = mxCreateStructMatrix(h->link_count, 1, 1, linksKeys);
  const char *jointsKeys[] = {"joint"};
  mxArray *jointsArray = mxCreateStructMatrix(h->joint_count, 1, 1, jointsKeys);
  mxArray *gravityModeArray = mxCreateDoubleMatrix(1, 1, mxREAL);

  double* idData = mxGetPr(idArray);
  double* gravityModeData = mxGetPr(gravityModeArray);

  idData[0] = h->id;
  int i;
  for(i=0; i<h->link_count; ++i)
    mxSetField(linksArray, i, "link", link_to_matlab(h->links+i));
  for(i=0; i<h->joint_count; ++i)
    mxSetField(jointsArray, i, "link", joint_to_matlab(h->joints+i));
  gravityModeData[0] = h->gravity_mode;

  mxSetField(s, 0, "name", nameArray);
  mxSetField(s, 0, "transform", transformArray);
  mxSetField(s, 0, "id", idArray);
  mxSetField(s, 0, "links", linksArray);
  mxSetField(s, 0, "joints", jointsArray);
  mxSetField(s, 0, "gravity_mode", gravityModeArray);

  return s;
}

mxArray*
contactpoints_to_matlab (const hxsContactPoints* h)
{
  const char *keys[] = {"link1",
                        "link2",
                        "point",
                        "normal",
                        "distance",
                        "wrench"};
  mxArray *s = mxCreateStructMatrix(h->contact_count, 1, 6, keys);

  int i;
  for (i=0; i<h->contact_count; ++i)
  {
    mxArray *link1Array = mxCreateString(h->contacts[i].link1);
    mxArray *link2Array = mxCreateString(h->contacts[i].link2);
    mxArray *pointArray = vector3_to_matlab(&(h->contacts[i].point));
    mxArray *normalArray = vector3_to_matlab(&(h->contacts[i].normal));
    mxArray *distanceArray = mxCreateDoubleMatrix(1, 1, mxREAL);
    mxArray *wrenchArray = wrench_to_matlab(&(h->contacts[i].wrench));
  
    double* distanceData = mxGetPr(distanceArray);
  
    distanceData[0] = h->contacts[i].distance;
  
    mxSetField(s, i, "link1", link1Array);
    mxSetField(s, i, "link2", link2Array);
    mxSetField(s, i, "point", pointArray);
    mxSetField(s, i, "normal", normalArray);
    mxSetField(s, i, "distance", distanceArray);
    mxSetField(s, i, "wrench", wrenchArray);
  }

  return s;
}

mxArray*
color_to_matlab (const hxsColor* h)
{
  mxArray *array = mxCreateDoubleMatrix(4, 1, mxREAL);
  double *data = mxGetPr(array);
  data[0] = h->r;
  data[1] = h->g;
  data[2] = h->b;
  data[3] = h->alpha;

  return array;
}

hxsVector3
matlab_to_vector3 (const mxArray* m)
{
  hxsVector3 v;

  if (mxGetM(m) != 3 || mxGetN(m) != 1)
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_vector3", "Expects 3x1 column vector");

  double *d = mxGetPr(m);
  v.x = d[0];
  v.y = d[1];
  v.z = d[2];

  return v;
}

hxsQuaternion
matlab_to_quaternion (const mxArray* m)
{
  hxsQuaternion q;

  if (mxGetM(m) != 4 || mxGetN(m) != 1)
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_quaternion", "Expects 4x1 column vector");

  double *d = mxGetPr(m);
  q.w = d[0];
  q.x = d[1];
  q.y = d[2];
  q.z = d[3];

  return q;
}

hxsTransform
matlab_to_transform (const mxArray* m)
{
  hxsTransform t;
  mxArray* v;

  // Sanity check: Verify that the first input argument is a struct.
  if (!mxIsStruct(m))
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_transform", "Expects struct");

  // Sanity check: Verify that the struct has fields:
  // pos, orient
  if (mxGetNumberOfFields(m) != 2)
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_transform", "Expects 2 fields");

  v = mxGetField(m, 0, "pos");
  t.pos = matlab_to_vector3(v);
  v = mxGetField(m, 0, "orient");
  t.orient = matlab_to_quaternion(v);

  return t;
}

hxsWrench
matlab_to_wrench (const mxArray* m)
{
  hxsWrench w;
  mxArray* v;

  // Sanity check: Verify that the first input argument is a struct.
  if (!mxIsStruct(m))
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_wrench", "Expects struct");

  // Sanity check: Verify that the struct has fields:
  // pos, orient
  if (mxGetNumberOfFields(m) != 2)
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_wrench", "Expects 2 fields");

  v = mxGetField(m, 0, "force");
  w.force = matlab_to_vector3(v);
  v = mxGetField(m, 0, "torque");
  w.torque = matlab_to_vector3(v);

  return w;
}

hxsColor
matlab_to_color (const mxArray* m)
{
  hxsColor c;

  if (mxGetM(m) != 4 || mxGetN(m) != 1)
    mexErrMsgIdAndTxt("HAPTIX:matlab_to_color", "Expects 4x1 column vector");

  double *d = mxGetPr(m);
  c.r = d[0];
  c.g = d[1];
  c.b = d[2];
  c.alpha = d[3];

  return c;
}

void
hxgzs_sim_info (int nlhs, mxArray *plhs[],
               int nrhs, const mxArray *prhs[])
{
  hxsSimInfo h;

  // Request robot information.
  if (hxs_sim_info(&h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_sim_info", hx_last_result());

  // Create a Matlab structure array.
  const char *keys[] = {"models",
                        "camera_transform"};
  mxArray *s = mxCreateStructMatrix (1, 1, 2, keys);

  const char *modelsKeys[] = {"model"};
  mxArray *modelsArray =
    mxCreateStructMatrix(h.model_count, 1, 1, modelsKeys);
  mxArray* cameraTransformArray =
    transform_to_matlab(&(h.camera_transform));

  int i;
  for(i=0; i<h.model_count; ++i)
    mxSetField(modelsArray, i, "model", model_to_matlab(h.models+i));

  mxSetField(s, 0, "models", modelsArray);
  mxSetField(s, 0, "camera_transform", cameraTransformArray);

  // Set the output arguments.
  plhs[0] = s;
}

void
hxgzs_camera_transform (int nlhs, mxArray *plhs[],
                        int nrhs, const mxArray *prhs[])
{
  hxsTransform h;

  if (hxs_camera_transform(&h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_camera_transform", hx_last_result());

  plhs[0] = transform_to_matlab(&h);
}

void
hxgzs_set_camera_transform (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[])
{
  hxsTransform h;

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_camera_transform", "Expects 1 argument");

  h = matlab_to_transform(prhs[0]);

  if (hxs_set_camera_transform(&h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_camera_transform", hx_last_result());
}

void
hxgzs_contacts (int nlhs, mxArray *plhs[],
                int nrhs, const mxArray *prhs[])
{
  hxsContactPoints h;
  char m[hxsMAXNAMESIZE];

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_contacts", "Expects 1 argument");

  if (mxGetString(prhs[0], m, sizeof(m)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_contacts", "Failed to determine model name");

  if (hxs_contacts(m, &h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_contacts", hx_last_result());

  plhs[0] = contactpoints_to_matlab(&h);
}

void
hxgzs_set_model_joint_state (int nlhs, mxArray *plhs[],
                             int nrhs, const mxArray *prhs[])
{
  char m[hxsMAXNAMESIZE];
  char j[hxsMAXNAMESIZE];
  double *d;
  float pos, vel;

  if (nrhs != 4)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_joint_state", "Expects 4 arguments");

  if (mxGetString(prhs[0], m, sizeof(m)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_joint_state", "Failed to determine model name");
  if (mxGetString(prhs[1], j, sizeof(j)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_joint_state",
      "Failed to determine joint name");
  d = mxGetPr(prhs[2]);
  pos = d[0];
  d = mxGetPr(prhs[3]);
  vel = d[0];

  if (hxs_set_model_joint_state(m, j, pos, vel) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_joint_state", hx_last_result());
}

void
hxgzs_set_model_link_state (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[])
{
  char m[hxsMAXNAMESIZE];
  char l[hxsMAXNAMESIZE];
  hxsTransform t;
  hxsVector3 lin_vel;
  hxsVector3 ang_vel;

  if (nrhs != 5)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_joint_state", "Expects 5 arguments");

  if (mxGetString(prhs[0], m, sizeof(m)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_link_state",
      "Failed to determine model name");
  if (mxGetString(prhs[1], l, sizeof(l)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_link_state",
      "Failed to determine link name");
  t = matlab_to_transform(prhs[2]);
  lin_vel = matlab_to_vector3(prhs[3]);
  ang_vel = matlab_to_vector3(prhs[4]);

  if (hxs_set_model_link_state(m, l, &t, &lin_vel, &ang_vel) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_link_state", hx_last_result());
}

void
hxgzs_add_model (int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
  char *sdf;
  char name[hxsMAXNAMESIZE];
  float x, y, z, roll, pitch, yaw;
  int gravity_mode;
  hxsModel model;

  if (nrhs != 5)
    mexErrMsgIdAndTxt("HAPTIX:hxs_add_model", "Expects 5 arguments");

  size_t sdfLen = mxGetN(prhs[0]) + 1;
  sdf = (char*)calloc(sdfLen, sizeof(char*));
  if (mxGetString(prhs[0], sdf, sdfLen))
    mexErrMsgIdAndTxt("HAPTIX:hxs_add_model", "Failed to determine sdf");
  if (mxGetString(prhs[1], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_add_model", "Failed to determine name");

  double *d;
  if (mxGetM(prhs[2]) != 3 || mxGetN(prhs[2]) != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_add_model", "Expects 3x1 column vector");
  d = mxGetPr(prhs[2]);
  x = d[0];
  y = d[1];
  z = d[2];
  if (mxGetM(prhs[3]) != 3 || mxGetN(prhs[3]) != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_add_model", "Expects 3x1 column vector");
  d = mxGetPr(prhs[3]);
  roll = d[0];
  pitch = d[1];
  yaw = d[2];
  if (mxGetM(prhs[4]) != 1 || mxGetN(prhs[4]) != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_add_model", "Expects scalar");
  d = mxGetPr(prhs[4]);
  gravity_mode = d[0];

  if (hxs_add_model(sdf, name, x, y, z, 
                    roll, pitch, yaw, gravity_mode, &model) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_link_state", hx_last_result());

  plhs[0] = model_to_matlab(&model);
}

void
hxgzs_remove_model (int nlhs, mxArray *plhs[],
                    int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_remove_model", "Expects 1 argument");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_remove_model", "Failed to determine name");

  if (hxs_remove_model(name) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_remove_model", hx_last_result());
}

void
hxgzs_model_transform (int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsTransform t;

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_transform", "Expects 1 argument");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_transform", "Failed to determine name");

  if (hxs_model_transform(name, &t) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_transform", hx_last_result());

  plhs[0] = transform_to_matlab(&t);
}

void
hxgzs_model_gravity_mode (int nlhs, mxArray *plhs[],
                          int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  int g;

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_gravity_mode", "Expects 1 argument");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_gravity_mode", "Failed to determine name");

  if (hxs_model_gravity_mode(name, &g) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_gravity_mode", hx_last_result());

  mxArray *m = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *d = mxGetPr(m);
  d[0] = g;

  plhs[0] = m;
}

void
hxgzs_set_model_gravity_mode (int nlhs, mxArray *plhs[],
                              int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  int g;

  if (nrhs != 2)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_gravity_mode", "Expects 2 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_gravity_mode",
      "Failed to determine name");
  double *d = mxGetPr(prhs[1]);
  g = d[0];

  if (hxs_set_model_gravity_mode(name, g) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_gravity_mode", hx_last_result());
}

void
hxgzs_linear_velocity (int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsVector3 v;

  if (nrhs != 2)
    mexErrMsgIdAndTxt("HAPTIX:hxs_linear_velocity", "Expects 2 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_linear_velocity", "Failed to determine name");
  v = matlab_to_vector3(prhs[1]);

  if (hxs_linear_velocity(name, &v) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_linear_velocity", hx_last_result());
}

void
hxgzs_angular_velocity (int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsVector3 v;

  if (nrhs != 2)
    mexErrMsgIdAndTxt("HAPTIX:hxs_angular_velocity", "Expects 2 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_angular_velocity", "Failed to determine name");
  v = matlab_to_vector3(prhs[1]);

  if (hxs_angular_velocity(name, &v) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_angular_velocity", hx_last_result());
}

void
hxgzs_force (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  char link[hxsMAXNAMESIZE];
  hxsVector3 v;
  float duration;

  if (nrhs != 4)
    mexErrMsgIdAndTxt("HAPTIX:hxs_force", "Expects 4 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_force", "Failed to determine name");
  if (mxGetString(prhs[1], link, sizeof(link)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_force", "Failed to determine link");
  v = matlab_to_vector3(prhs[2]);
  double *d = mxGetPr(prhs[3]);
  duration = d[0];

  if (hxs_force(name, link, &v, duration) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_force", hx_last_result());
}

void
hxgzs_torque (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  char link[hxsMAXNAMESIZE];
  hxsVector3 v;
  float duration;

  if (nrhs != 4)
    mexErrMsgIdAndTxt("HAPTIX:hxs_torque", "Expects 4 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_torque", "Failed to determine name");
  if (mxGetString(prhs[1], link, sizeof(link)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_torque", "Failed to determine link");
  v = matlab_to_vector3(prhs[2]);
  double *d = mxGetPr(prhs[3]);
  duration = d[0];

  if (hxs_torque(name, link, &v, duration) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_torque", hx_last_result());
}

void
hxgzs_wrench (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  char link[hxsMAXNAMESIZE];
  hxsWrench w;
  float duration;

  if (nrhs != 4)
    mexErrMsgIdAndTxt("HAPTIX:hxs_wrench", "Expects 4 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_wrench", "Failed to determine name");
  if (mxGetString(prhs[1], link, sizeof(link)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_wrench", "Failed to determine link");
  w = matlab_to_wrench(prhs[2]);
  double *d = mxGetPr(prhs[3]);
  duration = d[0];

  if (hxs_wrench(name, link, &w, duration) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_wrench", hx_last_result());
}

void
hxgzs_reset (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  int reset_limb_pose;

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_reset", "Expects 1 argument");

  double *d = mxGetPr(prhs[0]);
  reset_limb_pose = d[0];

  if (hxs_reset(reset_limb_pose) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_reset", hx_last_result());
}

void
hxgzs_reset_timer (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
  if (hxs_reset_timer() != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_reset_timer", hx_last_result());
}

void
hxgzs_start_timer (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
  if (hxs_start_timer() != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_start_timer", hx_last_result());
}

void
hxgzs_stop_timer (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
  if (hxs_stop_timer() != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_stop_timer", hx_last_result());
}

void
hxgzs_timer (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  hxTime t;

  if (hxs_timer(&t) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_timer", hx_last_result());

  mxArray *m = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *d = mxGetPr(m);
  d[0] = t.sec + t.nsec/1e9;

  plhs[0] = m;
}

void
hxgzs_start_logging (int nlhs, mxArray *plhs[],
                     int nrhs, const mxArray *prhs[])
{
  char filename[hxsMAXNAMESIZE];

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_start_logging", "Expects 1 argument");

  if (mxGetString(prhs[0], filename, sizeof(filename)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_start_logging", "Failed to determine filename");

  if (hxs_start_logging(filename) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_start_logging", hx_last_result());
}

void
hxgzs_is_logging (int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  int result;

  if (hxs_is_logging(&result) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_is_logging", hx_last_result());

  mxArray *m = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *d = mxGetPr(m);
  d[0] = result;

  plhs[0] = m;
}

void
hxgzs_stop_logging (int nlhs, mxArray *plhs[],
                    int nrhs, const mxArray *prhs[])
{
  if (hxs_stop_logging() != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_stop_logging", hx_last_result());
}

void
hxgzs_set_model_color (int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsColor c;

  if (nrhs != 2)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_color", "Expects 2 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_color", "Failed to determine name");
  c = matlab_to_color(prhs[1]);

  if (hxs_set_model_color(name, &c) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_color", hx_last_result());
}

void
hxgzs_model_color (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsColor c;

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_color", "Expects 1 argument");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_color", "Failed to determine name");

  if (hxs_model_color(name, &c) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_color", hx_last_result());

  mxArray *m = color_to_matlab(&c);
  
  plhs[0] = m;
}

void
hxgzs_set_model_collide_mode (int nlhs, mxArray *plhs[],
                              int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsCollideMode c;

  if (nrhs != 2)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_collide_mode", "Expects 2 arguments");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_collide_mode", "Failed to determine name");
  double *d = mxGetPr(prhs[1]);
  c = d[0];

  if (hxs_set_model_collide_mode(name, &c) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_set_model_collide_mode", hx_last_result());
}

void
hxgzs_model_collide_mode (int nlhs, mxArray *plhs[],
                          int nrhs, const mxArray *prhs[])
{
  char name[hxsMAXNAMESIZE];
  hxsCollideMode c;

  if (nrhs != 1)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_collide_mode", "Expects 1 argument");

  if (mxGetString(prhs[0], name, sizeof(name)))
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_collide_mode", "Failed to determine name");

  if (hxs_model_collide_mode(name, &c) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_model_collide_mode", hx_last_result());

  mxArray *m = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *d = mxGetPr(m);
  d[0] = c;
  
  plhs[0] = m;
}
