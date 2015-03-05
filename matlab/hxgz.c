#include <stdint.h>
#include <string.h>
#include "mex.h"
#include "haptix/comm/haptix.h"

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
mxArray* sensor_to_matlab (hxSensor* hs);

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
      mexErrMsgIdAndTxt("hxgz", "Failed to get robot info");
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

  char funcName[128];
  if (nrhs < 1)
    mexErrMsgIdAndTxt("hxgz", "Expects >= 1 argument");
  if (mxGetString(prhs[0], funcName, sizeof(funcName)))
    mexErrMsgIdAndTxt("hxgz", "Failed to determine function name");
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
  else
    mexErrMsgIdAndTxt("hxgz", "Unknown command");
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
    mexErrMsgIdAndTxt("hx_close", hx_last_result());
}

void
hxgz_read_sensors (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
  hxSensor sensor;

  // Request robot information.
  if (hx_read_sensors(&sensor) != hxOK)
    mexErrMsgIdAndTxt("hx_read_sensors", hx_last_result());

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
    mexErrMsgIdAndTxt("hx_robot_info", hx_last_result());

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
    mexErrMsgIdAndTxt("hx_update", "Expects struct");

  // Sanity check: Verify that the struct has fields:
  // ref_pos, ref_vel_max, gain_pos and gain_vel, plus the
  // *_enabled flag for each one.
  if (mxGetNumberOfFields(prhs[0]) != 8)
    mexErrMsgIdAndTxt("hx_update", "Expects 8 fields");

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
    mexErrMsgIdAndTxt("hx_update", hx_last_result());

  mxArray* s = sensor_to_matlab(&sensor);

  // Set the output arguments.
  plhs[0] = s;
}

mxArray*
sensor_to_matlab (hxSensor* hs)
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
