#include <stdint.h>
#include <string.h>
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
void hxgzs_siminfo (int nlhs, mxArray *plhs[],
                    int nrhs, const mxArray *prhs[]);
void hxgzs_camera_transform (int nlhs, mxArray *plhs[],
                             int nrhs, const mxArray *prhs[]);

// Data structure conversion helpers
mxArray* sensor_to_matlab (hxSensor* hs);
mxArray* vector3_to_matlab (hxVector3* h);
mxArray* quaternion_to_matlab (hxQuaternion* h);
mxArray* transform_to_matlab (hxTransform* h);
mxArray* link_to_matlab (hxLink* h);
mxArray* wrench_to_matlab (hxWrench* h);
mxArray* joint_to_matlab (hxJoint* h);
mxArray* model_to_matlab (hxModel* h);
mxArray* contactpoint_to_matlab (hxContactPoint* h);
mxArray* color_to_matlab (hxColor* h);

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
  else if (!strcmp(funcName, "siminfo"))
    hxgzs_siminfo(nlhs, plhs, nrhs-1, prhs+1);
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

mxArray*
vector3_to_matlab (hxVector3* h)
{
  mxArray *array = mxCreateDoubleMatrix(3, 1, mxREAL);
  double *data = mxGetPr(array);
  data[0] = h->x;
  data[1] = h->y;
  data[2] = h->z;

  return array;
}

mxArray*
quaternion_to_matlab (hxQuaternion* h)
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
transform_to_matlab (hxTransform* h)
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
link_to_matlab (hxLink* h)
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
wrench_to_matlab (hxWrench* h)
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
joint_to_matlab (hxJoint* h)
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
model_to_matlab (hxModel* h)
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
contactpoint_to_matlab (hxContactPoint* h)
{
  const char *keys[] = {"link1",
                        "link2",
                        "point",
                        "normal",
                        "distance",
                        "wrench"};
  mxArray *s = mxCreateStructMatrix(1, 1, 6, keys);

  mxArray *link1Array = mxCreateString(h->link1);
  mxArray *link2Array = mxCreateString(h->link2);
  mxArray *pointArray = vector3_to_matlab(&(h->point));
  mxArray *normalArray = vector3_to_matlab(&(h->normal));
  mxArray *distanceArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *wrenchArray = wrench_to_matlab(&(h->wrench));

  double* distanceData = mxGetPr(distanceArray);

  distanceData[0] = h->distance;

  mxSetField(s, 0, "link1", link1Array);
  mxSetField(s, 0, "link2", link2Array);
  mxSetField(s, 0, "point", pointArray);
  mxSetField(s, 0, "normal", normalArray);
  mxSetField(s, 0, "distance", distanceArray);
  mxSetField(s, 0, "wrench", wrenchArray);

  return s;
}

mxArray*
color_to_matlab (hxColor* h)
{
  mxArray *array = mxCreateDoubleMatrix(4, 1, mxREAL);
  double *data = mxGetPr(array);
  data[0] = h->r;
  data[1] = h->g;
  data[2] = h->b;
  data[3] = h->alpha;

  return array;
}

hxVector3*
matlab_to_vector(mxArray* m)
{
}

hxQuaternion*
matlab_to_quaternion(mxArray* m)
{
}

hxTransform*
matlab_to_transform(mxArray* m)
{
}

hxColor*
matlab_to_color(mxArray* m)
{
}

void
hxgzs_siminfo (int nlhs, mxArray *plhs[],
               int nrhs, const mxArray *prhs[])
{
  hxSimInfo h;

  // Request robot information.
  if (hxs_siminfo(&h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_siminfo", hx_last_result());

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
  hxTransform h;

  // Request robot information.
  if (hxs_camera_transform(&h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_camera_transform", hx_last_result());

  plhs[0] = transform_to_matlab(&h);
}

void
hxgzs_set_camera_transform (int nlhs, mxArray *plhs[],
                            int nrhs, const mxArray *prhs[])
{
  hxTransform h;

  // Request robot information.
  if (hxs_camera_transform(&h) != hxOK)
    mexErrMsgIdAndTxt("HAPTIX:hxs_camera_transform", hx_last_result());

  plhs[0] = transform_to_matlab(&h);
}
