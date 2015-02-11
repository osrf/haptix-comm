#include <stdint.h>
#include "mex.h"
#include "haptix/comm/haptix.h"

void
mexFunction (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  hxCommand cmd;
  hxSensor sensor;
  int i;
  mxArray *v;
  double *data;

  // Initialize the function result to 0.
  mxArray *result = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *resultData = mxGetPr(result);
  resultData[0] = 0;

  // Sanity check: Verify that the first input argument is a struct.
  if (nrhs != 1 || !mxIsStruct(prhs[0]))
    mexErrMsgTxt("Expects struct");

  // Sanity check: Verify that the struct has 4 fields: ref_pos, ref_vel_max,
  // gain_pos and gain_vel.
  if (mxGetNumberOfFields(prhs[0]) != 4)
    mexErrMsgTxt("Expects 4 fields");

  // ToDo: Use mxGetFieldNameByNumber() to verify that the fields names are
  // good. We should check the lenth of the arrays inside each field too.

  // Get the number of elements based on the size of the 'ref_pos' array.
  v = mxGetField(prhs[0], 0, "ref_pos");
  mwSize *cmdSize = mxGetDimensions(v);

  // Set the hxCommand struct.
  for (i = 0; i < cmdSize[1]; ++i)
  {
    v = mxGetField(prhs[0], 0, "ref_pos");
    data = mxGetPr(v);
    cmd.ref_pos[i] = data[i];

    v = mxGetField(prhs[0], 0, "ref_vel_max");
    data = mxGetPr(v);
    cmd.ref_vel_max[i] = data[i];

    v = mxGetField(prhs[0], 0, "gain_pos");
    data = mxGetPr(v);
    cmd.gain_pos[i] = data[i];

    v = mxGetField(prhs[0], 0, "gain_vel");
    data = mxGetPr(v);
    cmd.gain_vel[i] = data[i];
  }

  // Request robot information.
  if (hx_update(&cmd, &sensor) != hxOK)
  {
    mexPrintf("hx_update(): Request error.\n");
    resultData[0] = -1.0;
    plhs[1] = result;
  }

  // Create a Matlab structure array.
  const char *keys[] = {"motor_pos", "motor_vel", "motor_torque", "joint_pos",
    "joint_vel", "contact", "imu_linear_acc", "imu_angular_vel"};
  mxArray *s = mxCreateStructMatrix (1, 1, 8, keys);

  // Create the empty mxArrays for the structure array.
  mxArray *motorPosArray = mxCreateDoubleMatrix(hxMAXMOTOR, 1, mxREAL);
  mxArray *motorVelArray = mxCreateDoubleMatrix(hxMAXMOTOR, 1, mxREAL);
  mxArray *motorTorqueArray = mxCreateDoubleMatrix(hxMAXMOTOR, 1, mxREAL);
  mxArray *jointPosArray = mxCreateDoubleMatrix(hxMAXJOINT, 1, mxREAL);
  mxArray *jointVelArray = mxCreateDoubleMatrix(hxMAXJOINT, 1, mxREAL);
  mxArray *contactArray = mxCreateDoubleMatrix(hxMAXCONTACTSENSOR, 1, mxREAL);
  mxArray *imuLinAccArray = mxCreateDoubleMatrix(hxMAXIMU, 3, mxREAL);
  mxArray *imuAngVelArray = mxCreateDoubleMatrix(hxMAXIMU, 3, mxREAL);

  // Get the pointers to the data.
  double *motorPosData = mxGetPr(motorPosArray);
  double *motorVelData = mxGetPr(motorVelArray);
  double *motorTorqueData = mxGetPr(motorTorqueArray);
  double *jointPosData = mxGetPr(jointPosArray);
  double *jointVelData = mxGetPr(jointVelArray);
  double *contactData = mxGetPr(contactArray);
  double *imuLinAccData = mxGetPr(imuLinAccArray);
  double *imuAngVelData = mxGetPr(imuAngVelArray);

  // Fill the motor fields.
  for (i = 0; i < hxMAXMOTOR; ++i)
  {
    motorPosData[i] = (double)sensor.motor_pos[i];
    motorVelData[i] = (double)sensor.motor_vel[i];
    motorTorqueData[i] = (double)sensor.motor_torque[i];
  }

  // Fill the joint fields.
  for (i = 0; i < hxMAXJOINT; ++i)
  {
    jointPosData[i] = (double)sensor.joint_pos[i];
    jointVelData[i] = (double)sensor.joint_vel[i];
  }

  // Fill the contact sensor field.
  for (i = 0; i < hxMAXCONTACTSENSOR; ++i)
    contactData[i] = (double)sensor.contact[i];

  // Fill the IMU fields.
  for (i = 0; i < hxMAXIMU; ++i)
  {
    imuLinAccData[i] = (double)sensor.imu_linear_acc[i][0];
    imuLinAccData[i + hxMAXIMU] = (double)sensor.imu_linear_acc[i][1];
    imuLinAccData[i + 2 * hxMAXIMU] = (double)sensor.imu_linear_acc[i][2];

    imuAngVelData[i] = (double)sensor.imu_angular_vel[i][0];
    imuAngVelData[i + hxMAXIMU] = (double)sensor.imu_angular_vel[i][1];
    imuAngVelData[i + 2 * hxMAXIMU] = (double)sensor.imu_angular_vel[i][2];
  }

  // Set the structure array fields.
  mxSetField(s, 0, "motor_pos", motorPosArray);
  mxSetField(s, 0, "motor_vel", motorVelArray);
  mxSetField(s, 0, "motor_torque", motorTorqueArray);
  mxSetField(s, 0, "joint_pos", jointPosArray);
  mxSetField(s, 0, "joint_vel", jointVelArray);
  mxSetField(s, 0, "contact", contactArray);
  mxSetField(s, 0, "imu_linear_acc", imuLinAccArray);
  mxSetField(s, 0, "imu_angular_vel", imuAngVelArray);

  // Set the output arguments.
  plhs[0] = s;
  plhs[1] = result;
}
