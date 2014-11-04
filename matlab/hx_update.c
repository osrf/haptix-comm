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

  // Sanity check: Verify that the struct has 4 fields: ref_pos, ref_vel,
  // gain_pos and gain_vel.
  if (mxGetNumberOfFields(prhs[0]) != 4)
    mexErrMsgTxt("Expects 4 fields");

  // ToDo: Use mxGetFieldNameByNumber() to verify that the fields names are
  // good. We should check the lenth of the arrays inside each field too.

  // Set the hxCommand struct.
  int cmdSize = sizeof(cmd.ref_pos) / sizeof(cmd.ref_pos[0]);
  for (i = 0; i < cmdSize; ++i)
  {
    v = mxGetField(prhs[0], 0, "ref_pos");
    data = mxGetPr(v);
    cmd.ref_pos[i] = data[i];

    v = mxGetField(prhs[0], 0, "ref_vel");
    data = mxGetPr(v);
    cmd.ref_vel[i] = data[i];

    v = mxGetField(prhs[0], 0, "gain_pos");
    data = mxGetPr(v);
    cmd.gain_pos[i] = data[i];

    v = mxGetField(prhs[0], 0, "gain_vel");
    data = mxGetPr(v);
    cmd.gain_vel[i] = data[i];
  }

  // Request device information.
  if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
  {
    mexPrintf("hx_update(): Request error.\n");
    resultData[0] = -1.0;
    plhs[1] = result;
  }

  // Create a Matlab structure array.
  const char *keys[] = {"motor_pos", "motor_vel", "motor_torque", "joint_pos",
    "joint_vel", "contact", "IMU_linacc", "IMU_angvel"};
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
    imuLinAccData[i] = (double)sensor.IMU_linacc[i][0];
    imuLinAccData[i + hxMAXIMU] = (double)sensor.IMU_linacc[i][1];
    imuLinAccData[i + 2 * hxMAXIMU] = (double)sensor.IMU_linacc[i][2];

    imuAngVelData[i] = (double)sensor.IMU_angvel[i][0];
    imuAngVelData[i + hxMAXIMU] = (double)sensor.IMU_angvel[i][1];
    imuAngVelData[i + 2 * hxMAXIMU] = (double)sensor.IMU_angvel[i][2];
  }

  // Set the structure array fields.
  mxSetField(s, 0, "motor_pos", motorPosArray);
  mxSetField(s, 0, "motor_vel", motorVelArray);
  mxSetField(s, 0, "motor_torque", motorTorqueArray);
  mxSetField(s, 0, "joint_pos", jointPosArray);
  mxSetField(s, 0, "joint_vel", jointVelArray);
  mxSetField(s, 0, "contact", contactArray);
  mxSetField(s, 0, "IMU_linacc", imuLinAccArray);
  mxSetField(s, 0, "IMU_angvel", imuAngVelArray);

  // Set the output arguments.
  plhs[0] = s;
  plhs[1] = result;
}
