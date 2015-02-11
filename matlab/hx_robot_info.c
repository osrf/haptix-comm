#include <stdint.h>
#include "mex.h"
#include "haptix/comm/haptix.h"

void
mexFunction (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  hxRobotInfo robotInfo;
  int i;

  // Initialize the function result to 0.
  mxArray *result = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *resultData = mxGetPr(result);
  resultData[0] = 0;

  // Request robot information.
  if (hx_robot_info(&robotInfo) != hxOK)
  {
    mexPrintf("hx_robot_info(): Request error.\n");
    resultData[0] = -1.0;
    plhs[1] = result;
  }

  // Create a Matlab structure array.
  const char *keys[] = {"motor", "joint",
    "contact_sensor", "imu", "joint_limit", "motor_limit"};
  mxArray *s = mxCreateStructMatrix (1, 1, 5, keys);

  // Create the empty mxArrays for the structure array.
  mxArray *motorArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *jointArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *contactSensorArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *imuArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *jointLimitsArray = mxCreateDoubleMatrix(
      robotInfo.joint_count, 2, mxREAL);
  mxArray *motorLimitsArray = mxCreateDoubleMatrix(
      robotInfo.motor_count, 2, mxREAL);

  // Get the pointers to the data.
  double *motorData = mxGetPr(motorArray);
  double *jointData = mxGetPr(jointArray);
  double *contactSensorArrayData = mxGetPr(contactSensorArray);
  double *imuData = mxGetPr(imuArray);
  double *jointLimitsData = mxGetPr(jointLimitsArray);
  double *motorLimitsData = mxGetPr(jointLimitsArray);

  // Fill the nmotor field.
  motorData[0] = robotInfo.motor_count;

  // Fill the njoint field.
  jointData[0] = robotInfo.joint_count;

  // Fill the ncontactsensor field.
  contactSensorArrayData[0] = robotInfo.contact_sensor_count;

  // Fill the nIMU field.
  imuData[0] = robotInfo.imu_count;

  // Fill the joint limits.
  for (i = 0; i < robotInfo.joint_count; ++i)
  {
    jointLimitsData[i] = robotInfo.joint_limit[i][0];
    jointLimitsData[i + robotInfo.joint_count] = robotInfo.joint_limit[i][1];
  }

  // Fill the motor limits.
  for (i = 0; i < robotInfo.motor_count; ++i)
  {
    motorLimitsData[i] = robotInfo.motor_limit[i][0];
    motorLimitsData[i + robotInfo.motor_count] = robotInfo.motor_limit[i][1];
  }

  // Set the structure array fields.
  mxSetField(s, 0, "motor", motorArray);
  mxSetField(s, 0, "joint", jointArray);
  mxSetField(s, 0, "contact_sensor", contactSensorArray);
  mxSetField(s, 0, "imu", imuArray);
  mxSetField(s, 0, "joint_limit", jointLimitsArray);
  mxSetField(s, 0, "motor_limit", motorLimitsArray);

  // Set the output arguments.
  plhs[0] = s;
  plhs[1] = result;
}
