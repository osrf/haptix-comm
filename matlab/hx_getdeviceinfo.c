#include <stdint.h>
#include "mex.h"
#include "haptix/comm/haptix.h"

void
mexFunction (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  hxDeviceInfo deviceInfo;
  int i;

  // Initialize the function result to 0.
  mxArray *result = mxCreateDoubleMatrix(1, 1, mxREAL);
  double *resultData = mxGetPr(result);
  resultData[0] = 0;

  // Request device information.
  if (hx_getdeviceinfo(hxGAZEBO, &deviceInfo) != hxOK)
  {
    mexPrintf("hx_getdeviceinfo(): Request error.\n");
    resultData[0] = -1.0;
    plhs[1] = result;
  }

  // Create a Matlab structure array.
  const char *keys[] = {"nmotor", "njoint", "ncontactsensor", "nIMU", "limit"};
  mxArray *s = mxCreateStructMatrix (1, 1, 5, keys);

  // Create the empty mxArrays for the structure array.
  mxArray *nMotorArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *nJointArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *nContactSensorArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *nImuArray = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxArray *limitsArray = mxCreateDoubleMatrix(deviceInfo.njoint, 2, mxREAL);

  // Get the pointers to the data.
  double *nMotorData = mxGetPr(nMotorArray);
  double *nJointData = mxGetPr(nJointArray);
  double *nContactSensorArrayData = mxGetPr(nContactSensorArray);
  double *nImuData = mxGetPr(nImuArray);
  double *limitsData = mxGetPr(limitsArray);

  // Fill the nmotor field.
  nMotorData[0] = deviceInfo.nmotor;

  // Fill the njoint field.
  nJointData[0] = deviceInfo.njoint;

  // Fill the ncontactsensor field.
  nContactSensorArrayData[0] = deviceInfo.ncontactsensor;

  // Fill the nIMU field.
  nImuData[0] = deviceInfo.nIMU;

  // Fill the joint limits.
  for (i = 0; i < deviceInfo.njoint; ++i)
  {
    limitsData[i] = deviceInfo.limit[i][0];
    limitsData[i + deviceInfo.njoint] = deviceInfo.limit[i][1];
  }

  // Set the structure array fields.
  mxSetField(s, 0, "nmotor", nMotorArray);
  mxSetField(s, 0, "njoint", nJointArray);
  mxSetField(s, 0, "ncontactsensor", nContactSensorArray);
  mxSetField(s, 0, "nIMU", nImuArray);
  mxSetField(s, 0, "limit", limitsArray);

  // Set the output arguments.
  plhs[0] = s;
  plhs[1] = result;
}
