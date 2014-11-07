#include "mex.h"
#include "haptix/comm/haptix.h"

void
mexFunction (int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[])
{
  // Sanity check: Verify that the first input argument is a struct.
  if (nrhs != 0)
    mexErrMsgTxt("Expects no arguments");

  // Request device information.
  if (hx_close(hxGAZEBO) != hxOK)
    mexPrintf("hx_close(): Request error.\n");
}
