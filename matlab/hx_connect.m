%HX_CONNECT Connect to robot/simulator
%
% hx_connect(host, port)
%
% This function should be called before any other HAPTIX-related functions.
%
% Parameters:
%   host (string) : The name/address of the simulator/robot to use.
%   port (int) : The TCP/UDP port of the simulator/robot to use.
%
% Not all simulators/robots require the host and/or port parameters.  Use
% empty/zero values to use automatic discovery.
%
% Throws an error if something failed.
%
% See also HX_CLOSE, HX_READ_SENSORS, HX_ROBOT_INFO, and HX_UPDATE
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>
% and/or
% <a href="matlab:
% web('http://mujoco.org/haptix.html#hxMATLAB')">the MuJoCo HAPTIX site</a>.

function hx_connect()
  hxgz('connect');
end
