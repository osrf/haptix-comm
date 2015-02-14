%HX_READ_SENSORS Read sensor data from simulator/robot.
%
% Similar to HX_UPDATE, this function reads the latest sensor information
% from the simulator/robot in a non-blocking manner. Use this function when
% you want to read the sensor data without sending a new command.  This
% function may be called as often as desired, but the robot is only
% guaranteed to update at the rate returned from HX_ROBOT_INFO.
%
% HX_CONNECT should have been called first.
%
% sensor = hx_read_sensors()
%
% Return values: 
%   sensor: See documentation for HX_UPDATE
%
% Throws an error if something failed.
%
% See also HX_CLOSE, HX_CONNECT, HX_ROBOT_INFO, and HX_UPDATE
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>
% and/or
% <a href="matlab:
% web('http://mujoco.org/haptix.html#hxMATLAB')">the MuJoCo HAPTIX site</a>.

function sensor = hx_read_sensors()
  sensor = hxgz('read_sensors');
end
