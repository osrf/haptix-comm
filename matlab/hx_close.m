%HX_CLOSE Close connection to a robot or simulator.
%
% hx_close()
%
% HX_CONNECT should have been called first.  After this call, do not call
% any other HAPTIX-related functions (except for HX_CONNECT).
%
% Throws an error if something failed.
%
% See also HX_CONNECT, HX_READ_SENSORS, HX_ROBOT_INFO, and HX_UPDATE
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>
% and/or
% <a href="matlab:
% web('http://mujoco.org/haptix.html#hxMATLAB')">the MuJoCo HAPTIX site</a>.

function hx_close()
  hxgz('close');
end
