%HXS_ANGULAR_VELOCITY Get the angular velocity of a model.
%
% result = angular_velocity(name)
%
% Parameters:
%   name (string) : Name of the model.
%
% Return values:
%   result (vector3) : Angular velocity (rad/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_angular_velocity(name, velocity)
  hxgz("angular_velocity", name, velocity);
end
