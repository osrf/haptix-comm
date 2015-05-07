%HXS_SET_ANGULAR_VELOCITY Set the angular velocity of a model.
%
% hxs_set_angular_velocity(model, velocity)
%
% Parameters:
%   model (string) : Name of the model.
%   velocity (vector3) : Angular velocity (rad/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_angular_velocity(model, velocity)
  hxgz('set_angular_velocity', model, velocity);
end
