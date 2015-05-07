%HXS_SET_LINEAR_VELOCITY Set the linear velocity of a model.
%
% hxs_set_linear_velocity(model, velocity)
%
% Parameters:
%   model (string) : Name of the model.
%   velocity (vector3) : Velocity (m/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_linear_velocity(model, velocity)
  hxgz('set_linear_velocity', model, velocity);
end
