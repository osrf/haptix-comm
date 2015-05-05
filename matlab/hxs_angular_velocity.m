%HXS_ANGULAR_VELOCITY Get the angular velocity of a model.
%
% result = hxs_angular_velocity(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (vector3) : Angular velocity (rad/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_angular_velocity(model)
  result = hxgz("angular_velocity", model);
end
