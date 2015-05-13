%HXS_LINEAR_VELOCITY Get the linear velocity of a model.
%
% result = hxs_linear_velocity(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (vector3) : Velocity (m/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_linear_velocity(model)
  result = hxgz('linear_velocity', model);
end
