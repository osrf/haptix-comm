%HXS_ADD_CONSTRAINT Add constraint during runtime.
%
% constraint = hxs_add_constraint(sdf, name, pos, orient, gravity_mode)
%
% Parameters:
%   sdf (string) : SDF xml description of the constraint.
%   model (string) : Parent model name for the new constraint.
% 
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_add_constraint(sdf, model)
  result = hxgz('add_constraint', sdf, model);
end
