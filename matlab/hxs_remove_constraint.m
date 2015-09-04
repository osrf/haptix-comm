%HXS_REMOVE_CONSTRAINT Remove constraint.
%
% hxs_remove_constraint(name, model)
%
% Parameters:
%   name (string) : Name of the constraint.
%   model (string) : Name of the model containing the constraint.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_remove_constraint(name, model)
  hxgz('remove_constraint', name, model);
end
