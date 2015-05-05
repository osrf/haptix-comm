%HXS_REMOVE_MODEL Remove model.
%
% remove_model(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_remove_model(model)
  hxgz("remove_model", model);
end
