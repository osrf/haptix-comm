%HXS_SET_MODEL_TRANSFORM Set model transform.
%
% hxs_set_model_transform(model)
%
% Parameters:
%   model (string) : Name of the model.
%   transform (transform) : Model transform.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_model_transform(model, transform)
  hxgz("set_model_transform", model, transform);
end
