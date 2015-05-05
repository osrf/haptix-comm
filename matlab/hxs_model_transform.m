%HXS_MODEL_TRANSFORM Get model transform.
%
% result = hxs_model_transform(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (transform) : Current model transform.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_model_transform(model)
  result = hxgz("model_transform", model);
end
