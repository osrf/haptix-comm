%MODEL_GRAVITY_MODE Get whether or not this model is affected by gravity.
%
% result = model_gravity_mode(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (int) : If 1, the model is affected by gravity. If 0,
%     the model is free-floating.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_model_gravity_mode(model)
  result = hxgz("model_gravity_mode", model);
end
