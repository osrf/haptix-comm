%HXS_MODEL_TRANSFORM Get model transform.
%
% result = hxs_model_transform(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (model) : Model with joint(s) containing states.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_model_joint_state(model)
  result = hxgz('model_joint_state', model);
end
