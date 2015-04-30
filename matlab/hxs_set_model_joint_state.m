%HXS_SET_MODEL_JOINT_STATE
%
% set_model_joint_state(model, joint, position, velocity)
%
%TODO: document

function hxs_set_model_joint_state(model, joint, position, velocity)
  hxgz("set_model_joint_state", model, joint, position, velocity);
end
