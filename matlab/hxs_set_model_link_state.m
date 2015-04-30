%HXS_SET_MODEL_LINK_STATE
%
% set_model_link_state(model, link, transform, lin_vel, ang_vel)
%
%TODO: document

function hxs_set_model_link_state(model, link, transform, lin_vel, ang_vel)
  hxgz("set_model_link_state", model, link, transform, lin_vel, ang_vel);
end
