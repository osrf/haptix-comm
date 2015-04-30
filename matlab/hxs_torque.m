%HXS_TORQUE
%
% torque(model, link, torque, duration)
%
%TODO: document

function hxs_torque(model, link, torque, duration)
  hxgz("torque", model, link, torque, duration);
end
