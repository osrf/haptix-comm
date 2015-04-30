%HXS_RESET
%
% result = reset()
%
%TODO: document

function hxs_reset(reset_limb_pose)
  hxgz("reset", reset_limb_pose);
end
