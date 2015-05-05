%HXS_APPLY_FORCE Apply force to a link.
%
% apply_force(model, link, force, duration)
%
% Parameters:
%   model (string) : Name of the model containing the link.
%   link (string) : Name of the link.
%   force (vector3) : Force (N).
%   duration (float) : Duration of the force application (s). Set to 0
%     for persistent duration.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_apply_force(model, link, force, duration)
  hxgz("apply_force", model, link, force, duration);
end
