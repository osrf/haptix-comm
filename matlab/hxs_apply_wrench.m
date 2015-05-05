%HXS_APPLY_WRENCH Apply a wrench to a link.
%
% apply_wrench(model, link, wrench, duration)
%
% Parameters:
%   model (string) : Name of the model containing the link.
%   link (string) : Name of the link.
%   wrench (wrench) : Wrench to apply.
%   duration (float) : Duration of the torque application (s). Set to 0
%     for persistent duration.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_apply_wrench(model, link, wrench, duration)
  hxgz("apply_wrench", model, link, wrench, duration);
end
