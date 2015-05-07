%HXS_APPLY_TORQUE Apply torque to a link.
%
% hxs_apply_torque(model, link, torque, duration)
%
% Parameters:
%   model (string) : Name of the model containing the link.
%   link (string) : Name of the link.
%   torque (vector3) : Torque (Nm).
%   duration (float) : Duration of the torque application (s). Set to 0
%     for persistent duration.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_apply_torque(model, link, torque, duration)
  hxgz('apply_torque', model, link, torque, duration);
end
