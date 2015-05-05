%HXS_SET_MODEL_LINK_STATE Set simulation state (position and velocity) of link
% to the desired position and velocity. The link
% acceleration may change based on the constraints of model's dynamic system.
%
% set_model_link_state(model, link, transform, lin_vel, ang_vel)
%
% Parameters:
%   model (string) : Name of the model to set.
%   link (string) : Name of the link to set.
%   transform (transform) : Desired position and orientation of the link.
%   lin_vel (vector3) : Desired linear velocity of the link (m/s).
%   ang_vel (vector3) : Desired angular_velocity velocity of the link (rad/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_model_link_state(model, link, transform, lin_vel, ang_vel)
  hxgz("set_model_link_state", model, link, transform, lin_vel, ang_vel);
end
