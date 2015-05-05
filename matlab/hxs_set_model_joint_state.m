%HXS_SET_MODEL_JOINT_STATE Set simulation state (position and velocity) of joint
% to the desired position and velocity. The acceleration,
% torque, and reaction wrench of the joint may change based on the constraints
% of model's dynamic system.
%
% hxs_set_model_joint_state(model, joint, position, velocity)
%
% Parameters:
%   model (string) :  Name of the model to set.
%   joint (string) : Name of the joint to set.
%   position (float) : Desired position of the joint (rad).
%   velocity (float) : Desired velocity of the joint (rad/s).
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_model_joint_state(model, joint, position, velocity)
  hxgz("set_model_joint_state", model, joint, position, velocity);
end
