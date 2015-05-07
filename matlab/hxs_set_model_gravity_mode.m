%HXS_SET_MODEL_GRAVITY_MODE Set whether or not this model is affected by gravity.
%
% hxs_set_model_gravity_mode(model, gravity_mode)
%
% Parameters:
%   model (string) : Name of the model. 
%   gravity_mode (int) : If 1, the model is affected by gravity. If 0,
%     the model is free-floating.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_model_gravity_mode(model, gravity_mode)
  hxgz('set_model_gravity_mode', model, gravity_mode);
end
