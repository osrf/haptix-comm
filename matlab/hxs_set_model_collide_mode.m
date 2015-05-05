%HXS_SET_MODEL_COLLIDE_MODE Set the collide mode of the object.
%
% hxs_set_model_collide_mode(model, collide_mode)
%
% Parameters:
%   model (string) : Name of the model.
%   collide_mode (collidemode) : The collide mode of the object.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_model_collide_mode(model, collide_mode)
  hxgz("set_model_collide_mode", model, collide_mode);
end
