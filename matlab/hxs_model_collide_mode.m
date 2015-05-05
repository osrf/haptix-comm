%HXS_MODEL_COLLIDE_MODE Get the collide mode of the object.
%
% result = hxs_model_collide_mode(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (collidemode) : The collide mode of the object.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_model_collide_mode(model)
  result = hxgz("model_collide_mode", model);
end
