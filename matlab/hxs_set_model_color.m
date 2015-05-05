%HXS_SET_MODEL_COLOR Set the color of the model.
%
% set_model_color(model, color)
%
% Parameters:
%   model (string) : Name of the model.
%   color (color) : The color to set. 
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_model_color(model, color)
  hxgz("set_model_color", model, color);
end
