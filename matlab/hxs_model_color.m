%HXS_MODEL_COLOR Get the color of the model.
%
% result = hxs_model_color(model)
%
% Parameters:
%   model (string) : Name of the model.
%
% Return values:
%   result (color) : The color of the model.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_model_color(model)
  result = hxgz('model_color', model);
end
