%HXS_ADD_MODEL Add model during runtime.
%
% model = add_model(sdf, name, pos, orient, gravity_mode)
%
% Parameters:
%   sdf (string) : SDF xml description of the model.
%   name (string) : Model name.
%   pos (vector3) : Model position in the global frame (m).
%   orient (vector3) : Model orientation in the global frame (rad).
%   gravity_mode (bool) : 1 if the model is affected by gravity, 0 otherwise.
% 
% Return values:
%   model (model) : Information about the model that was created.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_add_model(sdf, name, pos, orient, gravity_mode)
  result = hxgz("add_model", sdf, name, pos, orient, gravity_mode);
end
