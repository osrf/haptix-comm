%ADD_MODEL Add model during runtime.
%
% model = add_model(sdf, name, pos, orient, gravity_mode)
%
% Parameters:
%   sdf (string) : SDF xml description of the model.
%   name (string) : Model name.
%   pos (float array) : A 3x1 (or 1x3) array (X,Y,Z) of model position in the
%     global frame (m).
%   orient (float array) : A 3x1 (or 1x3) array (roll, pitch, yaw) of model
%     orientation in the global frame (rad).
%   gravity_mode (bool) : true if the model is affected by gravity, 0 otherwise.
% 
% Return values:
%   model: Structure with the following named fields:
%     TODO    

function result = hxs_add_model(sdf, name, pos, orient, gravity_mode)
  result = hxgz("add_model", sdf, name, pos, orient, gravity_mode);
end
