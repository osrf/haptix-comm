%HXS_SIM_INFO
%
% result = hxs_sim_info()
%
% Return values:
%   result: Structure with the following named fields:
%     models (model array) : Array of the models in the world.
%     camera_transform (transform) : Pose of the user camera in the world.
%
% COMMON DATA STRUCTURES
%
% The following data structures are used throughout the simulation API.  For
% simplicity, there are documented here, and referred to elsewhere in the
% documentation for specific functions.
%
% vector3 (float array) : A 3x1 array (X, Y, Z).
%
% quaternion (float array) : A 4x1 array (W, X, Y, Z) of orientation.
%
% transform: Structure with the following named fields:
%   pos (vector3) : Position of the transform.
%   orient (quaternion) : Orientation of the transform.
%
% link: Structure with the following named fields:
%   name (string) : Name of the link.
%   transform (transform) : Pose of the link with respect to some parent (e.g.,
%     model).
%   lin_vel (vector3) : Linear velocity (vX, vY, vZ) of the link (m/s).
%   ang_vel (vector3) : Angular velocity (vroll, vpitch, vyaw) of the
%     link (rad/s).
%   lin_acc (vector3) : Linear acceleration (aX, aY, aZ) of the link (m/s^2).
%   ang_acc (vector3) : Angular acceleration (aroll, apitch, ayaw) of the
%     link (m/s^2).
%
% wrench: Structure with the following named fields:
%   force (vector3) : Force (fX, fY, fZ) (N).
%   torque (vector3) : Torque (tX, tY, tZ) (Nm).
%
% joint: Structure with the following named fields:
%   name (string) : Name of the joint.
%   pos (float) : Position of the joint (rad).
%   vel (float) : Velocity of the joint (rad/s).
%   acc (float) : Acceleration of the joint (rad/s^2).
%   torque_motor (float) : Torque due to actuation (Nm).
%   wrench_reactive (wrench) : Force/torque pair due to external disturbances.
%
% model: Structure with the following named fields:
%   name (string) : Name of the model.
%   transform (transform) : Pose of the model in the global frame (m, rad).
%   links (link array): Array of links in the model.
%   joints (joint array) : Array of joints in the model.
%   gravity_mode (int) : 1 if the model is affected by gravity, 0 otherwise.
%
% contactpoints: Structure with the following named fields:
%   link1 (string) : Name of the first contacting link.
%   link2 (string) : Name of the second contacting link.
%   point (vector3) : Description of contact frame relative to link 1 frame:
%     origin of contact on link 1.
%   normal (vector3) : Description of contact frame relative to link 1 frame:
%     normal direction (unit vector) of contact force on link 1.
%   distance (float) : Normal distance (penetration depth) in link 1 frame (m).
%   wrench (wrench) : Contact force/torque pair in link 1 frame at "point".
%
% color (float array) : A 3x1 array (red, green, blue, alpha) defining a color.
%
% collidemode (enum) :
%   0 (NO_COLLIDE) means the object will pass through other objects, and the
%     simulation does not know if this event occurs. hxs_contacts will not
%     generate contact points.
%   1 (DETECTION_ONLY) means that the object will pass through other objects, and
%     the simulation will detect when the object collides. hxs_contacts will
%     generate contact points when this happens, but the force and torque values
%     of the hxContactPoint struct will be invalid.
%   2 (COLLIDE) means that the object will obey the laws of physics and the
%     simulation will generate forces when it collides with other objects.

function result = hxs_sim_info()
  result = hxgz('sim_info');
end
