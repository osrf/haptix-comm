% This example exercises every simulation API call.

info = hxs_sim_info();

% List the models in the world
disp("Models:");
for model_idx = 1:size(info.models)
  model = info.models(model_idx).model;
  disp(model.name);
  disp("Links:");
  for link_idx = 1:size(model.links)
    link = model.links(link_idx).link;
    disp(link.name);
  end
  disp("Joints:");
  for joint_idx = 1:size(model.joints)
    joint = model.joints(joint_idx).joint;
    % TODO: this doesn't work yet
    %disp(joint.name);
  end
end

% Get the user camera pose
tx = hxs_camera_transform();
% Move and rotate the user camera pose
new_tx = tx;
new_tx.pos(3) += 1;
new_tx.orient(1) += pi/4;
hxs_set_camera_transform(new_tx);
% Restore the original camera pose
sleep(1);
hxs_set_camera_transform(tx);
sleep(1);

% Change the table color.
hxs_set_model_color("table", [1;0;0;1])
sleep(1);
hxs_set_model_color("table", [0;1;0;1])
sleep(1);
hxs_set_model_color("table", [0;0;1;1])
% Get the color
color = hxs_model_color("table");
disp("Table color:");
disp(color)

% Apply force to the small wooden cube, moving it sideways.
% Show linear velocity before, during, and afterward
disp("Sliding cube:")
vel = hxs_linear_velocity("wood_cube_5cm");
disp(vel);
hxs_apply_force("wood_cube_5cm", "link", [-1.0; 0; 0], 0.2);
% Let it get moving
sleep(0.1);
vel = hxs_linear_velocity("wood_cube_5cm");
disp(vel);
% Let it settle
sleep(1.5);
vel = hxs_linear_velocity("wood_cube_5cm");
disp(vel);

% Apply torque to the small wooden cube, rotating it in place.
% Show angular velocity before, during, and afterward
disp("Spinning cube:")
vel = hxs_angular_velocity("wood_cube_5cm");
disp(vel);
hxs_apply_torque("wood_cube_5cm", "link", [0; 0; 0.1], 0.1)
% Let it get moving
sleep(0.1);
vel = hxs_angular_velocity("wood_cube_5cm");
disp(vel);
% Let it settle
sleep(1.5);
vel = hxs_angular_velocity("wood_cube_5cm");
disp(vel);

% Apply force and torque at the same time.
wrench = struct("force", [0; 0; 1], "torque", [0; 0; 0.1])
hxs_apply_wrench("wood_cube_5cm", "link", wrench, 0.1);
sleep(1.5);

% Check gravity mode on wooden cube
gravity_mode = hxs_model_gravity_mode("wood_cube_5cm");
disp(gravity_mode);
% Turn off gravity for cube, then nudge it upward
hxs_set_model_gravity_mode("wood_cube_5cm", 0);
hxs_apply_force("wood_cube_5cm", "link", [0; 0; 0.1], 0.1);
% Let if tly
sleep(1);
% Bring it back down
hxs_set_model_gravity_mode("wood_cube_5cm", gravity_mode);

% Get the pose of the cube
tx = hxs_model_transform("wood_cube_5cm");
disp("Cube position:");
disp(tx.pos);
disp("Cube orientation:");
disp(tx.orient);

% Check collide mode on the cube
collide_mode = hxs_model_collide_mode("wood_cube_5cm");
disp(collide_mode);
% Let it drop through the world
hxs_set_model_collide_mode("wood_cube_5cm", 0);
% Hack: apply a small force to disturb the cube to make it actually fall.
hxs_apply_force("wood_cube_5cm", "link", [0; 0; 0.1], 0.1);
sleep(1);
% Turn collisions back on (won't bring the cube back, of course)
hxs_set_model_gravity_mode("wood_cube_5cm", collide_mode);

% Add a new model
% TODO
% Remove the model
% TODO

% Set the state of a wrist joint
%hxs_set_model_joint_state("mpl_haptix_right_forearm", 
