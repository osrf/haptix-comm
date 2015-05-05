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
sleep(0.1)
vel = hxs_linear_velocity("wood_cube_5cm");
disp(vel);
% Let it settle
sleep(1.5)
vel = hxs_linear_velocity("wood_cube_5cm");
disp(vel);

% Apply torque to the small wooden cube, rotating it in place.
% Show angular velocity before, during, and afterward
disp("Spinning cube:")
vel = hxs_angular_velocity("wood_cube_5cm");
disp(vel);
hxs_apply_torque("wood_cube_5cm", "link", [0; 0; 0.1], 0.1)
% Let it get moving
sleep(0.1)
vel = hxs_angular_velocity("wood_cube_5cm");
disp(vel);
% Let it settle
sleep(1.5)
vel = hxs_angular_velocity("wood_cube_5cm");
disp(vel);

%% Apply torque to the small wooden cube, moving it sideways
hxs_apply_force("wood_cube_5cm", "link", [-1.0; 0; 0], 0.1);
% Query linear velocity
ang_vel = hxs_angular_velocity("wood_cube_5cm");
disp(ang_vel);

% Get the angular velocity to of the wooden cube
hxs_angular_velocity("wood_cube_5cm")
