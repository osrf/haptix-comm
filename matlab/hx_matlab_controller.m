%
% Copyright (C) 2014-2015 Open Source Robotics Foundation
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%
%

counter = 0;

hx_connect();

deviceInfo = hx_robot_info();

while counter < 2000
  % Initialize the command scalar structure.
  cmd.ref_pos = [];
  cmd.ref_pos_enabled = 1;
  cmd.ref_vel_max = [];
  cmd.ref_vel_max_enabled = 0;
  cmd.gain_pos = [];
  cmd.gain_pos_enabled = 0;
  cmd.gain_vel = [];
  cmd.gain_vel_enabled = 0;

  % Create a new command based on a sinusoidal wave.
  for n = 0:deviceInfo.motor_count
    cmd.ref_pos(end + 1) = 0.5 * sin(0.05 * 2.0 * pi * counter * 0.01);
    cmd.ref_vel_max(end + 1) = 1.0;
    cmd.gain_pos(end + 1) = 1.0;
    cmd.gain_vel(end + 1) = 1.0;
  end

  % Send the new joint command and receive the state update.
  state = hx_update(cmd);

  counter = counter + 1;

  pause(0.001);
end

hx_close();
