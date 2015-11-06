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

% Uncomment this block to start logging.
% hxs_start_logging('/tmp/log/')

while counter < 250
  cmdSent = tic;

  % Initialize the command scalar structure.
  cmd.ref_pos = [];
  cmd.ref_vel_max = [];
  cmd.gain_pos = [];
  cmd.gain_vel = [];

  % Indicate that the positions we set should be used.
  cmd.ref_pos_enabled = 1;
  % We're not setting it, so indicate that ref_vel_max should be ignored.
  cmd.ref_vel_max_enabled = 0;
  % We're not setting it, so indicate that gain_pos should be ignored.
  cmd.gain_pos_enabled = 0;
  % We're not setting it, so indicate that gain_vel should be ignored.
  cmd.gain_vel_enabled = 0;

  % Create a new command based on a sinusoidal wave.
  for n = 0:deviceInfo.motor_count
    cmd.ref_pos(end + 1) = 0.5 * sin(0.05 * 2.0 * pi * counter * 0.08);
    % We could set a desired maximum velocity
    % cmd.ref_vel_max(end + 1) = 1.0;
    % We could set a desired controller position gain
    % cmd.gain_pos(end + 1) = 1.0;
    % We could set a desired controller velocity gain
    % cmd.gain_vel(end + 1) = 1.0;
  end

  % Send the new joint command and receive the state update.
  state = hx_update(cmd);

  counter = counter + 1;

  % Busy wait. pause() is not accurate enough on Windows.
  elapsedCmd = toc(cmdSent);
  while elapsedCmd < 0.02
    elapsedCmd = toc(cmdSent);
  end
end

% Uncomment this block to stop logging.
% hxs_stop_logging()

hx_close();
