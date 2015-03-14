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

targetWristPos = 1.0;
kThreshold = 0.00001;
samples = [];

hx_connect();

deviceInfo = hx_robot_info();

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

state = hx_update(cmd);

% Let the hand reach the target.
pause(2)

dPos = abs(targetWristPos - state.motor_pos(3));
lastDPos = dPos;

cmd.ref_pos(3) = targetWristPos;

cmdSent = tic;
lastPrint = tic;

while true

  % Send the new joint command and receive the state update.
  state = hx_update(cmd);

  dPos = abs(targetWristPos - state.motor_pos(3));

  if (lastDPos - dPos > kThreshold)
    % Update stats.
    elapsedCmd = toc(cmdSent);
    samples(end + 1) = elapsedCmd * 1000.0;

    % Change wrist direction.
    targetWristPos = -targetWristPos;
    cmd.ref_pos(3) = targetWristPos;

    dPos = abs(targetWristPos - state.motor_pos(3));
    cmdSent = tic;
  end

  lastDPos = dPos;

  % Time to print stats?
  elapsedPrint = toc(lastPrint);
  if (elapsedPrint > 1.0)
    fprintf('Commands stats:\n\tMean: %f ms\n\tMedian: %f ms\n\n',
      mean(samples), median(samples))
    lastPrint = tic;
  end

end

hx_close();
