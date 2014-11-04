%
% Copyright (C) 2014 Open Source Robotics Foundation
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

[deviceInfo, result] = hx_getdeviceinfo();
if result ~= 0
  exit;
end

while counter < 2000
  % Initialize the command scalar structure.
  cmd.ref_pos  = [];
  cmd.ref_vel  = [];
  cmd.gain_pos = [];
  cmd.gain_vel = [];

  % Create a new command based on a sinusoidal wave.
  for n = 0:deviceInfo.nmotor
    cmd.ref_pos(end + 1) = 0.5 * sin(0.05 * 2.0 * pi * counter * 0.01);
    cmd.ref_vel(end + 1) = 1.0;
    cmd.gain_pos(end + 1) = 1.0;
    cmd.gain_vel(end + 1) = 1.0;
  end

  % Send the new joint command and receive the state update.
  [state, result] = hx_update(cmd);
  if result ~= 0
    disp("hx_update(): Request error.");
  end

  counter = counter + 1;

  pause(0.001);
end
