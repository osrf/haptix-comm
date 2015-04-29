/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <stdio.h>
#include <haptix/comm/haptix_sim.h>

#ifdef _WIN32
#include <windows.h>
#endif

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  hxSimInfo simInfo;
  hxTransform cameraTransform;
  hxContactPoints contactPoints;
  hxTime time;
  hxColor color;

  // Requesting simulation information.
  if (hxs_siminfo(&simInfo) != hxOK)
  {
    printf("hxs_siminfo(): Request error.\n");
    return -1;
  }

  if (hxs_camera_transform(&cameraTransform) != hxOK)
  {
    printf("hxs_camera_transform(): Request error.\n");
    return -1;
  }

  if (hxs_set_camera_transform(&cameraTransform) != hxOK)
  {
    printf("hxs_set_camera_transform(): Request error.\n");
    return -1;
  }

  if (hxs_contacts("cricket_ball", &contactPoints) != hxOK)
  {
    printf("hxs_contacts(): Request error.\n");
    return -1;
  }

  // etc. etc.

  /*if (hxs_timer(&time) != hxOK)
  {
    printf("hxs_timer(): Request error.\n");
    return -1;
  }*/

  if (hxs_model_color("cricket_ball", &color) != hxOK)
  {
    printf("hxs_model_color(): Request error.\n");
    return -1;
  }

  if (hxs_set_model_color("cricket_ball", &color) != hxOK)
  {
    printf("hxs_set_model_color(): Request error.\n");
    return -1;
  }

  return 0;
}
