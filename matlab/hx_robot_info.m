%HX_ROBOT_INFO Get information from a simulator/robot.
%
% info = hx_robot_info()
%
% HX_CONNECT should have been called first.
%
% Return value:
%   info: Structure with the following named fields:
%     motor_count (int) : Number of motors.  The number of motors is less than or equal
%       to the number of joints. For example, one motor may control
%       several joints through kinematic joint coupling.
%     joint_count (int) : Number of joints.  The number of joints is greater than or equal
%       to the number of motors.
%     contact_sensor_count (int) : Number of contact sensors.  A contact sensor measures
%       the magnitude of the force on that sensor.
%     imu_count (int) : Number of IMUs (inertial measurement units).  An IMU or inertial
%       measurement unit measures the 3-dimensional linear acceleration
%       vector and the 3-dimensional angular velocity vector experienced by
%       the sensor.
%     motor_limit (float array) : An N by 2 array representing
%       the angular limits of each motor in the robot, where N is the maximum
%       number of motors (rad). Each 1x2 row/column of the array corresponds to
%       a motor. The first entry in the row/column is the lower limit of the motor.
%       The second entry is the upper limit of the motor.
%     joint_limit (float array) : Minimum and maximum joint angles (rad).  An M by 2
%       array representing the angular limits of each joint in the robot,
%       where M is the maximum number of joints. Each 1x2 row/column of the array
%       corresponds to a joint. The first entry in the row/column is the lower
%       limit of the joint. The second entry is the upper limit of the joint.
%     update_rate (float) : Rate at which the device will process new commands
%       and provide new sensor data (Hz).
%
% Throws an error if something failed.
%
% See also HX_CLOSE, HX_CONNECT, HX_READ_SENSORS, and HX_UPDATE
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>
% and/or
% <a href="matlab:
% web('http://mujoco.org/haptix.html#hxMATLAB')">the MuJoCo HAPTIX site</a>.

function info = hx_robot_info()
  info = hxgz('robot_info');
end
