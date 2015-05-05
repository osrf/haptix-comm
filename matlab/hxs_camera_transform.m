%HXS_CAMERA_TRANSFORM Get information about the simulation camera.
%
% result = hxs_camera_transform()
%
% Return values:
%   result (transform) : Information about the simulation camera.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_camera_transform()
  result = hxgz("camera_transform");
end
