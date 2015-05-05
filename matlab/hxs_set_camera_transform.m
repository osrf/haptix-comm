%HXS_SET_CAMERA_TRANSFORM Set camera transform.
%
% set_camera_transform(transform)
%
% Parameters:
%   transform (transform) : New camera transform.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_set_camera_transform(transform)
  hxgz("set_camera_transform", transform);
end
