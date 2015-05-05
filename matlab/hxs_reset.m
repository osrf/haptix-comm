%HXS_RESET Send world reset command/Carry over limb pose between world reset.
%
% reset(reset_limb_pose)
%
% Parameters:
%   reset_limb_pose (int) : Non-zero to reset the pose of the limb. 
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_reset(reset_limb_pose)
  hxgz("reset", reset_limb_pose);
end
