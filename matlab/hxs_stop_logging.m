%HXS_STOP_LOGGING Stop recording log file.
%
% hxs_stop_logging()
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_stop_logging()
  hxgz('stop_logging');
end
