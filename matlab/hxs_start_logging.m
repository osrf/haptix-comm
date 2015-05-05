%HXS_START_LOGGING Start recording log file. Only one log file may be recorded at
% a time.
%
% hxs_start_logging(filename)
%
% Parameters:
%   filename (string) : Name of the file to log information into.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function hxs_start_logging(filename)
  hxgz("start_logging", filename);
end
