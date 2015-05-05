%HXS_IS_LOGGING Determine if logging is running.
%
% result = is_logging()
%
% Return values:
%   result (bool) : 1 if logging is running, 0 otherwise.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_is_logging()
  result = hxgz("is_logging");
end
