%HXS_CONTACTS Get information about active contacts for a model.
%
% result = hxs_contacts(model)
%
% Parameters:
%   model (string) : The name of the model to query.
%
% Return values:
%   result (contactpoints) : The latest contact information.
%
% See also HXS_SIM_INFO (for data structure definitions).
%
% For more information, see <a href="matlab:
% web('http://gazebosim.org/haptix')">the Gazebo HAPTIX site</a>.

function result = hxs_contacts(model)
  result = hxgz("contacts", model);
end
