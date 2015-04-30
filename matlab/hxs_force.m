%HXS_FORCE
%
% result = force()
%
%TODO: document

function result = hxs_force(model, link, velocity, duration)
  result = hxgz("force", model, link, velocity, duration);
end
