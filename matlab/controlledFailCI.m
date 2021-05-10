% If the CI is running, forces script to fail with a known exception
% This is if the HW support is missing, and the "normal" exception
% is an unknown function (we don't want to add that to the list of known
% exceptions.
function controlledFailCI()

a = arduino(); % Known issue that CI runner does not have HW installed.
end
