
if ~exist('CI_Testing','var') % Java will bum out on Linux in headless mode
   cftool IRF520.sfit
end