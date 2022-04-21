% This function is a copy of the "map" function used in Arduino code.
% It takes an input value mapped from a min and max possible value and
% scales it to a desired output min and max.
%
% [output] = mapfun(value,fromLow,fromHigh,toLow,toHigh)
% value: can be a scalar entry or a matrix of values
% fromLow: lowest possible input value
% fromHigh: highest possible input value
% toLow: lowest desired output value
% toHigh: highest possible output value
% [output]: output matrix of same size as value
% 


% examples:
% output = mapfun(-.3,-1,1,0,5)
% output =
%
%    1.7500
%
% output = mapfun([-1 1 -.5 .5],-1,1,0,5)
% output =
%

function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
narginchk(5,5)
nargoutchk(0,1)
output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end
