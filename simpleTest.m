% Simple MATLAB / Octave Code to test Travis CI builds using
% the .travis.yml file in the root directory of this repository.
% Gergely Takács, 2019, www.gergelytakacs.com

%% Correct operations to test
	A = rand(5)
	B = rand(5)
	I = eye(5)
	C = A./(B+I)
	c=sqrt(10);

%% Uncomment to create a bug artificially:
% 	thisIsABug