%   Function used to save MATLAB matrices into a header file in a format
%   compatible with BLA Arduino IDE library so they can be directly
%   included into Arduino sketch file.
% 
%   Usage:
%   To save one matrix into the 'BLA_Matrices.h' file use:
%   printBLAMatrix(matrixVariable,'matrixName')
% 
%   To save one matrix into header file with custom fileName use:
%   printBLAMatrix(matrixVariable,'matrixName','fileName')
% 
%   To save multiple matrices into the 'BLA_Matrices.h' file use:
%   printBLAMatrix(matrix1,'name1',matrix2,'name2',...)
%  
%   To save multiple matrices into header file with custom fileName use:
%   printBLAMatrix(matrix1,'name1',matrix2,'name2',...,'fileName')
% 
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by:      Peter Chmurciak.
%   Last update on:  13.5.2020.

function printBLAMatrix(varargin)
% Check if at least two input arguments were provided
if nargin < 2
    error('Not enough input argumets provided. Function expects at least two arguments - printBLAMatrix(matrix,''matrixName'').')
end

% Check whether even or odd number of input arguments were provided 
% to figure out whether the user provided a custom header fileName
if ~mod(nargin, 2)
    fileName = 'BLA_Matrices';    % Use default header file name
    nOfPairs = nargin / 2;
else
    fileName = varargin{nargin};  % Use provided header file name
    nOfPairs = (nargin - 1) / 2;
end

% Clear the file and open it for writing, print AutomationShield title
fileHandle = fopen([fileName, '.h'], 'w');
fprintf(fileHandle, '// Header file storing BLA class matrices for Arduino IDE examples.\n');
fprintf(fileHandle, '// Automatically generated by the AutomationShield library.\n');
fprintf(fileHandle, '// Visit www.automationshield.com for more information.\n');
fprintf(fileHandle, '// ================================================================\n');
fprintf(fileHandle, '\n');

% Print all of the individual pairs matrix-'matrixName' into the header
% folder, in the correct format abiding BLA library and Arduino IDE rules 
for n = 1:nOfPairs
    matrix = varargin{2*n-1};
    matrixName = varargin{2*n};
    matrixSize = size(matrix);
    vector = matrix';
    vector = vector(:)';
    vector = round(vector, 5);
    fprintf(fileHandle, 'BLA::Matrix<%d, %d> %s%s', matrixSize(1), matrixSize(2), matrixName, ' = {');
    for i = 1:length(vector)
        fprintf(fileHandle, '%.5g', vector(i));
        if i < length(vector)
            fprintf(fileHandle, '%s', ', ');
        end
    end
    fprintf(fileHandle, '%s', '};');
    fprintf(fileHandle, '\n');
end
end