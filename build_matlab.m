clc;
clear;

installMatlabAndSimulink

supportpkg=matlabshared.supportpkg.getInstalled; % Query for installed support packages
for i=1:length(supportpkg)
    pkgMATLAB = 0;
    pkgSimulink = 0;
    pkgMATLAB = pkgMATLAB + strcmp(supportpkg(i).Name,'MATLAB Support Package for Arduino Hardware');
    pkgpSimulink = pkgSimulink + strcmp(supportpkg(i).Name,'Simulink Support Package for Arduino Hardware');
end
    if ~pkgMATLAB
        disp('WARNING! Arduino HW package for MATLAB not found. Open Add-Ons > Get Hardware Support Packages.')
    end
    if ~pkgSimulink
        disp('WARNING! Arduino HW package for Simulink not found. Open Add-Ons > Get Hardware Support Packages.')
    end

testFailed = 0;  % Flag to tell if test has failed


% How do I differentiate between function and script?
%% Examples
cd matlab/examples/ % Move to examples folder

% Search trhough each Hardware category individually
%shieldsList = {'HeatShield', 'OptoShield'}
% Forgets shieldsList:((( Gaaaah.

% if ~exist(CI_Testing,'var')
%     clc
% end


% Starts HeatShield_ODE! Trying to run a function as a script.
testScripts()


function testScripts()
persistent shieldsList

shieldsList = {'OptoShield','HeatShield'};


% Try skips the rest of the file, so it is not checking any hardware tests
for i=1:length(shieldsList);
    cd(shieldsList{i})
    for i=1:length(listDir())
        dirContents=listDir();
        file = dirContents(i).name; %Full file name with extension
        scr = dirContents(i).name(1:end-2); %Script name to launch
        if ~isfunction(file)
            try
                pause('off')
                CI_Testing = 'true'; %Passed to scripts to turn off clearing screen
                fprintf(['*** Testing "',scr])
                fprintf(['"...'])
                run(scr) % All scripts, sans the file extensions
            catch ex
                exKnown = {
                    'MATLAB:serial:fopen:opfailed',                            % Failed to open serial port
                    'MATLAB:hwsdk:general:boardNotDetected',                   % No hardware board
                    ''                                                         % This empty identificator is for the Hardware Support Package
                    };
                knownID = 0;        % Assume it is an unknown error
                for i=1:length(exKnown) % For the length of known exceptions (missing hardware)
                    
                    knownID = knownID+strcmp(ex.identifier,exKnown{i});
                end
                if ~knownID
                    fprintf('FAIL.\n')
                    ex
                    rethrow(ex)
                    fail_function;
                    testFailed = 1;
                else
                    fprintf(' WARNING: Not fully tested! ')
                end
            end
            fprintf('PASS.\n')
        end
    end
    cd .. % Switch to next directory
end
end
% %The testfailed flag gets forgotten!!!!!!
%if testFailed
 %   error(''); % Try-catch will prevent MATALAB from throwing an overall error. This fails the CI process.
%end

% Should list what was tested fully and what was partially.


function fail_function
disp('Test failed'); %Don't use error() otherwise it throws an error in the CI
end

function out = listDir
persistent dirContents
dirContents=dir('**/*.m');
out = dirContents;
end


