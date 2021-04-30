clc;
clear;

installMatlabAndSimulink

pkgMATLAB = 0;
pkgSimulink = 0;
supportpkg=matlabshared.supportpkg.getInstalled; % Query for installed support packages
for i=1:length(supportpkg)

    pkgMATLAB = pkgMATLAB + strcmp(supportpkg(i).Name,'MATLAB Support Package for Arduino Hardware');
    pkgpSimulink = pkgSimulink + strcmp(supportpkg(i).Name,'Simulink Support Package for Arduino Hardware');
end
if ~pkgMATLAB
       disp('WARNING! Arduino HW package for MATLAB not found. Open Add-Ons > Get Hardware Support Packages.')
end
if ~pkgSimulink
       disp('WARNING! Arduino HW package for Simulink not found. Open Add-Ons > Get Hardware Support Packages.')
end

testFailedCI = 0;               % Flag to tell if any test has failed
cd matlab/examples/             % Move to examples folder


exampleList = {'OptoShield','HeatShield','LinkShield','MotoShield','BoBShield','FloatShield'};

testScripts(exampleList)                   % Finds and tests all scripts, excludes exceptions caused by lack of hardware packages or physical hardware

if testFailedCI
    error('+++++++++++ At least one of the tests failed! +++++++++++ '); % Try-catch will prevent MATALAB from throwing an overall error. This fails the CI process.%end
else
    disp('+++++++++++ All tests passed. +++++++++++ '); 
end

function testScripts(exampleList)

for i=1:length(exampleList);
    cd(exampleList{i})
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
            catch exceptionCI
                exceptionCIKnown = {
                    'MATLAB:serial:fopen:opfailed',                            % Failed to open serial port
                    'MATLAB:hwsdk:general:boardNotDetected',                   % No hardware board
                    'MATLAB:hwsdk:general:invalidAddressPCMac'                 % Has HW extension, but no such address
                    ''                                                         % This empty identificator is for the Hardware Support Package
                    };
                knownID = 0;        % Assume it is an unknown error
                for i=1:length(exceptionCIKnown) % For the length of known exceptions (missing hardware)
                    
                    knownID = knownID+strcmp(exceptionCI.identifier,exceptionCIKnown{i});
                end
                if ~knownID
                    testFailedCI = 1;
                    failFunctionCI(exceptionCI);
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

function failFunctionCI(exceptionCI)
    fprintf('FAIL.\n')
    exceptionCI
    %rethrow(exceptionCI) % Only if you want to stop CI instantly on the first
    %error.
end


function out = listDir
    persistent dirContents
    dirContents=dir('**/*.m');
    out = dirContents;
end


