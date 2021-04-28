installMatlabAndSimulink


supportpkg=matlabshared.supportpkg.getInstalled; % Query for installed support packages
for i=1:length(supportpkg)
    pkgMATLAB = 0;
    pkgSimulink = 0;
    pkgMATLAB = pkgMATLAB + strcmp(supportpkg(i).Name,'MATLAB Support Package for Arduino Hardware')
    pkgpSimulink = pkgSimulink + strcmp(supportpkg(i).Name,'Simulink Support Package for Arduino Hardware')
    if ~pkgMATLAB
        disp('WARNING! Arduino HW package for MATLAB not found. Open Add-Ons > Get Hardware Support Packages.')
    end
    if ~pkgSimulink
        disp('WARNING! Arduino HW package for Simulink not found. Open Add-Ons > Get Hardware Support Packages.')
    end
end


% How do I differentiate between function and script?
%% Examples
cd matlab/examples/ % Move to examples folder

% Search trhough each Hardware category individually
%shieldsList = {'HeatShield', 'OptoShield'}
% Forgets shieldsList:((( Gaaaah.

shieldsList = {'HeatShield'};


% Try skips the rest of the file, so it is not checking any hardware tests
for i=1:length(shieldsList);
    cd(shieldsList{i})
    for i=1:length(listDir())
        dirContents=listDir();
        scr = dirContents(i).name(1:end-2);
        try
            pause('off')
            run(scr) % All scripts, sans the file extensions
        catch ex
            exKnown = {
                'MATLAB:serial:fopen:opfailed',         % Failed to open serial port
                'MATLAB:hwsdk:general:boardNotDetected', % No hardware board
                '' %This empty identificator is for the Hardware Support Package
                };
            for i=1:length(exKnown) % For the length of known exceptions (missing hardware)
                knownID = 0;        % Assume it is an unknown error
                knownID = knownID+strcmp(ex.identifier,exKnown{i});
            end
            if ~knownID
                %rethrow(ex)
                ex
                fail_function;
            end
        end
    end
    cd ..
end


function fail_function
    disp('Test failed');
    error('Will this fail the CI process?')
end

function out = listDir
persistent dirContents
    dirContents=dir('**/*.m');
    out = dirContents;
end   


