%   Installs AutomationShield MATLAB API.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Gergely Takács.
%   Last update: 22.10.2018.

function installForMATLAB()
thisdir=pwd;
addpath(genpath(thisdir));
savepath
disp('AutomationShield MATLAB API added to MATLAB path.')
if ~exist('tbxmanager','dir') % If the tbxmanager is not installed (checks directory)
    disp('AutomationShield MATLAB API added to MATLAB path.')
    mkdir('tbxmanager')
    cd tbxmanager
    urlwrite('http://www.tbxmanager.com/tbxmanager.m', 'tbxmanager.m');
    
    fid = fopen('tbxmanager.m','rt+');
    % Make sure the file opened correctly
    if (fid < 1)
        error(ferror(fid))
    end
    result = false;
    line=1;
    % Go over all lines
    while ~feof(fid) && ~result
        
        str = fgetl(fid);
        trimmed_str = strtrim(str);
        if strcmp(trimmed_str,'tbx_promptLicense(Toolbox);')
            break
        end
        line=line+1;
    end
    % Close the file
    fclose(fid);
    
    S = readlines('tbxmanager.m');
    S{line} = 'Disabled for headless run.';    %file says -1 change it to 31419
    [fid, msg] = fopen('tbxmanager.m', 'w');
    if fid < 1;
        error('could not write output file because "%s"', msg);
    end
    fwrite(fid, strjoin(S, '\n'));
    fclose(fid);
    
    tbxmanager
    savepath
    tbxmanager install mpt mptdoc cddmex fourier glpkmex hysdel lcp sedumi espresso
    
    tbxmanager restorepath
    mpt_init
    cd ..
end
end