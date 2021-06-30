function releaseManagement()

currVer='R2019a' %Currently supported Simulink version

libNames={'AutomationShield','FloatLibrary','HeatLibrary','OptoLibrary','TugLibrary','MagnetoLibrary','PressureLibrary','MotoLibrary'};
exampleList={'OptoShield','HeatShield','FloatShield','TugShield','PressureShield','MotoShield','MagnetoShield'};
versions=[];

disp('')
disp('=======================')
disp('')

disp('Library Model Versions:')
disp('-----------------------')
for i=1:length(libNames)
    info = Simulink.MDLInfo(libNames{i});
    versions{i}=info.ReleaseName;
    disp([libNames{i},': ',versions{i}])    
end
versions=[];

disp('')
disp('=======================')
disp('')




disp('Example Model Versions:')
disp('-----------------------')
cd('examples')
for i=1:length(exampleList);
    cd(exampleList{i})
    for i=1:length(listDirSLX())
        dirContents=listDirSLX();
        scr = dirContents(i).name; %Full file name with extension
        %scr = dirContents(i).name(1:end-2); %Script name to launch
        info = Simulink.MDLInfo(scr);
        versions{i}=info.ReleaseName;
        disp([scr,': ',versions{i}])  
    end
    cd .. % Switch to next directory
end
cd ..

disp('')
disp('=======================')
disp('')


if all(strcmp(versions(:),currVer))
    disp(['Current Simulink API and example version: ',currVer])
else
    error(['Error! At least one library or example is other than ',currVer])
end




if all(strcmp(versions(:),versions{1}))
    disp('All library modules and examples are saved in the same Simulink version.')
else
    disp('Warning! Not all modules and examples are saved in the same Simulink version!')
end

end


function out = listDirSLX
    persistent dirContents
    dirContents=dir('**/*.slx');
    out = dirContents;
end
