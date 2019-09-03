%    Simulink implementation of Pololu arduino library for
%    VL53L0X Time-of-Flight distance sensor.
%
%    This code is part of the AutomationShield hardware and software
%    ecosystem. Visit http://www.automationshield.com for more
%    details. This code is licensed under a Creative Commons
%    Attribution-NonCommercial 4.0 International License.

%    Created by Peter Chmurciak using S-Function Builder.
%    Last update: 23.7.2019.

function makeInfo=rtwmakecfg()
makeInfo.includePath = {};
makeInfo.sourcePath  = {};
makeInfo.linkLibsObjs = {};

sfBuilderBlocksByMaskType = find_system(bdroot,'FollowLinks','on','LookUnderMasks','on','MaskType','S-Function Builder');
sfBuilderBlocksByCallback = find_system(bdroot,'OpenFcn','sfunctionwizard(gcbh)');
sfBuilderBlocksDeployed   = find_system(bdroot,'BlockType','S-Function','SFunctionDeploymentMode','on');
sfBuilderBlocks = {sfBuilderBlocksByMaskType{:} sfBuilderBlocksByCallback{:} sfBuilderBlocksDeployed{:}};
sfBuilderBlocks = unique(sfBuilderBlocks);
if isempty(sfBuilderBlocks)
   return;
end
sfBuilderBlockNameMATFile = cell(1, length(sfBuilderBlocks));
for idx = 1:length(sfBuilderBlocks)
   sfBuilderBlockNameMATFile{idx} = get_param(sfBuilderBlocks{idx},'FunctionName');
   sfBuilderBlockNameMATFile{idx} = ['.' filesep 'SFB__' char(sfBuilderBlockNameMATFile{idx}) '__SFB.mat'];
end
sfBuilderBlockNameMATFile = unique(sfBuilderBlockNameMATFile);
for idx = 1:length(sfBuilderBlockNameMATFile)
   if exist(sfBuilderBlockNameMATFile{idx}, 'file')
      loadedData = load(sfBuilderBlockNameMATFile{idx});
      if isfield(loadedData,'SFBInfoStruct')
         makeInfo = UpdateMakeInfo(makeInfo,loadedData.SFBInfoStruct);
         clear loadedData;
      end
   end
end

function updatedMakeInfo = UpdateMakeInfo(makeInfo,SFBInfoStruct)
updatedMakeInfo = {};
if isfield(makeInfo,'includePath')
   if isfield(SFBInfoStruct,'includePath')
      updatedMakeInfo.includePath = {makeInfo.includePath{:} SFBInfoStruct.includePath{:}};
   else
      updatedMakeInfo.includePath = {makeInfo.includePath{:}};
   end
end
if isfield(makeInfo,'sourcePath')
   if isfield(SFBInfoStruct,'sourcePath')
      updatedMakeInfo.sourcePath = {makeInfo.sourcePath{:} SFBInfoStruct.sourcePath{:}};
   else
      updatedMakeInfo.sourcePath = {makeInfo.sourcePath{:}};
   end
end
if isfield(makeInfo,'linkLibsObjs')
   if isfield(SFBInfoStruct,'additionalLibraries')
      updatedMakeInfo.linkLibsObjs = {makeInfo.linkLibsObjs{:} SFBInfoStruct.additionalLibraries{:}};
   else
      updatedMakeInfo.linkLibsObjs = {makeInfo.linkLibsObjs{:}};
   end
end