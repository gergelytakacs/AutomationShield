clc; clear;                                    % Clear screen and memory 

%% Data preprocessing
load resultID.mat                              % Read identificaiton results
Ts=0.003;                                      % Sampling
data=iddata(y,u,Ts)                            % Create identification data object

                                 
% Import   mydata                   
mydatae = data([1992:3971])       
mydataee = mydatae([136:1920])      

  
                                     
% Transfer function estimation       
 Options = tfestOptions;             
 Options.Display = 'on';             
 Options.WeightingFilter = [];       
                                     
sys = tfest(mydataee, 2, 0, Options)
 
save sys sys
 
 