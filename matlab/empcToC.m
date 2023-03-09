function empcToC(obj, architecture)
global MPTOPTIONS

%    "MPT3" EXPLICIT MPC CONTROLLER GENERATOR FOR MICROCONTROLLERS 
% 
% 	 The function below has been adapted from the C code auto-generation
%    routine of the Multi-Parametric Toolbox 3 (MPT3) of Kvasnica et al. 
%    for  MATLAB. Please see http://www.mpt3.org for more information.
%    Please see the original "toC.m" for more details in the MPT3
% 
%    The following code has been adapted to perform microcontroller 
%    architecutre-specific changes for AVR and ARM Cortex-A devices.
% 
%    The list of changes from the original are as follows:
% 	 - Removed much of the diagnostic features.
%    - Removed the evaluation function part and Simulink port.
%    - Changed "static" modifiers to "const" so microcontrollers will
%    prefer to use ROM instead of RAM.
%    - Added PROGMEM functionality for the AVR architecture, and calling
%    the necessary headers.
%    - Removed the possibility to use PWQ, this only handles PWA.
%    - Removed matrix export for the tie-breaking function.
% 
%    This code snippet has been altered for the needs of the 
%    the AutomationShield hardware and software ecosystem. 
%    Visit http://www.automationshield.com for more details.
% 
%    Last updated by: Gergely Takacs
%    Last updated on: 11.10.2020
% 
% 
%    Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) 
%    Revised in 2012-2013 by Martin Herceg (herceg@control.ee.ethz.ch)    
%    Adapted for MCU use by Gergely Tak�cs in 2020 (gergely.takacs@stuba.sk)

AVR = 0;                        % Initialize default value
if strcmp(architecture,'AVR')
    AVR = 1;                    % Switch to enable PROGMEM functionality
    precision = 'float';        % The AVR architecture does not implement double
else
    precision = 'float';        % Most MCU cannot handle double properly anyways
end
   
obj = obj.optimizer;            % Extract optimizer from object
file_name = 'ectrl.h';          % Leave it to default, thus it remains compatible with the AutomationShield examples

% extract polyhedra with control law
Pn = [obj.Set];
nr = [obj.Num];
total_nr = sum(nr);

% extract hyperplane representation
An = cell(total_nr,1);
bn = cell(total_nr,1);
[An{:}]=deal(Pn.A);
[bn{:}]=deal(Pn.b);
if ~iscell(An),
    An = {An};
    bn = {bn};
end

% count number of constraints
nctotal = 0;
for ii=1:total_nr,
    nctotal = nctotal + size(Pn(ii).H,1);
end


% extract dimensions
nx = obj(1).Dim; % domain
nu = obj(1).Set(1).Functions('primal').R; % range

% extract PWA function
Fi = cell(total_nr,1);
Gi = Fi;
for i=1:total_nr
    Fi{i}=Pn(i).Functions('primal').F;
    Gi{i}=Pn(i).Functions('primal').g;
end



%% write mpt_getInput.c
fid = fopen(file_name, 'w');
if fid<0,
    error('Cannot open file "%s" for writing!',file_name);
end

header = {
'/*'
'    MATRICES FOR EXPLICIT MPC POLYTOPES AND CORRESPONDING PWA LAWS'
''
' 	  The automatically generated set of matrices have been adapted from '
'    the C code auto-generation routine of the Multi-Parametric Toolbox 3 '
'    MPT3) of Kvasnica et al. for  MATLAB. Please see http://www.mpt3.org '
'    for more information. Please see the original "toC.m" for more details'
'    in the MPT3.'
''
'    The following C code has been slightly adapted to perform microcontroller' 
'    architecutre-specific changes for AVR and ARM Cortex-A devices.'
''
'    The list of changes from the original are as follows:'
'    - Changed "static" modifiers to "const" so microcontrollers will'
'    prefer to use ROM instead of RAM.'
'    - Added PROGMEM functionality for the AVR architecture, and calling'
'    the necessary headers.'
'    - Removed the possibility to use PWQ, this only handles PWA.'
'    - Removed matrix export for the tie-breaking function.'
'' 
'    Usage: include alongside with the empcSequential.h header to your'
'    example.'
''
'    This code snippet has been altered for the needs of the' 
'    the AutomationShield hardware and software ecosystem. '
'    Visit http://www.automationshield.com for more details.'
''
''
'    Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) '
'    Revised in 2012-2013 by Martin Herceg (herceg@control.ee.ethz.ch)    '
'    Adapted for MCU use by Gergely Tak�cs in 2020 (gergely.takacs@stuba.sk)'
'*/'
};


% write the header
for i=1:numel(header)
    fprintf(fid,[header{i},'\n']);
end

if AVR;
    fprintf(fid, '#include <avr/pgmspace.h> \n\n');
end

fprintf(fid, '#define MPT_NR %d\n', total_nr);
fprintf(fid, '#define MPT_DOMAIN %d\n', nx);
fprintf(fid, '#define MPT_RANGE %d\n', nu);
fprintf(fid, '#define MPT_ABSTOL %e\n', MPTOPTIONS.abs_tol);


% write inequality constraints A*x <= b for each polytope
ctr = 0;
if AVR;
    fprintf(fid, '\nconst %s MPT_A[] PROGMEM = {\n', precision);
else
    fprintf(fid, '\nconst %s MPT_A[] = {\n', precision);
end

for ii = 1:total_nr,
    Ai = An{ii};
    nc = size(Ai, 1);
    for jj = 1:nc,
        a = Ai(jj, :);
        for kk = 1:length(a),
            ctr = ctr + 1;
            if ctr<nctotal*nx,
                if isequal(precision,'float')
                    fprintf(fid, '%.7e,\t', a(kk));
                else
                    fprintf(fid, '%.14e,\t', a(kk));
                end
            else
                if isequal(precision,'float')
                    fprintf(fid, '%.7e ', a(kk));
                else
                    fprintf(fid, '%.14e ', a(kk));
                end
            end
            if mod(ctr, 5)==0,
                fprintf(fid, '\n');
            end
        end
    end
end
fprintf(fid, '};\n\n');

ctr = 0;
if AVR;
    fprintf(fid, 'const %s MPT_B[] PROGMEM = s{\n',precision);
else
    fprintf(fid, 'const %s MPT_B[] = {\n',precision);
end


for ii = 1:total_nr,
    bi = bn{ii};
    nc = size(bi, 1);
    for jj = 1:nc,
        ctr = ctr + 1;
        if ctr<nctotal,
            if isequal(precision,'float')
                fprintf(fid, '%.7e,\t', bi(jj));
            else
                fprintf(fid, '%.14e,\t', bi(jj));
            end
        else
            if isequal(precision,'float')
                fprintf(fid, '%.7e ', bi(jj));
            else
                fprintf(fid, '%.14e ', bi(jj));
            end
        end
        if mod(ctr, 5)==0,
            fprintf(fid, '\n');
        end
    end
end
fprintf(fid, '};\n\n');

if AVR;
    fprintf(fid, 'const int MPT_NC[] PROGMEM = {\n');
else 
    fprintf(fid, 'const int MPT_NC[] = {\n');
end
for ii = 1:total_nr,
    if ii < total_nr,
        fprintf(fid, '%d,\t', size(Pn(ii).H,1));
    else
        fprintf(fid, '%d ', size(Pn(ii).H,1));
    end
    if mod(ii, 5)==0,
        fprintf(fid, '\n');
    end
end
fprintf(fid, '};\n\n');


% write  linear and affine terms f(x) = F*x + G
if AVR;
    sub_write_matrix(Fi, 'MPT_F', fid, precision, 'AVR'); % linear term F
    sub_write_matrix(Gi, 'MPT_G', fid, precision, 'AVR'); % affine term G
else
    sub_write_matrix(Fi, 'MPT_F', fid, precision, 'generic'); % linear term F
    sub_write_matrix(Gi, 'MPT_G', fid, precision, 'generic'); % affine term G
end

fclose(fid);
fprintf('Output written to "%s".\n', file_name);

end % Main function ends here.


function sub_write_matrix(matrices, name, fid, precision, architecture)
%
% writes a cell of matrices to a file with a given name
%
% syntax:
%         matrix - a cell array with matrix data
%         name - name of the matrix given as string
%         fid - file identificator
%         precision - either "float" or "double"

AVR = 0;
if strcmp(architecture, 'AVR')
    AVR = 1;
end
   

nr = numel(matrices);
[nu,nx] = size(matrices{1});


nctotalh = nu*nx*nr;
ctr = 0;
fprintf(fid, '\n');
if AVR
    fprintf(fid, 'const %s %s[] PROGMEM = {\n', precision, name);
else    
    fprintf(fid, 'const %s %s[] = {\n', precision, name);
end
for ii = 1:nr,
    M = matrices{ii};
    for jj = 1:nu,
        h = M(jj, :);
        for kk = 1:nx,
            ctr = ctr + 1;
            if ctr<nctotalh,
                if isequal(precision,'float')
                    fprintf(fid, '%.7e,\t', h(kk));
                else
                    fprintf(fid, '%.14e,\t', h(kk));
                end
            else
                if isequal(precision,'float')
                    fprintf(fid, '%.7e ', h(kk));
                else
                    fprintf(fid, '%.14e ', h(kk));
                end
            end
            if mod(ctr, 5)==0,
                fprintf(fid, '\n');
            end
        end
    end
end
fprintf(fid, '};\n\n');

end
