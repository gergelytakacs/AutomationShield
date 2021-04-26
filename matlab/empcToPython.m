function empcToPython(obj)
global MPTOPTIONS

%    "MPT3" EXPLICIT MPC CONTROLLER GENERATOR FOR PYTHON
% 
% 	 The function below has been adapted from the C code auto-generation
%    routine of the Multi-Parametric Toolbox 3 (MPT3) of Kvasnica et al. 
%    for  MATLAB. Please see http://www.mpt3.org for more information.
%    Please see the original "toC.m" for more details in the MPT3
% 
%    The following code has been adapted to write the region and PWA law
%    matrices to a Python module.
% 
%    The list of changes from the original are as follows:
% 	 - Removed much of the diagnostic features.
%    - Removed the evaluation function part and Simulink port.
%    - Removed the possibility to use PWQ, this only handles PWA.
%    - Removed matrix export for the tie-breaking function.
% 
%    This code snippet has been altered for the needs of the 
%    the AutomationShield hardware and software ecosystem. 
%    Visit http://www.automationshield.com for more details.
% 
%    Last updated by: Gergely Takacs
%    Last updated on: 11.11.2020
% 
%    Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) 
%    Revised in 2012-2013 by Martin Herceg (herceg@control.ee.ethz.ch)    
%    Adapted for MCU use by Gergely Takács in 2020 (gergely.takacs@stuba.sk)

 
obj = obj.optimizer;            % Extract optimizer from object
file_name = 'ectrl.py';          % Leave it to default, thus it remains compatible with the AutomationShield examples

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
'"""'
'    MATRICES FOR EXPLICIT MPC POLYTOPES AND CORRESPONDING PWA LAWS'
''
' 	 The automatically generated set of matrices have been adapted from '
'    the C code auto-generation routine of the Multi-Parametric Toolbox 3 '
'    MPT3) of Kvasnica et al. for  MATLAB. Please see http://www.mpt3.org '
'    for more information. Please see the original "toC.m" for more details'
'    in the MPT3.'
''
'    The following Python script has been rewritten from it C language'
'    counterpart.'
''    
'    The list of changes from the original are as follows:'
'    - Removed the possibility to use PWQ, this only handles PWA.'
'    - Removed matrix export for the tie-breaking function.'
'' 
'    Usage: this Python module is imported by the empc module'
'    implementing the sequential search algorithm.'
''
'  There are several limitations to this code. CircuitPython compiles'
'  the *.py files at runtime and in RAM. Large problems stored in'
'  ectrl.py cannot be evaluated because the file size itself is too'
'  large. Moreover, the Python evaluation of the search function is'
'  much slower than the C implementation. There are a couple of'
'  possible but untried workarounds: (a) storing the lists as frozen'
'  bytecode in the firmware, and (b) using uLab functionality instead'
'  of lists and C-like element-by-element operations.'
''
'  If you have found any use of this code, please cite our work in your'
'  academic publications, such as thesis, conference articles or journal'
'  papers. A list of publications connected to the AutomationShield'
'  project is available at:'
'  https://github.com/gergelytakacs/AutomationShield/wiki/Publications'
'
'  This code is part of the AutomationShield hardware and software'
'  ecosystem. Visit http://www.automationshield.com for more'
'  details. This code is licensed under a Creative Commons'
'  Attribution-NonCommercial 4.0 International License.'
''
''
'    Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) '
'    Revised in 2012-2013 by Martin Herceg (herceg@control.ee.ethz.ch)    '
'    Adapted for to Python by Gergely Takács in 2020 (gergely.takacs@stuba.sk)'
'"""'
''
};


% write the header
for i=1:numel(header)
    fprintf(fid,[header{i},'\n']);
end

fprintf(fid, 'MPT_NR = %d\n', total_nr);
fprintf(fid, 'MPT_DOMAIN = %d\n', nx);
fprintf(fid, 'MPT_RANGE = %d\n', nu);
fprintf(fid, 'MPT_ABSTOL = %e\n', MPTOPTIONS.abs_tol);


% write inequality constraints A*x <= b for each polytope
ctr = 0;
fprintf(fid, '\nMPT_A = [\n');

for ii = 1:total_nr,
    Ai = An{ii};
    nc = size(Ai, 1);
    for jj = 1:nc,
        a = Ai(jj, :);
        for kk = 1:length(a),
            ctr = ctr + 1;
            if ctr<nctotal*nx,
                    fprintf(fid, '%.14e,\t', a(kk));
            else
                    fprintf(fid, '%.14e ', a(kk));
            end
            if mod(ctr, 5)==0,
                fprintf(fid, '\n');
            end
        end
    end
end
fprintf(fid, ']\n\n');

ctr = 0;

fprintf(fid, 'MPT_B = [\n');

for ii = 1:total_nr,
    bi = bn{ii};
    nc = size(bi, 1);
    for jj = 1:nc,
        ctr = ctr + 1;
        if ctr<nctotal,
                fprintf(fid, '%.14e,\t', bi(jj));
        else
                fprintf(fid, '%.14e ', bi(jj));
        end
        if mod(ctr, 5)==0,
            fprintf(fid, '\n');
        end
    end
end
fprintf(fid, ']\n\n');

fprintf(fid, 'MPT_NC = [\n');

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
fprintf(fid, ']\n\n');


% write  linear and affine terms f(x) = F*x + G
sub_write_matrix(Fi, 'MPT_F', fid); % linear term F
sub_write_matrix(Gi, 'MPT_G', fid); % affine term G


fclose(fid);
fprintf('Output written to "%s".\n', file_name);

end % Main function ends here.


function sub_write_matrix(matrices, name, fid)
%
% writes a cell of matrices to a file with a given name
%
% syntax:
%         matrix - a cell array with matrix data
%         name - name of the matrix given as string
%         fid - file identificator

nr = numel(matrices);
[nu,nx] = size(matrices{1});

nctotalh = nu*nx*nr;
ctr = 0;
fprintf(fid, '\n');
fprintf(fid, '%s = [\n', name);
for ii = 1:nr,
    M = matrices{ii};
    for jj = 1:nu,
        h = M(jj, :);
        for kk = 1:nx,
            ctr = ctr + 1;
            if ctr<nctotalh,
                    fprintf(fid, '%.14e,\t', h(kk));
            else
                    fprintf(fid, '%.14e ', h(kk));
            end
            if mod(ctr, 5)==0,
                fprintf(fid, '\n');
            end
        end
    end
end
fprintf(fid, ']\n\n');

end
