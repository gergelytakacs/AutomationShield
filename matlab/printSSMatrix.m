function printSSMatrix(SSmatrix, name)
matrixSize = size(SSmatrix);
vector = SSmatrix';
vector = vector(:)';
vector = round(vector, 5);
fprintf('BLA::Matrix<%d, %d> %s%s', matrixSize(1), matrixSize(2), name, ' = {');
for i = 1:length(vector)
    fprintf('%.5g', vector(i));
    if i < length(vector)
        fprintf('%s', ', ');
    end
end
fprintf('%s', '};');
fprintf('\n');
end
