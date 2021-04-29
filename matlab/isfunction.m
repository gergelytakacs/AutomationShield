% From: https://www.mathworks.com/matlabcentral/answers/91508-is-there-a-native-function-in-matlab-that-can-determine-if-a-matlab-file-or-p-file-is-a-function-or
function result = isfunction(fname)
% Open the file passed as a string to the function
fid = fopen(fname,'rt');
% Make sure the file opened correctly
if (fid < 1)
    error(ferror(fid))
end
result = false;
% Count the number of lines
while ~feof(fid) && ~result
    str = fgetl(fid)
    trimmed_str = strtrim(str);
    if ~isempty(str)
        if ~isempty(trimmed_str) && ~strcmp(trimmed_str(1), '%')
            try
                result = strcmp(trimmed_str(1:8), 'function');
            catch
                %first line of code is not a function
                break
            end
        end
    end
end
% Close the file
fclose(fid);