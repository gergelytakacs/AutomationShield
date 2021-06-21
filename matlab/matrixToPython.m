function matrixToPython(filehandle,X)
    [lengthX, widthX]=size(X);

    fprintf(filehandle, '\n');


    if widthX==1 &&  lengthX==1
    fprintf(filehandle, [inputname(2),' = [[',num2str(X),']]']);
    else if widthX==1  && lengthX~=1
        fprintf(filehandle, [inputname(2),' = [']);
        for i = 1:lengthX
            fprintf(filehandle, '[ %.9g', X(i,1));
            fprintf(filehandle, '],');
        end
        fprintf(filehandle, ']');
    else
        fprintf(filehandle, [inputname(2),' = [\n']);
        for i = 1:lengthX
            fprintf(filehandle, '\t');
            fprintf(filehandle, '[ %.9g', X(i,1));
            fprintf(filehandle, ', %.9g', X(i,2:end));
            fprintf(filehandle, '],\n');
        end
        fprintf(filehandle, '\t]');
    end

    fprintf(filehandle, '\n');
end