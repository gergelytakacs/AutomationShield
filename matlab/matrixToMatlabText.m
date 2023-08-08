function matrixToMatlabText(filehandle,X)
    [lengthX, widthX]=size(X);

    fprintf(filehandle, '\n');


    if widthX==1 &&  lengthX==1
    fprintf(filehandle, [inputname(2),' = [',num2str(X),']']);
    else if widthX==1  && lengthX~=1
        fprintf(filehandle, [inputname(2),' = [']);
        for i = 1:lengthX-1
            fprintf(filehandle, '%.9g; ', X(i,1));
        end
        fprintf(filehandle, '%.9g] ', X(end,1));
    else
        fprintf(filehandle, [inputname(2),' = [']);
        for i = 1:lengthX-1
            %fprintf(filehandle, '%.9g', X(i,1));
            fprintf(filehandle, '%.9g, ', X(i,1:end-1));
            fprintf(filehandle, '%.9g; ', X(i,end));
        end
        fprintf(filehandle, '%.9g, ', X(i,1:end-1));
        fprintf(filehandle, '%.9g]\n ', X(i,end));
    end

    fprintf(filehandle, '\n');
    end