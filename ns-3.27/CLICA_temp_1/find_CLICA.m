fileID = fopen('ext_channels_CLICA_2.txt','a');

for i=1:50
    M = dlmread(sprintf('CLICA_%d.csv', i));
    l = length(dlmread(sprintf('CLICA_%d.csv', i)));
    mn = 100000000.0;
    ch = [0, 0];
    for j=1:l
        if M(j, 3) < mn
            mn = M(j, 3);
            ch(1) = M(j, 1);
            ch(2) = M(j, 2);
        end
    end
    fprintf('%d %d\n', ch(1), ch(2));
    fprintf(fileID, '{%d, %d},\n', ch(1), ch(2));
end

fclose(fileID);