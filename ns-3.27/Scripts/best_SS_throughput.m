fileID = fopen('throughput_best_SS_2.txt','a');
fileID1 = fopen('channel_best_SS_2.txt','a');

A = dlmread('throughput_SS_2.txt');

for i=1:50
    mx=0.0;
    ch = [];
    for j=1:11
        thr = A((i-1)*11+j, 3);
        if thr > mx
            mx = thr;
            ch = j;
        end
    end
    fprintf(fileID, '%d %f\n', i, thr);
    fprintf(fileID1, '%d %d\n', i, ch);
end