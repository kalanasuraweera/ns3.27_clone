alpha = 0:0.1:1;

for k=1:50

    I = dlmread(sprintf('COMTAC_ss_%d.csv', k));
    I_N = dlmread(sprintf('COMTAC_totals_%d.csv', k));

    chs = [];

    for i=1:length(alpha)
        al = alpha(i);
        mn = 100000000.0;
        ch = [];
        for j=1:length(I)
            cd = (1-al) * I_N(j) + al * I(j);
            if(cd < mn)
                ch=j;
                mn=cd;
            end
        end
        %fprintf('%d\n', ch);
        %pause(1);
        chs(end+1) = ch;
    end

    %fprintf('%d\n', mode(chs));
    f1 = mode(chs);

    chs_1 = [];

    for i=1:length(alpha)
        al = alpha(i);
        mn = 100000000.0;
        ch = [];
        for j=1:length(I)
            if j==f1
                continue;
            end
            cd = (1-al) * I_N(j) + al * I(j);
            cnd = cd * get_i_factor(f1, j);
            if(cnd < mn)
                ch=j;
                mn=cnd;
            end
        end
        %fprintf('%d\n', ch);
        %pause(1);
        chs_1(end+1) = ch;
    end

    fprintf('%d %d\n', mode(chs), mode(chs_1));

end

function i_f = get_i_factor(a, b)
    if abs(a-b)>=5
        i_f=0;
        return;
    end
    i_factors = [0.0, 0.22, 0.60, 0.72, 0.77, 1.0, 0.96, 0.77, 0.66, 0.39, 0.0];
    i_f=i_factors(6+b-a);
end