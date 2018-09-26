I = dlmread('COMTAC_ss_5.csv');
I_N = dlmread('COMTAC_totals_5.csv');


n_ch = 11;

alpha = 0.1:0.1:0.9;

chs = [];

for i=1:length(alpha)
    al = alpha(i);
    mn = 100000000.0;
    ch = [0, 0];
    for j=1:n_ch
        for k=1:n_ch
            if k==j
                continue;
            end
            cd1 = (1-al) * I_N(j) + al * I(j);
            cd2 = (1-al) * I_N(k) + al * I(k);
            cnd = cd1 * cd2 * get_i_factor(j, k);
            fprintf('%d %d %.4f\n', j, k, cnd);
            %fprintf('%.3f %.3f %d\n', cd1, cd2, get_i_factor(j, k));
            if(cnd < mn)
                ch(1)=j;
                ch(2)=k;
                mn=cnd;
            end
        end
    end
    %fprintf('%d\n', ch);
    %pause(1);
    %chs(end+1) = ch;
    fprintf('%d %d\n', ch(1), ch(2));
    %pause(1);
end

% fprintf('%d\n', mode(chs));
% f1 = mode(chs);

function i_f = get_i_factor(a, b)
    if abs(a-b)>=5
        i_f=0.1;
        return;
    end
    i_factors = [0.0, 0.22, 0.60, 0.72, 0.77, 1.0, 0.96, 0.77, 0.66, 0.39, 0.0];
    i_f=i_factors(6+b-a);
end