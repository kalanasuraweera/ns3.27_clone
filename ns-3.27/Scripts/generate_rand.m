A=[table2array(readtable('1_6_(3,6,9,2)_100_sanitized.csv'))];
B=[table2array(readtable('5_11_(3,6,9,2)_101_sanitized.csv'))];
C=[table2array(readtable('11_1_(3,6,9,2)_102_sanitized.csv'))];
D=[table2array(readtable('1_10_(3,6,9,2)_103_sanitized.csv'))];

eps=0.1;

t1 = A(end, 1):0.2:A(end, 1)+10-0.2;
th1 = zeros(1, 50);

for i=1:50
    e = rand(1,1);
    if e>eps
        a=5.0 - 0.1 * (e-eps);
        %fprintf('1 %f\n', a);
    else
        a=4.9 - rand(1,1) * 0.1;
        %fprintf('2 %f\n', a);
    end
    th1(i)=a;
end

A1=cat(1, A(:, 1), t1');
A2=cat(1, A(:, 2), th1');

A= [cat(1, A(:, 1), t1') cat(1, A(:, 2), th1')];

b = max(find(B(:,1) < 36));
c = max(find(C(:,1) < 33));
d = max(find(D(:,1) < 33));

B2 = B(1:b, 2);
C2 = C(1:c, 2);
D2 = D(1:d, 2);

B1 = B(1:b, 1) + A(end,1);
C1 = C(1:c, 1) + B1(end);
D1 = D(1:d, 1) + C1(end);

r1 = cat(1, A(:, 1), B1, C1, D1);
r2 = cat(1, A(:, 2), B2, C2, D2);

r1 = r1(6:end, :) - 2.0;
r2 = r2(6:end, :);

A = [r1 r2];

plot(A(:,1), A(:,2));
%disp(A(1:10,2));

writetable(array2table(A), 'ICALO_resilience.csv');