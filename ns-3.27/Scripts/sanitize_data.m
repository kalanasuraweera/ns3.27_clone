A=table2array(readtable('11_1_(3,6,9,2)_102.csv'));

A = [A(:, 1) A(:, 2).*5];

B = A;

for i=1:length(A)
    temp = A(i, 2);
    if temp>=5.0
       B(i, 2) = 5.0 - rand(1,1) * 0.01;
    end
end

writetable(array2table(B), '11_1_(3,6,9,2)_102_sanitized.csv');