for i = 1:4
    T{i} = eval(['T12_set' num2str(i)]);
    
%     disp(T{i}(1:3,1:3));
end

T{5}=[-0.000998989743379000,4.49386056470000e-05,1.19777362000000e-07,1.10325686135910;-4.49386367230000e-05,-0.000998989712462000,-2.70785636000000e-07,0.0329685182279130;1.07487624000000e-07,-2.75894705000000e-07,0.000999999956164000,-0.000264376677753000;0,0,0,0.00100000000000000]*1e3;

for i = 1:5
%     v = rotm2eul(T{i}(1:3,1:3));
%     disp(v);
disp(T{i}(1:3,4)')
end