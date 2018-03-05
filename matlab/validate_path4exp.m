% Validate paths with for experiments

for i = 0%:17
    Q = load(['../paths/robot_path_twist_' num2str(i) '.txt']);
    A = load(['../paths/afile_' num2str(i) '.txt']);
    
    figure(1)
    clf
    subplot(211)
    plot(rad2deg(Q),'.-');
    
    subplot(212)
    plot(A,'.-');
%     pause();
end

%%
% No solution
% 
%
% Error
% 
%
% Needs orientation correction:
% 
%
% angles out-of-bounds
%  _19

% Approved:
% _... 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17
% Ok but may change orientation:
