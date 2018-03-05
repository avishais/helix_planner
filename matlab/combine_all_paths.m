fileID = fopen('../paths/all_paths.txt','w');

for i = 0:17
    Q = load(['../paths/robot_path_twist_' num2str(i) '.txt']);
    
    for j = 1:size(Q,1)
        for k = 1:size(Q,2)
            fprintf(fileID,'%f ', Q(j,k));
        end
        fprintf(fileID,'\n');
    end
    
    fprintf(fileID,'\n');
end

fclose(fileID);