clc;
clear;

x1_range = 90:0.2:91;
x2_range = 32:0.01:32.05;
x4_range = 10:0.2:11;
x5_range = 30:0.01:30.05;

dt = 0.1;

allTraj = cell(length(x1_range) * length(x2_range) * length(x4_range) * length(x5_range));

count = 1;
for x1 = x1_range
    for x2 = x2_range
        for x4 = x4_range
            for x5 = x5_range
                allTraj{count} = sim_system([x1; x2; 0; x4; x5; 0], dt);
                count = count + 1;
            end
        end
    end
end

figure;
plot(allTraj{1}(2,:), allTraj{1}(5,:), 'r')
hold on;
for i = 2:size(allTraj)
   plot(allTraj{i}(2,:), allTraj{i}(5,:), 'r')
end

save('trajectories.mat', 'allTraj');