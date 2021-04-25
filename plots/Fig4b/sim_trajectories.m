clc;
clear;

x1_range = 0.38:0.01:0.4;
x2_range = 0.45:0.01:0.47;
x3_range = 0.25:0.01:0.27;

dt = 0.2;

allTraj = cell(length(x1_range) * length(x2_range));

count = 1;
for x1 = x1_range
    for x2 = x2_range
        for x3 = x3_range
            allTraj{count} = sim_system([x1; x2; x3], dt);
            count = count + 1;
        end
    end
end

figure;
plot(allTraj{1}(1,:), allTraj{1}(2,:), 'r')
hold on;
for i = 2:size(allTraj)
   plot(allTraj{i}(1,:), allTraj{i}(2,:), 'r')
end

save('trajectories.mat', 'allTraj');