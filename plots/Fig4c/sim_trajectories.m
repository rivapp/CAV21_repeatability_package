clc;
clear;

x1_range = -0.77:0.02:-0.75;
x2_range = -0.45:0.02:-0.43;
%x3_range = 0.51:0.02:0.54;
x3_range = [0.51];
%x4_range = -0.3:0.03:-0.28;
x4_range = [-0.3];

dt = 0.5;

allTraj = cell(length(x1_range) * length(x2_range) * length(x3_range) * length(x4_range));

count = 1;
for x1 = x1_range
    for x2 = x2_range
        for x3 = x3_range
            for x4 = x4_range
                allTraj{count} = sim_system([x1; x2; x3; x4], dt);
                count = count + 1;
            end
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