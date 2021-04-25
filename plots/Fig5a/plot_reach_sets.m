clc;
clear;

nnv_reach = load('nnv_flowpipes.mat');

traj = load('trajectories.mat');

target_x0 = [22.81, 22.87];
target_x1 = [29.88, 30.02];

goal_x = [target_x0(1), target_x0(1), target_x0(2), target_x0(2), target_x0(1)];
goal_y = [target_x1(1), target_x1(2), target_x1(2), target_x1(1), target_x1(1)];

fig = figure('Color', [1,1,1]);
set(fig, 'Position', [100 100 800 600])
map_mat = [0 1 0 0 0 0; 0 0 0 0 1 0];
map_vec = [];
plot(traj.allTraj{1}(2,:), traj.allTraj{1}(5,:), 'r')
hold on;

nnv_reach.ncs.plotOutputReachSets('blue', map_mat, map_vec);
acc_tanh_tmp

for i = 1:size(traj.allTraj)
   plot(traj.allTraj{i}(2,:), traj.allTraj{i}(5,:), 'r')
end
plot(goal_x, goal_y, 'magenta', 'linewidth', 2);
set(gca,'fontsize',24)
ylabel('x_4', 'FontSize',30);
xlabel('x_1', 'FontSize',30);
xlim([22.5 32.1])
grid on;

export_fig('reach_sets_nnv_acc.pdf', '-transparent')