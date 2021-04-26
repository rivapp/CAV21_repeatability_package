clc;
clear;

traj = load('trajectories.mat');

target_x0 = [-0.38, -0.43];
target_x1 = [0.16, 0.18];

goal_x = [target_x0(1), target_x0(1), target_x0(2), target_x0(2), target_x0(1)];
goal_y = [target_x1(1), target_x1(2), target_x1(2), target_x1(1), target_x1(1)];

fig = figure('Color', [1,1,1]);
set(fig, 'Position', [100 100 800 600])
plot(traj.allTraj{1}(1,:), traj.allTraj{1}(2,:), 'r')
hold on;

nn_5_tanh;
ex5_tanh_tmp;

for i = 2:size(traj.allTraj)
   plot(traj.allTraj{i}(1,:), traj.allTraj{i}(2,:), 'r')
end
plot(goal_x, goal_y, 'magenta', 'linewidth', 2);
set(gca,'fontsize',24)
ylabel('x_2', 'FontSize',30);
xlabel('x_1', 'FontSize',30);
grid on;

export_fig('reach_sets_reachnn_ex5_tanh.pdf', '-transparent')