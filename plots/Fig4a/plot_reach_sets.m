clc;
clear;

traj = load('trajectories.mat');

target_x0 = [0, 0.2];
target_x1 = [0.3, 0.05];

goal_x = [target_x0(1), target_x0(1), target_x0(2), target_x0(2), target_x0(1)];
goal_y = [target_x1(1), target_x1(2), target_x1(2), target_x1(1), target_x1(1)];

fig = figure('Color', [1,1,1]);
set(fig, 'Position', [100 100 800 600])
plot(traj.allTraj{1}(1,:), traj.allTraj{1}(2,:), 'r')
hold on;

nn_1_sigmoid;
ex1_sig_tmp;

for i = 1:size(traj.allTraj)
   plot(traj.allTraj{i}(1,:), traj.allTraj{i}(2,:), 'r')
end
plot(goal_x, goal_y, 'magenta', 'linewidth', 2);
set(gca,'fontsize',24)
ylabel('x_2', 'FontSize',30);
xlabel('x_1', 'FontSize',30);
%ylim([-1 1.5])
%title('Reachable sets for B_1 sig', 'FontSize',20);
%[lgd, hObj] = legend('Simulated Trajectories', 'TMP', 'NNV', 'Location', 'northeast');
%lgd.FontSize = 22;
%hL=findobj(hObj,'type','line');  % get the lines, not text
%set(hL,'linewidth',3)            % set their width property
grid on;

export_fig('reach_sets_reachnn_ex1_sig.pdf', '-transparent')