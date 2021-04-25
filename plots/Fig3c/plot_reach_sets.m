clc;
clear;

traj = load('trajectories.mat');

fig = figure('Color', [1,1,1]);
set(fig, 'Position', [100 100 800 600])
plot(traj.allX{1}(1,:), traj.allX{1}(2,:), 'r')
hold on;
verisig_reach();
tmp_reach();
for i = 2:size(traj.allX)
   plot(traj.allX{i}(1,:), traj.allX{i}(2,:), 'r')
end
set(gca,'fontsize',24)
ylabel('x_2', 'FontSize',30);
xlabel('x_1', 'FontSize',30);
%ylim([-1 1.5])
%title('Reachable sets for MC 2x200', 'FontSize',20);
%[lgd, hObj] = legend('Simulated Trajectories', 'TMP', 'NNV', 'Location', 'northeast');
%lgd.FontSize = 22;
%hL=findobj(hObj,'type','line');  % get the lines, not text
%set(hL,'linewidth',3)            % set their width property
grid on;

export_fig('reach_sets_verisig_mc_big.pdf', '-transparent')