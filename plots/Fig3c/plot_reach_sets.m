clc;
clear;

traj = load('trajectories.mat');

fig = figure('Color', [1,1,1]);
set(fig, 'Position', [100 100 800 600])
plot(traj.allX{1}(1,:), traj.allX{1}(2,:), 'r')
hold on;

MC_large_sig_verisig;
MC_large_sig_tmp;

for i = 2:size(traj.allX)
   plot(traj.allX{i}(1,:), traj.allX{i}(2,:), 'r')
end
set(gca,'fontsize',24)
ylabel('x_2', 'FontSize',30);
xlabel('x_1', 'FontSize',30);
grid on;

export_fig('reach_sets_verisig_mc_large.pdf', '-transparent')