function [allX] = sim_system(x0, dt)
%SIM_MC Summary of this function goes here
%   Detailed explanation goes here

x = x0;

weights = load('weights.mat');
biases = load('biases.mat');
activations = load('activations.mat');

num_steps = 50;

allX = zeros(6, num_steps + 1);
allX(:,1) = x;

V_set = 30;
T_gap = 1.4;

for step = 1:num_steps

    dnn_input = [V_set; T_gap; x(5); x(1) - x(4); x(2) - x(5)];
 
    control = dnn(weights, biases, activations, dnn_input);
    
    t_span = [0, dt];
    [t, y] = ode45(@(t,y) dynamics(t, y, control), t_span, x);

    x = y(end,:);
    
    allX(:,step + 1) = x;
        
end

%allX = allX(:,1:step);

end

