function [allX] = sim_system(x0, dt)
%SIM_MC Summary of this function goes here
%   Detailed explanation goes here

x = x0;

weights = load('weights.mat');
biases = load('biases.mat');
activations = load('activations.mat');

num_states = 2;

num_steps = 9;

%allX = zeros(num_states, num_steps + 1);
allX = [];
allX(:,1) = x;

for step = 1:num_steps

    dnn_input = [x(1); x(2)];
 
    control = dnn(weights, biases, activations, dnn_input);
    
    t_span = [0, dt];
    [t, y] = ode45(@(t,y) dynamics(t, y, control), t_span, x);

    x = y(end,:);
    
    allX = [allX, y'];
        
end

%allX = allX(:,1:step);

end

