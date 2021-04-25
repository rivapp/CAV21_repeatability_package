clc;
clear;

x1_range = -0.53:0.005:-0.5;
x2_range = [0];
num_states = 2;

weights = load('weights.mat');
biases = load('biases.mat');
activations = load('activations.mat');

num_steps = 150;

allX = cell(length(x1_range) * length(x2_range));

count = 1;

for x1 = x1_range
    for x2 = x2_range
        
        x = [x1; x2];
        
        cur_trace = zeros(num_states, num_steps + 1);
        cur_trace(:,1) = x;

        for step = 1:num_steps
            u = dnn(weights, biases, activations, x);

            [x, done] = dynamics(x, u);

            cur_trace(:,step + 1) = x;

            if done
                cur_trace = cur_trace(:, 1:step+1);
                break
            end

        end
        
        allX{count} = cur_trace;
        count = count + 1;
    end
end

figure;
for i = 1:count-1
    cur_trace = allX{i};
    plot(cur_trace(1,:), cur_trace(2,:), 'r');
    hold on
end

save('trajectories.mat', 'allX');