function [output] = dnn(weights, biases, activations, input)
%DNN Summary of this function goes here
%   Detailed explanation goes here

curNeurons = input;
fn = fieldnames(weights);

for i = 1:length(fn)
    layerName = strcat('layer', int2str(i));
    
    curWeights = weights.(layerName);
    curBiases = biases.(layerName);
    
    curNeurons = curWeights' * curNeurons + curBiases';
    
    if strcmp(activations.(layerName), 'Sigmoid')
        curNeurons = 1 ./ (1 + exp(- curNeurons));
    
    elseif strcmp(activations.(layerName), 'Tanh')
        curNeurons = tanh(curNeurons); 
    end
end

output = curNeurons;

end

