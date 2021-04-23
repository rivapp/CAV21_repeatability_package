#!/usr/bin/python3

import os
import subprocess
import multiprocessing

verisig_path = '../../verisig'
flowstar_path = '../../flowstar/flowstar'
output_path = 'output'
dnn_yaml = 'tanh20x20.yml'

if not os.path.exists(output_path):
        os.mkdir(output_path)


legend = ['X1_LOWER', 'X1_UPPER', 'X2_LOWER', 'X2_UPPER']

test_set = [
    [-0.05, -0.025, -0.05, -0.025],
    [-0.025, 0, -0.05, -0.025],
    [0, 0.025, -0.05, -0.025],
    [0.025, 0.05, -0.05, -0.025],
    [-0.05, -0.025, -0.025, 0],
    [-0.025, 0, -0.025, 0],
    [0, 0.025, -0.025, 0],
    [0.025, 0.05, -0.025, 0],
    [-0.05, -0.025, 0, 0.025],
    [-0.025, 0, 0, 0.025],
    [0, 0.025, 0, 0.025],
    [0.025, 0.05, 0, 0.025],
    [-0.05, -0.025, 0.025, 0.05],
    [-0.025, 0, 0.025, 0.05],
    [0, 0.025, 0.025, 0.05],
    [0.025, 0.05, 0.025, 0.05],
]

print("Building the base model...")
subprocess.run([verisig_path, '-vc=quadrotor_MPC_multi.yml', '-o' ,'-nf', 'quadrotor_MPC.xml', dnn_yaml])

with open('quadrotor_MPC.model', 'r') as f:
    model = f.read()


#===========================================================================================
# Begin Parallel Function
#===========================================================================================
def evaluate_conditions(conditions):
    test_model = model
    for i in range(len(legend)):
        test_model = test_model.replace(legend[i], str(conditions[i]))

    with open(output_path + '/quadrotor_' + str(conditions[0]) + '_' + str(conditions[2]) + '.txt', 'w') as f:
        subprocess.run(flowstar_path + ' ' + dnn_yaml , input=test_model, shell=True, universal_newlines=True, stdout=f)
#===========================================================================================
# End Parallel Function
#===========================================================================================

print("Starting parallel verification")
num_parallel = multiprocessing.cpu_count() // 2
with multiprocessing.Pool(processes=num_parallel) as pool:
    pool.map(evaluate_conditions, test_set)
