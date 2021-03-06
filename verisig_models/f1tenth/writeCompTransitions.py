import numpy as np
from six.moves import cPickle as pickle

LIDAR_MAX_DISTANCE = 5 # in m
LIDAR_RANGE = 115 # in degrees
LIDAR_OFFSET = 11.5 # in degrees
NUM_RAYS = int(round((2 *  LIDAR_RANGE) / LIDAR_OFFSET))  + 1

print(NUM_RAYS)

PIBY180 = np.pi / 180.0
INPUT_CONST = 15
MAX_TURNING_INPUT = 15 # in degrees // NB: This is not used currently since the last layer is tanh (guaranteed to be less than INPUT_CONST)

NUM_MODES_REG1 = 12
NUM_MODES_REG2 = 15
NUM_MODES_REG3 = 15

mode1_reg1 = 12
modeL_reg1 = mode1_reg1 + NUM_RAYS * NUM_MODES_REG1 - 1 # NUM_MODES_REG1 modes per ray in region 1

mode1_reg2 = mode1_reg1 + NUM_RAYS * NUM_MODES_REG1     # NUM_MODES_REG1 modes per ray in region 1
modeL_reg2 = mode1_reg2 + NUM_RAYS * NUM_MODES_REG2 - 1 # NUM_MODES_REG2 modes per ray in region 2

mode1_reg3 = mode1_reg2 + NUM_RAYS * NUM_MODES_REG2     # NUM_MODES_REG2 modes per ray in region 2
modeL_reg3 = mode1_reg3 + NUM_RAYS * NUM_MODES_REG3 - 1 # NUM_MODES_REG3 modes per ray in region 3

trans = {}
trans['dnn2plant'] = {}
trans['dnn2plant'][1] = {}
trans['dnn2plant'][1]['guards1'] = ['clock = 0']
trans['dnn2plant'][1]['reset1'] = ['clock\' := 0', 'u\' := ' + str(INPUT_CONST * PIBY180) + ' * _f1']

#the normalization below is performed here as it was harder to do in the dynamics model
trans['plant2dnn'] = {}
trans['plant2dnn'][modeL_reg1] = {}
trans['plant2dnn'][modeL_reg1]['guards1'] = ['clock = 0', '_f' + str(NUM_RAYS) + ' <= ' + str(LIDAR_MAX_DISTANCE)]
trans['plant2dnn'][modeL_reg1]['reset1'] = ['clock\' := 0']
trans['plant2dnn'][modeL_reg1]['guards2'] = ['clock = 0', '_f' + str(NUM_RAYS) + ' >= ' + str(LIDAR_MAX_DISTANCE)]
trans['plant2dnn'][modeL_reg1]['reset2'] = ['clock\' := 0', '_f' + str(NUM_RAYS) + '\' := ' + str(0.5)]

trans['plant2dnn'][modeL_reg2] = {}
trans['plant2dnn'][modeL_reg2]['guards1'] = ['clock = 0', '_f' + str(NUM_RAYS) + ' <= ' + str(LIDAR_MAX_DISTANCE)]
trans['plant2dnn'][modeL_reg2]['reset1'] = ['clock\' := 0']
trans['plant2dnn'][modeL_reg2]['guards2'] = ['clock = 0', '_f' + str(NUM_RAYS) + ' >= ' + str(LIDAR_MAX_DISTANCE)]
trans['plant2dnn'][modeL_reg2]['reset2'] = ['clock\' := 0', '_f' + str(NUM_RAYS) + '\' := ' + str(0.5)]

trans['plant2dnn'][modeL_reg3] = {}
trans['plant2dnn'][modeL_reg3]['guards1'] = ['clock = 0', '_f' + str(NUM_RAYS) + ' <= ' + str(LIDAR_MAX_DISTANCE)]
trans['plant2dnn'][modeL_reg3]['reset1'] = ['clock\' := 0']
trans['plant2dnn'][modeL_reg3]['guards2'] = ['clock = 0', '_f' + str(NUM_RAYS) + ' >= ' + str(LIDAR_MAX_DISTANCE)]
trans['plant2dnn'][modeL_reg3]['reset2'] = ['clock\' := 0', '_f' + str(NUM_RAYS) + '\' := ' + str(0.5)]

# normalize rays
for i in range(NUM_RAYS):
    trans['plant2dnn'][modeL_reg1]['reset1'].append('_f' + str(i + 1) + '\' := (_f' + str(i + 1) + ' - 2.5) * 0.2')
    trans['plant2dnn'][modeL_reg2]['reset1'].append('_f' + str(i + 1) + '\' := (_f' + str(i + 1) + ' - 2.5) * 0.2')
    trans['plant2dnn'][modeL_reg3]['reset1'].append('_f' + str(i + 1) + '\' := (_f' + str(i + 1) + ' - 2.5) * 0.2')

    # don't reset the last f since it was already reset in this transition
    if i == NUM_RAYS - 1:
        continue
    
    trans['plant2dnn'][modeL_reg1]['reset2'].append('_f' + str(i + 1) + '\' := (_f' + str(i + 1) + ' - 2.5) * 0.2')
    trans['plant2dnn'][modeL_reg2]['reset2'].append('_f' + str(i + 1) + '\' := (_f' + str(i + 1) + ' - 2.5) * 0.2')
    trans['plant2dnn'][modeL_reg3]['reset2'].append('_f' + str(i + 1) + '\' := (_f' + str(i + 1) + ' - 2.5) * 0.2')

filename = 'glue_' + str(NUM_RAYS) + '.pickle'

try:
    with open(filename, 'wb') as f:
        pickle.dump(trans, f, pickle.HIGHEST_PROTOCOL)
except Exception as e:
    print('Unable to save data to', filename, ':', e)
