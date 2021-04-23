from six.moves import cPickle as pickle
import os, sys
import time
import subprocess
import yaml
import numpy as np

HALLWAY_WIDTH = 1.5
HALLWAY_LENGTH = 20
WALL_LIMIT = 0.15
WALL_MIN = str(WALL_LIMIT)
WALL_MAX = str(1.5 - WALL_LIMIT)
SPEED_EPSILON = 1e-8

TURN_ANGLE = -np.pi/2
#TURN_ANGLE = -2 * np.pi / 3

CORNER_ANGLE = np.pi - np.abs(TURN_ANGLE)
SIN_CORNER = np.sin(CORNER_ANGLE)
COS_CORNER = np.cos(CORNER_ANGLE)

NUM_STEPS = 50

name = 'sharp_turn_'

POS_LB = 0.65
POS_UB = 0.75
HEADING_LB = -0.005
HEADING_UB = 0.005

# just a check to avoid numerical error
if TURN_ANGLE == -np.pi/2:
    name = 'right_turn_'
    SIN_CORNER = 1
    COS_CORNER = 0

NORMAL_TO_TOP_WALL = [SIN_CORNER, -COS_CORNER]

def getCornerDist(next_heading=np.pi/2 + TURN_ANGLE, reverse_cur_heading=-np.pi/2,\
                  hallLength=HALLWAY_LENGTH, hallWidth=HALLWAY_WIDTH, turnAngle=TURN_ANGLE):

    outer_x = -hallWidth/2.0
    outer_y = hallLength/2.0
    
    out_wall_proj_length = np.abs(hallWidth / np.sin(turnAngle))
    proj_point_x = outer_x + np.cos(next_heading) * out_wall_proj_length
    proj_point_y = outer_y + np.sin(next_heading) * out_wall_proj_length
    
    in_wall_proj_length = np.abs(hallWidth / np.sin(turnAngle))
    inner_x = proj_point_x + np.cos(reverse_cur_heading) * in_wall_proj_length
    inner_y = proj_point_y + np.sin(reverse_cur_heading) * in_wall_proj_length

    corner_dist = np.sqrt((outer_x - inner_x) ** 2 + (outer_y - inner_y) ** 2)
    wall_dist = np.sqrt(corner_dist ** 2 - hallWidth ** 2)

    return wall_dist

def writeDnnModes(stream, weights, offsets, activations, dynamics, states):

    numStates = getNumStates(offsets)
    numLayers = len(offsets)
    
    #first mode
    writeOneMode(stream, 0, dynamics, states)

    #DNN mode
    writeOneMode(stream, 1, dynamics, states, 'DNN')
    
def writeOneMode(stream, modeIndex, dynamics, states, name = ''):
    stream.write('\t\t' + name + 'm' + str(modeIndex) + '\n')
    stream.write('\t\t{\n')
    stream.write('\t\t\tnonpoly ode\n')
    stream.write('\t\t\t{\n')

    for sysState in states:
        
        stream.write('\t\t\t\t' + sysState +'\' = 0\n')
        
    stream.write('\t\t\t\tclock\' = 1\n')
    stream.write('\t\t\t}\n')

    stream.write('\t\t\tinv\n')
    stream.write('\t\t\t{\n')

    stream.write('\t\t\t\tclock <= 0\n')

    stream.write('\t\t\t}\n')            
    stream.write('\t\t}\n')

def writePlantModes(stream, plant, allPlantStates, numNeurLayers):

    for modeId in plant:

        modeName = ''
        if 'name' in plant[modeId] and len(plant[modeId]['name']) > 0:
            modeName = plant[modeId]['name']
        
        stream.write('\t\t' + modeName + 'm' + str(numNeurLayers + modeId) + '\n')
        stream.write('\t\t{\n')
        stream.write('\t\t\tnonpoly ode\n')
        stream.write('\t\t\t{\n')
        
        for sysState in allPlantStates:
            if sysState in plant[modeId]['dynamics']:
                stream.write('\t\t\t\t' + plant[modeId]['dynamics'][sysState])
            else:
                stream.write('\t\t\t\t' + sysState + '\' = 0\n')

        stream.write('\t\t\t\tclock\' = 1\n')
        stream.write('\t\t\t}\n')
        stream.write('\t\t\tinv\n')
        stream.write('\t\t\t{\n')

        usedClock = False

        for inv in plant[modeId]['invariants']:
            stream.write('\t\t\t\t' + inv + '\n')

            if 'clock' in inv:
                usedClock = True

        if not usedClock:
            stream.write('\t\t\t\tclock <= 0')

        stream.write('\n')
        stream.write('\t\t\t}\n')
        stream.write('\t\t}\n')

def writeEndMode(stream, dynamics, name):
    stream.write('\t\t' + name + '\n')
    stream.write('\t\t{\n')
    stream.write('\t\t\tnonpoly ode\n')
    stream.write('\t\t\t{\n')

    for sysState in dynamics:
        if not 'clock' in sysState:
            stream.write('\t\t\t\t' + sysState + '\' = 0\n')

    stream.write('\t\t\t\tclock\' = 1\n')
    stream.write('\t\t\t}\n')

    stream.write('\t\t\tinv\n')
    stream.write('\t\t\t{\n')

    stream.write('\t\t\t\tclock <= 0\n')

    stream.write('\t\t\t}\n')
    stream.write('\t\t}\n')
        
def writeDnnJumps(stream, weights, offsets, activations, dynamics):
    numLayers = len(offsets)

    #jump from m0 to DNN-----------------------------------------------------
    writeIdentityDnnJump(stream, 'm0', 'DNNm1', dynamics)

def writeIdentityDnnJump(stream, curModeName, nextModeName, dynamics):

    stream.write('\t\t' + curModeName + ' -> ' + nextModeName + '\n')

    stream.write('\t\tguard { clock = 0 }\n')

    stream.write('\t\treset { ')
        
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')        
    
def writePlantJumps(stream, plant, numNeurLayers):

    for modeId in plant:
        for trans in plant[modeId]['transitions']:

            for i in range(1, int(round(len(plant[modeId]['transitions'][trans])/2)) + 1):

                curModeName = ''
                nextModeName = ''

                if 'name' in plant[modeId] and len(plant[modeId]['name']) > 0:
                    curModeName = plant[modeId]['name']
                    
                if 'name' in plant[trans[1]] and len(plant[trans[1]]['name']) > 0:
                    nextModeName = plant[trans[1]]['name']
                
                stream.write('\t\t' + curModeName + 'm' + str(trans[0] + numNeurLayers) + \
                         ' -> ' + nextModeName + 'm' + str(trans[1] + numNeurLayers) + '\n')
                stream.write('\t\tguard { ')

                for guard in plant[modeId]['transitions'][trans]['guards' + str(i)]:
                    stream.write(guard + ' ')

                stream.write('}\n')

                stream.write('\t\treset { ')

                usedClock = False
                
                for reset in plant[modeId]['transitions'][trans]['reset' + str(i)]:
                    stream.write(reset + ' ')
                    if 'clock' in reset:
                        usedClock = True
                        
                if not usedClock:
                    stream.write('clock\' := 0')
                
                stream.write('}\n')
                stream.write('\t\tinterval aggregation\n')

def writeDnn2PlantJumps(stream, trans, numNeurLayers, lastActivation, plant):

    for modeId in trans:

        for i in range(1, int(round(len(trans[modeId])/2)) + 1):
        
            stream.write('\t\tDNNm1 -> ')

            if 'name' in plant[modeId]:
                stream.write(plant[modeId]['name'])
            
            stream.write('m' + str(numNeurLayers + modeId) + '\n')
            stream.write('\t\tguard { ')

            for guard in trans[modeId]['guards' + str(i)]:
                stream.write(guard + ' ')
            
            stream.write('}\n')

            stream.write('\t\treset { ')

            for reset in trans[modeId]['reset' + str(i)]:
                stream.write(reset + ' ')

            stream.write('clock\' := 0')
            stream.write('}\n')
            stream.write('\t\tinterval aggregation\n')

def writePlant2DnnJumps(stream, trans, dynamics, numNeurLayers, plant):

    for nextTrans in trans:

        for i in range(1, int(round(len(trans[nextTrans])/2)) + 1):

            stream.write('\t\t')
            if 'name' in plant[nextTrans]:
                stream.write(plant[nextTrans]['name'])
            
            stream.write('m' + str(nextTrans + numNeurLayers) + ' -> m0\n')
            stream.write('\t\tguard { ')

            for guard in trans[nextTrans]['guards' + str(i)]:
                stream.write(guard + ' ')

            stream.write('}\n')

            stream.write('\t\treset { ')

            for reset in trans[nextTrans]['reset' + str(i)]:
                stream.write(reset + ' ')

            stream.write('clock\' := 0')
            stream.write('}\n')
            stream.write('\t\tinterval aggregation\n')

def writeEndJump(stream):

    stream.write('\t\t_cont_m2 ->  m_end_pl\n')
    stream.write('\t\tguard { k = ' + str(NUM_STEPS) + ' y1 <= ' + str(POS_LB) + '}\n')
    stream.write('\t\treset { ')
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')

    stream.write('\t\t_cont_m2 ->  m_end_pr\n')
    stream.write('\t\tguard { k = ' + str(NUM_STEPS) + ' y1 >= ' + str(POS_UB) + '}\n')
    stream.write('\t\treset { ')
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')

    stream.write('\t\t_cont_m2 ->  m_end_hr\n')
    stream.write('\t\tguard { k = ' + str(NUM_STEPS) + ' y4 <= ' + str(HEADING_LB) + '}\n')
    stream.write('\t\treset { ')
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')

    stream.write('\t\t_cont_m2 ->  m_end_hl\n')
    stream.write('\t\tguard { k = ' + str(NUM_STEPS) + ' y4 >= ' + str(HEADING_UB) + '}\n')
    stream.write('\t\treset { ')
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')

    stream.write('\t\t_cont_m2 ->  m_end_sr\n')
    stream.write('\t\tguard { k = ' + str(NUM_STEPS) + ' y3 >= ' + str(2.4 + SPEED_EPSILON) + ' }\n')
    stream.write('\t\treset { ')
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')

    stream.write('\t\t_cont_m2 ->  m_end_sl\n')
    stream.write('\t\tguard { k = ' + str(NUM_STEPS) + ' y3 <= ' + str(2.4 - SPEED_EPSILON) + ' }\n')
    stream.write('\t\treset { ')
    stream.write('clock\' := 0')
    stream.write('}\n')
    stream.write('\t\tinterval aggregation\n')        
            
def writeInitCond(stream, initProps, numInputs, initMode = 'm0'):
            
    stream.write('\tinit\n')
    stream.write('\t{\n')
    stream.write('\t\t' + initMode + '\n')
    stream.write('\t\t{\n')

    for prop in initProps:
        stream.write('\t\t\t' + prop + '\n')

    stream.write('\t\t\tclock in [0, 0]\n')  
    stream.write('\t\t}\n')
    stream.write('\t}\n')


def getNumNeurLayers(activations):

    count = 0

    for layer in activations:
        
        if 'Sigmoid' in activations[layer] or 'Tanh' in activations[layer] or 'Relu' in activations[layer]:
            count += 1
            
        count += 1

    return count

def getNumStates(offsets):
    numStates = 0
    for offset in offsets:
        if len(offsets[offset]) > numStates:
            numStates = len(offsets[offset])

    return numStates

def getInputLBUB(state, bounds, weights, offsets):
    lbSum = 0
    ubSum = 0

    varIndex = 0
    for inVar in bounds:
        weight = weights[1][state][varIndex]
        if weight >= 0:
            lbSum += weight * bounds[inVar][0]
            ubSum += weight * bounds[inVar][1]
        else:
            lbSum += weight * bounds[inVar][1]
            ubSum += weight * bounds[inVar][0]

        varIndex += 1

    lb = lbSum + offsets[1][state]
    ub = ubSum + offsets[1][state]

    numLayers = len(offsets)
    if numLayers > 1:
        for layer in range(1, numLayers):
            lbSum = 0
            ubSum = 0

            for weight in weights[layer + 1][state]:
                if weight >= 0:
                    ubSum += weight
                else:
                    lbSum += weight

            if ubSum + offsets[layer + 1][state] > ub:
                ub = ubSum + offsets[layer + 1][state]

            if lbSum + offsets[layer + 1][state] < lb:
                lb = lbSum + offsets[layer + 1][state]
            
    return (lb, ub)

'''
1. initProps is a list of properties written in strings that can be parsed by Flow*
  -- assumes the states are given as 'xi'
2. dnn is a dictionary such that:
  -- key 'weights' is a dictionary mapping layer index
     to a MxN-dimensional list of weights
  -- key 'offsets'  is a dictionary mapping layer index
     to a list of offsets per neuron in that layer
  -- key 'activations' is a dictionary mapping layer index
     to the layer activation function type
3. plant is a dictionary such that:
  -- Each dictionary key is a mode id that maps to a dictionary such that:
    -- key 'dynamics' maps to a dictionary of the dynamics of each var in that mode such that:
      -- each key is of the form 'xi' and maps to a dynamics string that can be parsed by Flow*
      -- assume inputs in dynamics are coded as 'ci' to make composition work
    -- key 'invariants' maps to a list of invariants that can be parsed by Flow*
    -- key 'transitions' maps to a dictionary such that:
      -- each key is a tuple of the form '(mode id, mode id)' that maps to a dictionary such that:
        -- key 'guards' maps to a list of guards that can be parsed by Flow*
        -- key 'reset' maps to a list of resets that can be parsed by Flow*
    -- key 'odetype' maps to a string describing the Flow* dynamics ode type 
4. glueTrans is a dictionary such that:
  -- key 'dnn2plant' maps to a dictionary such that:
    -- each key is an int specifying plant mode id that maps to a dictionary such that:
       -- key 'guards' maps to a list of guards that can be parsed by Flow*
       -- key 'reset' maps to a list of resets that can be parsed by Flow*
  -- key 'plant2dnn' maps to a dictionary such that:
    -- each key is an int specifying plant mode id that maps to a dictionary such that:
       -- key 'guards' maps to a list of guards that can be parsed by Flow*
       -- key 'reset' maps to a list of resets that can be parsed by Flow*
5. safetyProps is assumed to be a string containing a 
   logic formula that can be parsed by Flow*'''
def writeComposedSystem(filename, initProps, dnn, plant, glueTrans, safetyProps, numSteps):

    with open(filename, 'w') as stream:

        stream.write('hybrid reachability\n')
        stream.write('{\n')

        #encode variable names--------------------------------------------------
        stream.write('\t' + 'state var ')

        #numNeurStates = getNumStates(dnn['offsets'])
        #numNeurLayers = getNumNeurLayers(dnn['activations'])
        numNeurLayers = 1
        numSysStates = len(plant[1]['dynamics'])
        numInputs = len(dnn['weights'][1][0])

        plant_states = []
            
        if 'states' in plant[1]:
            for index in range(len(plant[1]['states'])):
                stream.write(plant[1]['states'][index] + ', ')
                plant_states.append(plant[1]['states'][index])

        # add any remaining states
        for state in plant[1]['dynamics']:
            if 'clock' in state:
                continue
            
            if state in plant_states:
                continue

            else:
                plant_states.append(state)
            
            stream.write(state + ', ')

        # for i in range(numNeurStates):
        #     stream.write('_f' + str(i + 1) + ', ')
        
        stream.write('clock\n\n')

        #settings---------------------------------------------------------------
        stream.write('\tsetting\n')
        stream.write('\t{\n')
        stream.write('\t\tadaptive steps {min 1e-6, max 0.005}\n') # F1/10 case study (HSCC)
        stream.write('\t\ttime ' + str(numSteps * (0.1)) + '\n') #F1/10 case study (HSCC)
        stream.write('\t\tremainder estimation 1e-1\n')
        stream.write('\t\tidentity precondition\n')
        stream.write('\t\tmatlab octagon y1, y2\n')
        stream.write('\t\tfixed orders 4\n')
        stream.write('\t\tcutoff 1e-12\n')
        stream.write('\t\tprecision 100\n')
        stream.write('\t\toutput f1tenth_tanh_verisig\n')
        stream.write('\t\tmax jumps ' + str((numNeurLayers + 2 + 10 + 6 * numInputs) * numSteps) + '\n') #F1/10
        #stream.write('\t\tmax jumps 10\n') #F1/10 
        stream.write('\t\tprint on\n')
        stream.write('\t}\n\n')

        #encode modes-----------------------------------------------------------------------------------------------
        stream.write('\tmodes\n')
        stream.write('\t{\n')
 
        writeDnnModes(stream, dnn['weights'], dnn['offsets'], dnn['activations'], plant[1]['dynamics'], plant_states)
        writePlantModes(stream, plant, plant_states, numNeurLayers)

        writeEndMode(stream, plant[1]['dynamics'], 'm_end_pr')
        writeEndMode(stream, plant[1]['dynamics'], 'm_end_pl')
        writeEndMode(stream, plant[1]['dynamics'], 'm_end_hr')
        writeEndMode(stream, plant[1]['dynamics'], 'm_end_hl')
        writeEndMode(stream, plant[1]['dynamics'], 'm_end_sr')
        writeEndMode(stream, plant[1]['dynamics'], 'm_end_sl')

        #close modes brace
        stream.write('\t}\n')
 
        #encode jumps----------------------------------------------------------------------------------------------
        stream.write('\tjumps\n')
        stream.write('\t{\n')

        writeDnnJumps(stream, dnn['weights'], dnn['offsets'], dnn['activations'], plant[1]['dynamics'])
        writeDnn2PlantJumps(stream, glueTrans['dnn2plant'], numNeurLayers, dnn['activations'][len(dnn['activations'])], plant)
        writePlantJumps(stream, plant, numNeurLayers)
        writePlant2DnnJumps(stream, glueTrans['plant2dnn'], plant[1]['dynamics'], numNeurLayers, plant)
        writeEndJump(stream)
        
        #close jumps brace
        stream.write('\t}\n')

        #encode initial condition----------------------------------------------------------------------------------
        writeInitCond(stream, initProps, numInputs, 'm3') #F1/10 (HSCC)
        
        #close top level brace
        stream.write('}\n')
        
        #encode unsafe set------------------------------------------------------------------------------------------
        stream.write(safetyProps)


def main(argv):    

    dnnYaml = 'tanh64x64.yml'
    numRays = 21
    
    plantPickle = 'dynamics_' + name + '{}.pickle'.format(numRays)
    gluePickle = 'glue_{}.pickle'.format(numRays)

    with open(plantPickle, 'rb') as f:

        plant = pickle.load(f)

    with open(gluePickle, 'rb') as f:

        glue = pickle.load(f)

    WALL_MIN = str(WALL_LIMIT)
    WALL_MAX = str(HALLWAY_WIDTH - WALL_LIMIT)

    wall_dist = getCornerDist()

    # F1/10 Safety + Reachability
    safetyProps = 'unsafe\n{\tleft_wallm2000001\n\t{\n\t\ty1 <= ' + WALL_MIN + '\n\n\t}\n' \
        + '\tright_bottom_wallm3000001\n\t{'\
        + '\n\t\ty1 >= ' + WALL_MAX + '\n\t\ty2 >= ' + str(wall_dist - WALL_LIMIT) + '\n\n\t}\n' \
        + '\ttop_wallm4000001\n\t{\n\t\t ' + str(NORMAL_TO_TOP_WALL[0]) + ' * y2 + ' + str(NORMAL_TO_TOP_WALL[1]) + ' * y1 <= ' + WALL_MIN + '\n\n\t}\n' \
        + '\tm_end_pl\n\t{\n\t\ty1 <= ' + str(POS_LB) + '\n\n\t}\n' \
        + '\tm_end_pr\n\t{\n\t\ty1 >= ' + str(POS_UB) + '\n\n\t}\n' \
        + '\tm_end_hl\n\t{\n\t\ty4 >= ' + str(HEADING_UB) + '\n\n\t}\n' \
        + '\tm_end_hr\n\t{\n\t\ty4 <= ' + str(HEADING_LB) + '\n\n\t}\n}' \
        + '\tm_end_sr\n\t{\n\t\ty3 >= ' + str(2.4 + SPEED_EPSILON) + '\n\n\t}\n' \
        + '\tm_end_sl\n\t{\n\t\ty3 <= ' + str(2.4 - SPEED_EPSILON) + '\n\n\t}\n}'

    modelFolder = '.'
    if not os.path.exists(modelFolder):
        os.makedirs(modelFolder)

    curLBPos = 0.75
    posOffset = 0.005

    headingOffset = 0.005

    init_y2 = 8
    if TURN_ANGLE == -np.pi/2:
        init_y2 = 7
    
    count = 1

    initProps = ['y1 in [' + str(curLBPos) + ', ' + str(curLBPos + posOffset) + ']',
                 'y2 in [' + str(init_y2) + ', ' + str(init_y2) + ']',
                 'y3 in [' + str(2.4 - SPEED_EPSILON) + ', ' + str(2.4 + SPEED_EPSILON) + ']',
                 'y4 in [' + str(-headingOffset) + ', ' + str(headingOffset) + ']', 'k in [0, 0]',
                 'u in [0, 0]', 'angle in [0, 0]', 'temp1 in [0, 0]', 'temp2 in [0, 0]',
                 'theta_l in [0, 0]', 'theta_r in [0, 0]']  # F1/10
    
    with open(dnnYaml, 'rb') as f:

        dnn = yaml.load(f)
            
    with open(plantPickle, 'rb') as f:

        plant = pickle.load(f)
    
    with open(gluePickle, 'rb') as f:

        glue = pickle.load(f)

    curModelFile = 'f1tenth_tanh_verisig.model'

    writeComposedSystem(curModelFile, initProps, dnn, plant, glue, safetyProps, NUM_STEPS)

if __name__ == '__main__':
    main(sys.argv[1:])    
