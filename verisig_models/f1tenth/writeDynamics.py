import numpy as np
from six.moves import cPickle as pickle

# k is the current step
# u is the NN's input
# y1 is distance to side wall
# y2 is distance to front wall
# y3 is linear velocity
# y4 is car heading in radians (relative to hallway)

# angle is the rotating lidar angle
# theta_l and theta_r are angles w.r.t. left and right wall, respectively
# temp1 and temp2 are used in various computations (denoted in each one)
# f_i are lidar rays (assuming lidar rays are -LIDAR_RANGE:LIDAR_OFFSET:LIDAR_RANGE)

MAX_TURNING_INPUT = 15  # in degrees
CONST_THROTTLE = 16  # constant throttle input for this case study

CAR_LENGTH = .45  # in m
CAR_LENGTH_INV = 1 / CAR_LENGTH  # in m
CAR_CENTER_OF_MASS = 0.225  # from rear of car (m)
CAR_ACCEL_CONST = 1.633
CAR_MOTOR_CONST = 0.2  # 45 MPH top speed (20 m/s) at 100 throttle

LIDAR_MAX_DISTANCE = 5  # in m
LIDAR_RANGE = 115 * np.pi / 180  # in radians
LIDAR_OFFSET = 11.5 * np.pi / 180  # in radians
NUM_RAYS = int(round((2 * LIDAR_RANGE) / LIDAR_OFFSET)) + 1

print(NUM_RAYS)

TURN_ANGLE = - np.pi/2
#TURN_ANGLE = - 2 * np.pi/3

HALLWAY_WIDTH = 1.5
HALLWAY_LENGTH = 20
MODE_SWITCH_OFFSET = 20 # set this high in order to avoid a reset in this comparison
WALL_LIMIT = 0.15

NUMERIC_OFFSET = 0.2

CORNER_ANGLE = np.pi - np.abs(TURN_ANGLE)
SIN_CORNER = np.sin(CORNER_ANGLE)
COS_CORNER = np.cos(CORNER_ANGLE)

name = 'sharp_turn_'

# just a check to avoid numerical error
if TURN_ANGLE == -np.pi/2:
    SIN_CORNER = 1
    COS_CORNER = 0
    name = 'right_turn_'

NORMAL_TO_TOP_WALL = [SIN_CORNER, -COS_CORNER]

TIME_STEP = 0.1  # in s

PIBY2 = np.pi / 2
PIBY180 = np.pi / 180.0
ONE80BYPI = 180.0 / np.pi

HYSTERESIS_CONSTANT = 4

WALL_MIN = str(WALL_LIMIT)
WALL_MAX = str(HALLWAY_WIDTH - WALL_LIMIT)


def getCornerDist(next_heading=np.pi/2 + TURN_ANGLE, reverse_cur_heading=-np.pi/2,
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


plant = {}

plant[1] = {}
plant[1]['name'] = '_cont_'
plant[1]['states'] = ['angle', 'theta_l', 'theta_r',
                      'temp1', 'temp2', 'y1', 'y2', 'y3', 'y4', 'u', 'k']
plant[1]['odetype'] = 'nonpoly ode'
plant[1]['dynamics'] = {}
plant[1]['dynamics']['y1'] = 'y1\' = -y3 * sin(y4)\n'
plant[1]['dynamics']['y2'] = 'y2\' = -y3 * cos(y4)\n'
plant[1]['dynamics']['y3'] = 'y3\' = ' + str(CAR_ACCEL_CONST) +\
    ' * ' + str(CAR_MOTOR_CONST) + ' * (' + str(CONST_THROTTLE) +\
    ' - ' + str(HYSTERESIS_CONSTANT) + ') - ' + str(CAR_ACCEL_CONST) + ' * y3\n'
plant[1]['dynamics']['y4'] = 'y4\' = ' + str(CAR_LENGTH_INV) + ' * y3 * sin(u) / cos(u)\n'
plant[1]['dynamics']['k'] = 'k\' = 0\n'
plant[1]['dynamics']['u'] = 'u\' = 0\n'
plant[1]['dynamics']['angle'] = 'angle\' = 0\n'
plant[1]['dynamics']['clock'] = 'clock\' = 0\n'
plant[1]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[1]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[1]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[1]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[1]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

plant[1]['invariants'] = ['clock <= ' + str(TIME_STEP)]

plant[1]['transitions'] = {}
plant[1]['transitions'][(1, 2)] = {}
plant[1]['transitions'][(1, 2)]['guards1'] =\
    ['clock = ' + str(TIME_STEP), 'y1 >= ' + str(MODE_SWITCH_OFFSET)]
plant[1]['transitions'][(1, 2)]['reset1'] =\
    ['clock\' := 0', 'k\' := k + 1', 'y4\' := y4 + ' + str(-TURN_ANGLE),
     'y1\' := ' + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1',
     'y2\' := ' + str(HALLWAY_LENGTH) + ' - ' + str(COS_CORNER) +
     ' * y2 - ' + str(SIN_CORNER) + ' * y1']

plant[1]['transitions'][(1, 2)]['guards2'] =\
    ['clock = ' + str(TIME_STEP), 'y1 <= ' + str(MODE_SWITCH_OFFSET)]
plant[1]['transitions'][(1, 2)]['reset2'] = ['clock\' := 0', 'k\' := k + 1']

plant[1]['transitions'][(1, 2000000)] = {}
plant[1]['transitions'][(1, 2000000)]['guards1'] = ['y1 <= ' + WALL_MIN]
plant[1]['transitions'][(1, 2000000)]['reset1'] = []

plant[1]['transitions'][(1, 3000000)] = {}
plant[1]['transitions'][(1, 3000000)]['guards1'] =\
        ['y1 >= ' + WALL_MAX,
         str(NORMAL_TO_TOP_WALL[0]) + ' * y2 + ' + str(NORMAL_TO_TOP_WALL[1]) + ' * y1  >= ' + WALL_MAX]
plant[1]['transitions'][(1, 3000000)]['reset1'] = []

plant[1]['transitions'][(1, 4000000)] = {}
plant[1]['transitions'][(1, 4000000)]['guards1'] =\
        [str(NORMAL_TO_TOP_WALL[0]) + ' * y2 + ' +
         str(NORMAL_TO_TOP_WALL[1]) + ' * y1  <= ' + WALL_MIN]
plant[1]['transitions'][(1, 4000000)]['reset1'] = []

plant[1000000] = {}
plant[1000000]['name'] = 'past_goal'
plant[1000000]['odetype'] = 'nonpoly ode'
plant[1000000]['dynamics'] = {}
plant[1000000]['dynamics']['y1'] = 'y1\' = 0\n'
plant[1000000]['dynamics']['y2'] = 'y2\' = 0\n'
plant[1000000]['dynamics']['y3'] = 'y3\' = 0\n'
plant[1000000]['dynamics']['y4'] = 'y4\' = 0\n'
plant[1000000]['dynamics']['k'] = 'k\' = 0\n'
plant[1000000]['dynamics']['u'] = 'u\' = 0\n'
plant[1000000]['dynamics']['angle'] = 'angle\' = 0\n'
plant[1000000]['dynamics']['clock'] = 'clock\' = 0\n'
plant[1000000]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[1000000]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[1000000]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[1000000]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[1000000]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

plant[1000000]['invariants'] = []
plant[1000000]['transitions'] = {}

plant[2000000] = {}
plant[2000000]['name'] = 'left_wall'
plant[2000000]['odetype'] = 'nonpoly ode'
plant[2000000]['dynamics'] = {}
plant[2000000]['dynamics']['y1'] = 'y1\' = 0\n'
plant[2000000]['dynamics']['y2'] = 'y2\' = 0\n'
plant[2000000]['dynamics']['y3'] = 'y3\' = 0\n'
plant[2000000]['dynamics']['y4'] = 'y4\' = 0\n'
plant[2000000]['dynamics']['k'] = 'k\' = 0\n'
plant[2000000]['dynamics']['u'] = 'u\' = 0\n'
plant[2000000]['dynamics']['angle'] = 'angle\' = 0\n'
plant[2000000]['dynamics']['clock'] = 'clock\' = 0\n'
plant[2000000]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[2000000]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[2000000]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[2000000]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[2000000]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

plant[2000000]['invariants'] = []
plant[2000000]['transitions'] = {}

plant[3000000] = {}
plant[3000000]['name'] = 'right_bottom_wall'
plant[3000000]['odetype'] = 'nonpoly ode'
plant[3000000]['dynamics'] = {}
plant[3000000]['dynamics']['y1'] = 'y1\' = 0\n'
plant[3000000]['dynamics']['y2'] = 'y2\' = 0\n'
plant[3000000]['dynamics']['y3'] = 'y3\' = 0\n'
plant[3000000]['dynamics']['y4'] = 'y4\' = 0\n'
plant[3000000]['dynamics']['k'] = 'k\' = 0\n'
plant[3000000]['dynamics']['u'] = 'u\' = 0\n'
plant[3000000]['dynamics']['angle'] = 'angle\' = 0\n'
plant[3000000]['dynamics']['clock'] = 'clock\' = 0\n'
plant[3000000]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[3000000]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[3000000]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[3000000]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[3000000]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

plant[3000000]['invariants'] = []
plant[3000000]['transitions'] = {}

plant[4000000] = {}
plant[4000000]['name'] = 'top_wall'
plant[4000000]['odetype'] = 'nonpoly ode'
plant[4000000]['dynamics'] = {}
plant[4000000]['dynamics']['y1'] = 'y1\' = 0\n'
plant[4000000]['dynamics']['y2'] = 'y2\' = 0\n'
plant[4000000]['dynamics']['y3'] = 'y3\' = 0\n'
plant[4000000]['dynamics']['y4'] = 'y4\' = 0\n'
plant[4000000]['dynamics']['k'] = 'k\' = 0\n'
plant[4000000]['dynamics']['u'] = 'u\' = 0\n'
plant[4000000]['dynamics']['angle'] = 'angle\' = 0\n'
plant[4000000]['dynamics']['clock'] = 'clock\' = 0\n'
plant[4000000]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[4000000]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[4000000]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[4000000]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[4000000]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

plant[4000000]['invariants'] = []
plant[4000000]['transitions'] = {}

# end of plant dynanmics

# beginning of lidar model

wall_dist = getCornerDist()

# this mode is used to prepare the temp variables for the div mode
plant[2] = {}
plant[2]['name'] = ''
plant[2]['odetype'] = 'lti ode'
plant[2]['dynamics'] = {}
plant[2]['dynamics']['y1'] = 'y1\' = 0\n'
plant[2]['dynamics']['y2'] = 'y2\' = 0\n'
plant[2]['dynamics']['y3'] = 'y3\' = 0\n'
plant[2]['dynamics']['y4'] = 'y4\' = 0\n'
plant[2]['dynamics']['k'] = 'k\' = 0\n'
plant[2]['dynamics']['u'] = 'u\' = 0\n'
plant[2]['dynamics']['angle'] = 'angle\' = 0\n'
plant[2]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[2]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[2]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[2]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[2]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[2]['invariants'] = ['clock <= 0']
plant[2]['transitions'] = {}
plant[2]['transitions'][(2, 3)] = {}
plant[2]['transitions'][(2, 3)]['guards1'] = ['clock = 0', 'y2 >= ' +
                                              str(wall_dist), 'y1 <= ' + str(HALLWAY_WIDTH)]
plant[2]['transitions'][(2, 3)]['reset1'] =\
    ['clock\' := 0',
     'temp1\' := y2',
     #'temp2\' := (y2 - ' + str(wall_dist) + ')',
     'temp2\' := ' + str(HALLWAY_WIDTH) + ' - y1',
     'theta_l\' := 0',
     'theta_r\' := 0']

plant[2]['transitions'][(2, 3)]['guards2'] = ['clock = 0', 'y2 <= ' +
                                              str(wall_dist), 'y1 <= ' + str(HALLWAY_WIDTH),
                                              'y1 >= ' + str(HALLWAY_WIDTH - NUMERIC_OFFSET)]
plant[2]['transitions'][(2, 3)]['reset2'] =\
    ['clock\' := 0',
     'temp1\' := y2',
     'temp2\' := ' + str(wall_dist) + ' - y2',
     'theta_l\' := 0',
     'theta_r\' := 0']

plant[2]['transitions'][(2, 3)]['guards3'] = ['clock = 0', 'y2 <= ' + str(wall_dist),
                                              'y1 <= ' + str(HALLWAY_WIDTH - NUMERIC_OFFSET)]
plant[2]['transitions'][(2, 3)]['reset3'] =\
    ['clock\' := 0',
     'temp1\' := y2',
     'temp2\' := ' + str(HALLWAY_WIDTH) + ' - y1',
     'theta_l\' := 0',
     'theta_r\' := 0']

plant[2]['transitions'][(2, 3)]['guards4'] = ['clock = 0', 'y2 <= ' +
                                              str(wall_dist), 'y1 >= ' + str(HALLWAY_WIDTH + NUMERIC_OFFSET)]
plant[2]['transitions'][(2, 3)]['reset4'] =\
    ['clock\' := 0',
     'temp1\' := y2',
     'temp2\' := y1 - ' + str(HALLWAY_WIDTH),
     'theta_l\' := 0',
     'theta_r\' := 0']

plant[2]['transitions'][(2, 3)]['guards5'] = ['clock = 0', 'y2 <= ' +
                                              str(wall_dist), 'y1 <= ' + str(HALLWAY_WIDTH + NUMERIC_OFFSET),
                                              'y1 >= ' + str(HALLWAY_WIDTH)]
plant[2]['transitions'][(2, 3)]['reset5'] =\
    ['clock\' := 0',
     'temp1\' := y2',
     'temp2\' := ' + str(wall_dist) + ' - y2',
     'theta_l\' := 0',
     'theta_r\' := 0']

plant[2]['transitions'][(2, 3)]['guards6'] = ['clock = 0', 'y2 >= ' +
                                              str(wall_dist), 'y1 >= ' + str(HALLWAY_WIDTH)]
plant[2]['transitions'][(2, 3)]['reset6'] =\
    ['clock\' := 0',
     'temp1\' := y2',
     'temp2\' := (y1 - ' + str(HALLWAY_WIDTH) + ')',
     'theta_l\' := 0',
     'theta_r\' := 0']

# need an empty mode to first perform the temp1 and temp2 resets
plant[3] = {}
plant[3]['name'] = ''
plant[3]['odetype'] = 'lti ode'
plant[3]['dynamics'] = {}
plant[3]['dynamics']['y1'] = 'y1\' = 0\n'
plant[3]['dynamics']['y2'] = 'y2\' = 0\n'
plant[3]['dynamics']['y3'] = 'y3\' = 0\n'
plant[3]['dynamics']['y4'] = 'y4\' = 0\n'
plant[3]['dynamics']['k'] = 'k\' = 0\n'
plant[3]['dynamics']['u'] = 'u\' = 0\n'
plant[3]['dynamics']['angle'] = 'angle\' = 0\n'
plant[3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[3]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[3]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[3]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[3]['invariants'] = ['clock <= 0']
plant[3]['transitions'] = {}
plant[3]['transitions'][(3, 4)] = {}
plant[3]['transitions'][(3, 4)]['guards1'] = ['clock = 0']
plant[3]['transitions'][(3, 4)]['reset1'] = ['clock\' := 0']

# modes reg1, reg2, reg3 correspond to the car being in Region 1, 2, 3, respectively
reg1 = 6
reg2 = 7
reg3 = 8

# longest path is currently through reg2 modes

plant[4] = {}
plant[4]['name'] = '_div_1_3_'
plant[4]['odetype'] = 'lti ode'
plant[4]['dynamics'] = {}
plant[4]['dynamics']['y1'] = 'y1\' = 0\n'
plant[4]['dynamics']['y2'] = 'y2\' = 0\n'
plant[4]['dynamics']['y3'] = 'y3\' = 0\n'
plant[4]['dynamics']['y4'] = 'y4\' = 0\n'
plant[4]['dynamics']['k'] = 'k\' = 0\n'
plant[4]['dynamics']['u'] = 'u\' = 0\n'
plant[4]['dynamics']['angle'] = 'angle\' = 0\n'
plant[4]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[4]['dynamics']['temp2'] = 'temp2\' = 0\n'
plant[4]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[4]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
for i in range(NUM_RAYS):
    plant[4]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[4]['invariants'] = ['clock <= 0']
plant[4]['transitions'] = {}
plant[4]['transitions'][(4, 5)] = {}
plant[4]['transitions'][(4, 5)]['guards1'] = ['clock = 0']
plant[4]['transitions'][(4, 5)]['reset1'] = ['clock\' := 0', 'theta_l\' := theta_l * y1']

plant[5] = {}
plant[5]['name'] = '_div_2_4_'
plant[5]['odetype'] = 'lti ode'
plant[5]['dynamics'] = {}
plant[5]['dynamics']['y1'] = 'y1\' = 0\n'
plant[5]['dynamics']['y2'] = 'y2\' = 0\n'
plant[5]['dynamics']['y3'] = 'y3\' = 0\n'
plant[5]['dynamics']['y4'] = 'y4\' = 0\n'
plant[5]['dynamics']['k'] = 'k\' = 0\n'
plant[5]['dynamics']['u'] = 'u\' = 0\n'
plant[5]['dynamics']['angle'] = 'angle\' = 0\n'
plant[5]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[5]['dynamics']['temp2'] = 'temp2\' = 0\n'
plant[5]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[5]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
for i in range(NUM_RAYS):
    plant[5]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[5]['invariants'] = ['clock <= 0']
plant[5]['transitions'] = {}
plant[5]['transitions'][(5, reg1)] = {}
plant[5]['transitions'][(5, reg1)]['guards1'] =\
                       ['clock = 0', 'y1 <= ' + str(HALLWAY_WIDTH), 'y2 >= ' + str(wall_dist)]
plant[5]['transitions'][(5, reg1)]['reset1'] =\
                       ['clock\' := 0',
                        'theta_r\' := theta_r * (y2 - ' + str(wall_dist) + ')']

if not TURN_ANGLE == -np.pi/2:
    plant[5]['transitions'][(5, reg1)]['guards2'] =\
        ['clock = 0', 'y1 <= ' + str(HALLWAY_WIDTH),
         str(NORMAL_TO_TOP_WALL[0]) + ' * y2 + ' +
         str(NORMAL_TO_TOP_WALL[1]) + ' * y1  >= ' + str(HALLWAY_WIDTH),
         'y2 <= ' + str(wall_dist)]
    plant[5]['transitions'][(5, reg1)]['reset2'] = ['clock\' := 0',
                                                    'theta_r\' := theta_r * (' + str(wall_dist) + ' - y2)']

plant[5]['transitions'][(5, reg2)] = {}
plant[5]['transitions'][(5, reg2)]['guards1'] =\
    ['clock = 0', 'y1 <= ' + str(HALLWAY_WIDTH - NUMERIC_OFFSET),
                  str(NORMAL_TO_TOP_WALL[0]) + ' * y2 + ' + str(NORMAL_TO_TOP_WALL[1]) + ' * y1  <= ' + str(HALLWAY_WIDTH)]
plant[5]['transitions'][(5, reg2)]['reset1'] =\
    ['clock\' := 0',
     'theta_r\' := theta_r * (' + str(wall_dist) + ' - y2)']

plant[5]['transitions'][(5, reg2)]['guards2'] =\
    ['clock = 0', 'y1 <= ' + str(HALLWAY_WIDTH), 'y1 >= ' + str(HALLWAY_WIDTH - NUMERIC_OFFSET),
                  str(NORMAL_TO_TOP_WALL[0]) + ' * y2 + ' + str(NORMAL_TO_TOP_WALL[1]) + ' * y1  <= ' + str(HALLWAY_WIDTH)]
plant[5]['transitions'][(5, reg2)]['reset2'] =\
    ['clock\' := 0',
     'theta_r\' := theta_r * (' + str(HALLWAY_WIDTH) + ' - y1)']

plant[5]['transitions'][(5, reg3)] = {}
plant[5]['transitions'][(5, reg3)]['guards1'] =\
    ['clock = 0', 'y1 >= ' + str(HALLWAY_WIDTH + NUMERIC_OFFSET), 'y2 <= ' + str(wall_dist)]
plant[5]['transitions'][(5, reg3)]['reset1'] =\
    ['clock\' := 0',
     'theta_r\' := theta_r * (' + str(wall_dist) + ' - y2)']

plant[5]['transitions'][(5, reg3)]['guards2'] =\
    ['clock = 0', 'y1 >= ' + str(HALLWAY_WIDTH), 'y1 <= ' + str(HALLWAY_WIDTH + NUMERIC_OFFSET), 'y2 <= ' + str(wall_dist)]
plant[5]['transitions'][(5, reg3)]['reset2'] =\
    ['clock\' := 0',
     'theta_r\' := theta_r * (y1 - ' + str(HALLWAY_WIDTH) + ')']

if not TURN_ANGLE == -np.pi/2:
    plant[5]['transitions'][(5, reg3)]['guards3'] =\
        ['clock = 0', 'y1 >= ' + str(HALLWAY_WIDTH), 'y2 >= ' + str(wall_dist)]
    plant[5]['transitions'][(5, reg3)]['reset3'] =\
        ['clock\' := 0',
         'theta_r\' := theta_r * (y2 - ' + str(wall_dist) + ')']

NUM_MODES_REG1 = 12
NUM_MODES_REG2 = 15
NUM_MODES_REG3 = 15

mode1_reg1 = reg3 + 4  # 14 currently
mode1_reg2 = mode1_reg1 + NUM_RAYS * NUM_MODES_REG1  # NUM_MODES_REG1 modes per ray in region 1
mode1_reg3 = mode1_reg2 + NUM_RAYS * NUM_MODES_REG2  # NUM_MODES_REG2 modes per ray in region 2

# temp1 is angle - theta_l (after all the reg1 modes)
# temp2 is angle - theta_r (after all the reg1 modes)

# temp1 = arctan(theta_l)
plant[reg1] = {}
plant[reg1]['name'] = '_arc_3_1_reg1'
plant[reg1]['odetype'] = 'lti ode'
plant[reg1]['dynamics'] = {}
plant[reg1]['dynamics']['y1'] = 'y1\' = 0\n'
plant[reg1]['dynamics']['y2'] = 'y2\' = 0\n'
plant[reg1]['dynamics']['y3'] = 'y3\' = 0\n'
plant[reg1]['dynamics']['y4'] = 'y4\' = 0\n'
plant[reg1]['dynamics']['k'] = 'k\' = 0\n'
plant[reg1]['dynamics']['u'] = 'u\' = 0\n'
plant[reg1]['dynamics']['angle'] = 'angle\' = 0\n'
plant[reg1]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[reg1]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[reg1]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[reg1]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[reg1]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[reg1]['invariants'] = ['clock <= 0']
plant[reg1]['transitions'] = {}
plant[reg1]['transitions'][(reg1, reg1 + 3)] = {}
plant[reg1]['transitions'][(reg1, reg1 + 3)]['guards1'] = ['clock = 0']
plant[reg1]['transitions'][(reg1, reg1 + 3)]['reset1'] = ['clock\' := 0']

# temp2 = arctan(theta_r)
plant[reg1 + 3] = {}
plant[reg1 + 3]['name'] = '_arc_4_2_'
plant[reg1 + 3]['odetype'] = 'lti ode'
plant[reg1 + 3]['dynamics'] = {}
plant[reg1 + 3]['dynamics']['y1'] = 'y1\' = 0\n'
plant[reg1 + 3]['dynamics']['y2'] = 'y2\' = 0\n'
plant[reg1 + 3]['dynamics']['y3'] = 'y3\' = 0\n'
plant[reg1 + 3]['dynamics']['y4'] = 'y4\' = 0\n'
plant[reg1 + 3]['dynamics']['k'] = 'k\' = 0\n'
plant[reg1 + 3]['dynamics']['u'] = 'u\' = 0\n'
plant[reg1 + 3]['dynamics']['angle'] = 'angle\' = 0\n'
plant[reg1 + 3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[reg1 + 3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[reg1 + 3]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[reg1 + 3]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[reg1 + 3]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[reg1 + 3]['invariants'] = ['clock <= 0']
plant[reg1 + 3]['transitions'] = {}
plant[reg1 + 3]['transitions'][(reg1 + 3, mode1_reg1)] = {}
plant[reg1 + 3]['transitions'][(reg1 + 3, mode1_reg1)
                               ]['guards1'] = ['clock = 0', 'y2 >= ' + str(wall_dist)]
plant[reg1 + 3]['transitions'][(reg1 + 3, mode1_reg1)]['reset1'] =\
                              ['clock\' := 0',
                               'theta_l\' := temp1',
                               'theta_r\' := - ' + str(np.pi/2) + ' + temp2',
                               'angle\' := y4 + ' + str(-LIDAR_RANGE),
                               'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                               'temp2\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp2 + ' + str(np.pi/2)]

if not TURN_ANGLE == -np.pi/2:
    plant[reg1 + 3]['transitions'][(reg1 + 3, mode1_reg1)
                                   ]['guards2'] = ['clock = 0', 'y2 <= ' + str(wall_dist)]
    plant[reg1 + 3]['transitions'][(reg1 + 3, mode1_reg1)]['reset2'] =\
                                  ['clock\' := 0',
                                   'theta_l\' := temp1',
                                   'theta_r\' := - ' + str(np.pi/2) + ' - temp2',
                                   'angle\' := y4 + ' + str(-LIDAR_RANGE),
                                   'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                                   'temp2\' := y4 + ' + str(-LIDAR_RANGE) + ' + ' + str(np.pi/2) + ' + temp2']

# temp1 is angle - theta_l (after all the reg2 modes)
# temp2 is angle - theta_r (after all the reg2 modes)

# temp2 = arctan(theta_l)
plant[reg2] = {}
plant[reg2]['name'] = '_arc_3_1_reg2'
plant[reg2]['odetype'] = 'lti ode'
plant[reg2]['dynamics'] = {}
plant[reg2]['dynamics']['y1'] = 'y1\' = 0\n'
plant[reg2]['dynamics']['y2'] = 'y2\' = 0\n'
plant[reg2]['dynamics']['y3'] = 'y3\' = 0\n'
plant[reg2]['dynamics']['y4'] = 'y4\' = 0\n'
plant[reg2]['dynamics']['k'] = 'k\' = 0\n'
plant[reg2]['dynamics']['u'] = 'u\' = 0\n'
plant[reg2]['dynamics']['angle'] = 'angle\' = 0\n'
plant[reg2]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[reg2]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[reg2]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[reg2]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[reg2]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[reg2]['invariants'] = ['clock <= 0']
plant[reg2]['transitions'] = {}
plant[reg2]['transitions'][(reg2, reg2 + 3)] = {}
plant[reg2]['transitions'][(reg2, reg2 + 3)]['guards1'] = ['clock = 0']
plant[reg2]['transitions'][(reg2, reg2 + 3)]['reset1'] = ['clock\' := 0']

# temp2 = arctan(theta_r)
plant[reg2 + 3] = {}
plant[reg2 + 3]['name'] = '_arc_4_2_'
plant[reg2 + 3]['odetype'] = 'lti ode'
plant[reg2 + 3]['dynamics'] = {}
plant[reg2 + 3]['dynamics']['y1'] = 'y1\' = 0\n'
plant[reg2 + 3]['dynamics']['y2'] = 'y2\' = 0\n'
plant[reg2 + 3]['dynamics']['y3'] = 'y3\' = 0\n'
plant[reg2 + 3]['dynamics']['y4'] = 'y4\' = 0\n'
plant[reg2 + 3]['dynamics']['k'] = 'k\' = 0\n'
plant[reg2 + 3]['dynamics']['u'] = 'u\' = 0\n'
plant[reg2 + 3]['dynamics']['angle'] = 'angle\' = 0\n'
plant[reg2 + 3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[reg2 + 3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[reg2 + 3]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[reg2 + 3]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[reg2 + 3]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[reg2 + 3]['invariants'] = ['clock <= 0']
plant[reg2 + 3]['transitions'] = {}
plant[reg2 + 3]['transitions'][(reg2 + 3, mode1_reg2)] = {}
plant[reg2 + 3]['transitions'][(reg2 + 3, mode1_reg2)]['guards1'] = ['clock = 0', 'y1 <= ' + str(HALLWAY_WIDTH - NUMERIC_OFFSET)]
plant[reg2 + 3]['transitions'][(reg2 + 3, mode1_reg2)]['reset1'] =\
                              ['clock\' := 0',
                               'theta_l\' := temp1',
                               'theta_r\' := - temp2 - ' + str(np.pi/2),
                               'angle\' := y4 + ' + str(-LIDAR_RANGE),
                               'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                               'temp2\' := y4 + ' + str(-LIDAR_RANGE) + ' + temp2 + ' + str(np.pi/2)]
plant[reg2 + 3]['transitions'][(reg2 + 3, mode1_reg2)]['guards2'] = ['clock = 0', 'y1 >= ' + str(HALLWAY_WIDTH - NUMERIC_OFFSET)]
plant[reg2 + 3]['transitions'][(reg2 + 3, mode1_reg2)]['reset2'] =\
                              ['clock\' := 0',
                               'theta_l\' := temp1',
                               'theta_r\' := temp2 - ' + str(np.pi),
                               'angle\' := y4 + ' + str(-LIDAR_RANGE),
                               'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                               'temp2\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp2 + ' + str(np.pi)]

# temp1 = arctan(theta_l)
plant[reg3] = {}
plant[reg3]['name'] = '_arc_3_1_reg3'
plant[reg3]['odetype'] = 'lti ode'
plant[reg3]['dynamics'] = {}
plant[reg3]['dynamics']['y1'] = 'y1\' = 0\n'
plant[reg3]['dynamics']['y2'] = 'y2\' = 0\n'
plant[reg3]['dynamics']['y3'] = 'y3\' = 0\n'
plant[reg3]['dynamics']['y4'] = 'y4\' = 0\n'
plant[reg3]['dynamics']['k'] = 'k\' = 0\n'
plant[reg3]['dynamics']['u'] = 'u\' = 0\n'
plant[reg3]['dynamics']['angle'] = 'angle\' = 0\n'
plant[reg3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[reg3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[reg3]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[reg3]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[reg3]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[reg3]['invariants'] = ['clock <= 0']
plant[reg3]['transitions'] = {}
plant[reg3]['transitions'][(reg3, reg3 + 3)] = {}
plant[reg3]['transitions'][(reg3, reg3 + 3)]['guards1'] = ['clock = 0']
plant[reg3]['transitions'][(reg3, reg3 + 3)]['reset1'] = []

# temp2 = arctan(theta_r)
plant[reg3 + 3] = {}
plant[reg3 + 3]['name'] = '_arc_4_2_'
plant[reg3 + 3]['odetype'] = 'lti ode'
plant[reg3 + 3]['dynamics'] = {}
plant[reg3 + 3]['dynamics']['y1'] = 'y1\' = 0\n'
plant[reg3 + 3]['dynamics']['y2'] = 'y2\' = 0\n'
plant[reg3 + 3]['dynamics']['y3'] = 'y3\' = 0\n'
plant[reg3 + 3]['dynamics']['y4'] = 'y4\' = 0\n'
plant[reg3 + 3]['dynamics']['k'] = 'k\' = 0\n'
plant[reg3 + 3]['dynamics']['u'] = 'u\' = 0\n'
plant[reg3 + 3]['dynamics']['angle'] = 'angle\' = 0\n'
plant[reg3 + 3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
plant[reg3 + 3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
plant[reg3 + 3]['dynamics']['temp1'] = 'temp1\' = 0\n'
plant[reg3 + 3]['dynamics']['temp2'] = 'temp2\' = 0\n'
for i in range(NUM_RAYS):
    plant[reg3 + 3]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'
plant[reg3 + 3]['invariants'] = ['clock <= 0']
plant[reg3 + 3]['transitions'] = {}
plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)] = {}
plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)
                               ]['guards1'] = ['clock = 0', 'y2 <= ' + str(wall_dist), 'y1 >= ' + str(HALLWAY_WIDTH + NUMERIC_OFFSET)]
plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)]['reset1'] =\
                              ['clock\' := 0',
                               'theta_l\' := temp1',
                               'theta_r\' := ' + str(np.pi/2) + ' + temp2',
                               'angle\' := y4 + ' + str(-LIDAR_RANGE),
                               'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                               'temp2\' := y4 + ' + str(-LIDAR_RANGE) + ' - ' + str(np.pi/2) + ' - temp2']

plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)
                               ]['guards2'] = ['clock = 0', 'y2 <= ' + str(wall_dist),
                                               'y1 <= ' + str(HALLWAY_WIDTH + NUMERIC_OFFSET),
                                               'y1 >= ' + str(HALLWAY_WIDTH)]
plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)]['reset2'] =\
                              ['clock\' := 0',
                               'theta_l\' := temp1',
                               'theta_r\' := ' + str(np.pi) + ' - temp2',
                               'angle\' := y4 + ' + str(-LIDAR_RANGE),
                               'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                               'temp2\' := y4 + ' + str(-LIDAR_RANGE) + ' - ' + str(np.pi) + ' + temp2']

if not TURN_ANGLE == -np.pi/2:
    plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)
                                   ]['guards3'] = ['clock = 0', 'y2 >= ' + str(wall_dist)]
    plant[reg3 + 3]['transitions'][(reg3 + 3, mode1_reg3)]['reset3'] =\
                                  ['clock\' := 0',
                                   'theta_l\' := temp1',
                                   'theta_r\' := ' + str(np.pi/2) + ' - temp2',
                                   'angle\' := y4 + ' + str(-LIDAR_RANGE),
                                   'temp1\' := y4 + ' + str(-LIDAR_RANGE) + ' - temp1',
                                   'temp2\' := y4 + ' + str(-LIDAR_RANGE - np.pi/2) + ' + temp2']

# Region 1
nextAngle = -LIDAR_RANGE + LIDAR_OFFSET
index = 0
curRay = 1

# while nextAngle <= LIDAR_RANGE + LIDAR_OFFSET:
while curRay <= NUM_RAYS:

    # name computation
    namePre = ''
    if nextAngle - LIDAR_OFFSET < 0:
        namePre = 'm'

    # first mode
    plant[mode1_reg1 + index] = {}
    plant[mode1_reg1 + index]['name'] = 'computing_ray_for_' + str(curRay)
    plant[mode1_reg1 + index]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index]['dynamics'] = {}
    plant[mode1_reg1 + index]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index]['transitions'] = {}

    # transitions to convert angle to (-180, 180), i.e., (-pi, pi)
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 + index, mode1_reg1 + index + 1)] = {}
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 + index, mode1_reg1 + index + 1)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(np.pi)]
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 + index, mode1_reg1 + index + 1)]['reset1'] =\
        ['angle\' := angle - ' + str(2 * np.pi),
         'temp1\' := temp1 - ' + str(2 * np.pi), 'temp2\' := temp2 - ' + str(2 * np.pi)]
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 + index, mode1_reg1 + index + 1)]['guards2'] =\
        ['clock = 0', 'angle <= ' + str(-np.pi)]
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 + index, mode1_reg1 + index + 1)]['reset2'] =\
        ['angle\' := angle + ' + str(2 * np.pi), 'temp1\' := temp1 + ' + str(2 * np.pi),
         'temp2\' := temp2 + ' + str(2 * np.pi)]
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 + index, mode1_reg1 + index + 1)]['guards3'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi)]
    plant[mode1_reg1 + index]['transitions'][(mode1_reg1 +
                                              index, mode1_reg1 + index + 1)]['reset3'] = []

    plant[mode1_reg1 + index + 1] = {}
    plant[mode1_reg1 + index + 1]['name'] = 'computing_ray_for_' + str(curRay)
    plant[mode1_reg1 + index + 1]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 1]['dynamics'] = {}
    plant[mode1_reg1 + index + 1]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 1]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 1]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 1]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 1]['transitions'] = {}

    # transition to correct wall

    # [-LIDAR_RANGE, theta_r]
    plant[mode1_reg1 + index +
          1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 2)] = {}
    plant[mode1_reg1 + index + 1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 2)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi), 'temp2 <= 0']
    plant[mode1_reg1 + index + 1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 2)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := 0',
         'angle\' := (' + str(np.pi/2) + ' + angle)']

    # (theta_r, theta_l]
    plant[mode1_reg1 + index +
          1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 3)] = {}
    plant[mode1_reg1 + index + 1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 3)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi),
         'temp2 >= 0', 'temp1 <= 0']
    plant[mode1_reg1 + index + 1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 3)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := 0',
         'angle\' := ' + str(np.pi/2 + TURN_ANGLE) + ' - angle']

    # (theta_l, LIDAR_RANGE]
    plant[mode1_reg1 + index +
          1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 4)] = {}
    plant[mode1_reg1 + index + 1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 4)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi), 'temp1 >= 0']
    plant[mode1_reg1 + index + 1]['transitions'][(mode1_reg1 + index + 1, mode1_reg1 + index + 4)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := 0',
         'angle\' := (' + str(np.pi/2) + ' - angle)']

    # compute cos(angle) [-LIDAR_RANGE, theta_r]
    plant[mode1_reg1 + index + 2] = {}
    plant[mode1_reg1 + index + 2]['name'] = 'right_wall_'
    plant[mode1_reg1 + index + 2]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 2]['dynamics'] = {}
    plant[mode1_reg1 + index + 2]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 2]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 2]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 2]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 2]['transitions'] = {}
    plant[mode1_reg1 + index +
          2]['transitions'][(mode1_reg1 + index + 2, mode1_reg1 + index + 5)] = {}
    plant[mode1_reg1 + index +
          2]['transitions'][(mode1_reg1 + index + 2, mode1_reg1 + index + 5)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index +
          2]['transitions'][(mode1_reg1 + index + 2, mode1_reg1 + index + 5)]['reset1'] = ['clock\' := 0']

    # compute cos(angle)
    plant[mode1_reg1 + index + 5] = {}
    plant[mode1_reg1 + index + 5]['name'] = '_cos_0_0_'
    plant[mode1_reg1 + index + 5]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 5]['dynamics'] = {}
    plant[mode1_reg1 + index + 5]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg1 + index + 5]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 5]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 5]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 5]['transitions'] = {}
    plant[mode1_reg1 + index +
          5]['transitions'][(mode1_reg1 + index + 5, mode1_reg1 + index + 6)] = {}
    plant[mode1_reg1 + index +
          5]['transitions'][(mode1_reg1 + index + 5, mode1_reg1 + index + 6)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index +
          5]['transitions'][(mode1_reg1 + index + 5, mode1_reg1 + index + 6)]['reset1'] = []

    # compute 1/cos(angle)
    plant[mode1_reg1 + index + 6] = {}
    plant[mode1_reg1 + index + 6]['name'] = '_div_0_0_'
    plant[mode1_reg1 + index + 6]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 6]['dynamics'] = {}
    plant[mode1_reg1 + index + 6]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg1 + index + 6]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 6]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 6]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 6]['transitions'] = {}
    plant[mode1_reg1 + index +
          6]['transitions'][(mode1_reg1 + index + 6, mode1_reg1 + index + 11)] = {}
    plant[mode1_reg1 + index +
          6]['transitions'][(mode1_reg1 + index + 6, mode1_reg1 + index + 11)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index + 6]['transitions'][(mode1_reg1 + index + 6, mode1_reg1 + index + 11)]['reset1'] = [
        'clock\' := 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := angle * (' + str(HALLWAY_WIDTH) + ' - y1)']

    # compute cos(angle) (theta_r, theta_l]
    plant[mode1_reg1 + index + 3] = {}
    plant[mode1_reg1 + index + 3]['name'] = 'front_wall_'
    plant[mode1_reg1 + index + 3]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 3]['dynamics'] = {}
    plant[mode1_reg1 + index + 3]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 3]['dynamics']['temp2'] = 'temp2\' = 0\n'

    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 3]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 3]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 3]['transitions'] = {}
    plant[mode1_reg1 + index +
          3]['transitions'][(mode1_reg1 + index + 3, mode1_reg1 + index + 7)] = {}
    plant[mode1_reg1 + index +
          3]['transitions'][(mode1_reg1 + index + 3, mode1_reg1 + index + 7)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index +
          3]['transitions'][(mode1_reg1 + index + 3, mode1_reg1 + index + 7)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg1 + index + 7] = {}
    plant[mode1_reg1 + index + 7]['name'] = '_cos_0_0_'
    plant[mode1_reg1 + index + 7]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 7]['dynamics'] = {}
    plant[mode1_reg1 + index + 7]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg1 + index + 7]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 7]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 7]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 7]['transitions'] = {}
    plant[mode1_reg1 + index +
          7]['transitions'][(mode1_reg1 + index + 7, mode1_reg1 + index + 8)] = {}
    plant[mode1_reg1 + index +
          7]['transitions'][(mode1_reg1 + index + 7, mode1_reg1 + index + 8)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index +
          7]['transitions'][(mode1_reg1 + index + 7, mode1_reg1 + index + 8)]['reset1'] = []

    plant[mode1_reg1 + index + 8] = {}
    plant[mode1_reg1 + index + 8]['name'] = '_div_0_0_'
    plant[mode1_reg1 + index + 8]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 8]['dynamics'] = {}
    plant[mode1_reg1 + index + 8]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg1 + index + 8]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 8]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 8]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 8]['transitions'] = {}
    plant[mode1_reg1 + index +
          8]['transitions'][(mode1_reg1 + index + 8, mode1_reg1 + index + 11)] = {}
    plant[mode1_reg1 + index +
          8]['transitions'][(mode1_reg1 + index + 8, mode1_reg1 + index + 11)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index + 8]['transitions'][(mode1_reg1 + index + 8, mode1_reg1 + index + 11)]['reset1'] = [
        'clock\' := 0',
        '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := angle * (' + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1)']

    # compute cos(angle) (theta_l, LIDAR_RANGE]
    plant[mode1_reg1 + index + 4] = {}
    plant[mode1_reg1 + index + 4]['name'] = 'left_wall_'
    plant[mode1_reg1 + index + 4]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 4]['dynamics'] = {}
    plant[mode1_reg1 + index + 4]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 4]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 4]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 4]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 4]['transitions'] = {}
    plant[mode1_reg1 + index +
          4]['transitions'][(mode1_reg1 + index + 4, mode1_reg1 + index + 9)] = {}
    plant[mode1_reg1 + index +
          4]['transitions'][(mode1_reg1 + index + 4, mode1_reg1 + index + 9)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index +
          4]['transitions'][(mode1_reg1 + index + 4, mode1_reg1 + index + 9)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg1 + index + 9] = {}
    plant[mode1_reg1 + index + 9]['name'] = '_cos_0_0_'
    plant[mode1_reg1 + index + 9]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 9]['dynamics'] = {}
    plant[mode1_reg1 + index + 9]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg1 + index + 9]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 9]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 9]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 9]['transitions'] = {}
    plant[mode1_reg1 + index +
          9]['transitions'][(mode1_reg1 + index + 9, mode1_reg1 + index + 10)] = {}
    plant[mode1_reg1 + index +
          9]['transitions'][(mode1_reg1 + index + 9, mode1_reg1 + index + 10)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index +
          9]['transitions'][(mode1_reg1 + index + 9, mode1_reg1 + index + 10)]['reset1'] = []

    plant[mode1_reg1 + index + 10] = {}
    plant[mode1_reg1 + index + 10]['name'] = '_div_0_0_'
    plant[mode1_reg1 + index + 10]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 10]['dynamics'] = {}
    plant[mode1_reg1 + index + 10]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg1 + index + 10]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 10]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 10]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 10]['transitions'] = {}
    plant[mode1_reg1 + index +
          10]['transitions'][(mode1_reg1 + index + 10, mode1_reg1 + index + 11)] = {}
    plant[mode1_reg1 + index +
          10]['transitions'][(mode1_reg1 + index + 10, mode1_reg1 + index + 11)]['guards1'] = ['clock = 0']
    plant[mode1_reg1 + index + 10]['transitions'][(mode1_reg1 + index + 10, mode1_reg1 + index + 11)]['reset1'] = [
        'clock\' := 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := angle * y1']

    # last mode
    plant[mode1_reg1 + index + 11] = {}
    plant[mode1_reg1 + index + 11]['name'] = ''
    plant[mode1_reg1 + index + 11]['odetype'] = 'lti ode'
    plant[mode1_reg1 + index + 11]['dynamics'] = {}
    plant[mode1_reg1 + index + 11]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg1 + index + 11]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg1 + index + 11]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg1 + index + 11]['invariants'] = ['clock <= 0']
    plant[mode1_reg1 + index + 11]['transitions'] = {}

    # if last ray, need to transition to m0 (in composed transitions)
    # if nextAngle == LIDAR_RANGE + LIDAR_OFFSET:
    if curRay == NUM_RAYS:
        break

    plant[mode1_reg1 + index +
          11]['transitions'][(mode1_reg1 + index + 11, mode1_reg1 + index + 12)] = {}

    plant[mode1_reg1 + index + 11]['transitions'][(mode1_reg1 + index + 11, mode1_reg1 + index + 12)]['guards1'] =\
        ['clock = 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) +
         ' >= ' + str(LIDAR_MAX_DISTANCE)]
    plant[mode1_reg1 + index + 11]['transitions'][(mode1_reg1 + index + 11, mode1_reg1 + index + 12)]['reset1'] =\
        ['_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) + '\' := ' + str(LIDAR_MAX_DISTANCE),
         'angle\' := y4 + ' + str(nextAngle),
         'temp1\' := y4 + ' + str(nextAngle) + ' - theta_l',
         'temp2\' := y4 + ' + str(nextAngle) + ' - theta_r']

    plant[mode1_reg1 + index + 11]['transitions'][(mode1_reg1 + index + 11, mode1_reg1 + index + 12)]['guards2'] =\
        ['clock = 0', '_f' + str((index + NUM_MODES_REG1)//NUM_MODES_REG1) +
         ' <= ' + str(LIDAR_MAX_DISTANCE)]
    plant[mode1_reg1 + index + 11]['transitions'][(mode1_reg1 + index + 11, mode1_reg1 + index + 12)]['reset2'] =\
        ['angle\' := y4 + ' + str(nextAngle),
         'temp1\' := y4 + ' + str(nextAngle) + ' - theta_l',
         'temp2\' := y4 + ' + str(nextAngle) + ' - theta_r']

    nextAngle += LIDAR_OFFSET
    index += NUM_MODES_REG1
    curRay += 1

# Region 2
nextAngle = -LIDAR_RANGE + LIDAR_OFFSET
curRay = 1
index = 0

# while nextAngle <= LIDAR_RANGE + LIDAR_OFFSET:
while curRay <= NUM_RAYS:

    # first mode
    plant[mode1_reg2 + index] = {}
    plant[mode1_reg2 + index]['name'] = ''
    plant[mode1_reg2 + index]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index]['dynamics'] = {}
    plant[mode1_reg2 + index]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index]['transitions'] = {}

    # transitions to convert angle to (-180, 180), i.e., (-pi, pi)
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 + index, mode1_reg2 + index + 1)] = {}
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 + index, mode1_reg2 + index + 1)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(np.pi)]
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 + index, mode1_reg2 + index + 1)]['reset1'] =\
        ['angle\' := angle - ' + str(2 * np.pi),
         'temp1\' := temp1 - ' + str(2 * np.pi),
         'temp2\' := temp2 - ' + str(2 * np.pi)]
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 + index, mode1_reg2 + index + 1)]['guards2'] =\
        ['clock = 0', 'angle <= ' + str(-np.pi)]
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 + index, mode1_reg2 + index + 1)]['reset2'] =\
        ['angle\' := angle + ' + str(2 * np.pi),
         'temp1\' := temp1 + ' + str(2 * np.pi),
         'temp2\' := temp2 + ' + str(2 * np.pi)]
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 + index, mode1_reg2 + index + 1)]['guards3'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi)]
    plant[mode1_reg2 + index]['transitions'][(mode1_reg2 +
                                              index, mode1_reg2 + index + 1)]['reset3'] = []

    # transition to correct wall
    plant[mode1_reg2 + index + 1] = {}
    plant[mode1_reg2 + index + 1]['name'] = ''
    plant[mode1_reg2 + index + 1]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 1]['dynamics'] = {}
    plant[mode1_reg2 + index + 1]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 1]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 1]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 1]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 1]['transitions'] = {}

    # [-LIDAR_RANGE, theta_r]
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 2)] = {}
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 2)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi),
         'angle <= ' + str(np.pi),
         'temp2 <= 0']
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 2)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := 0',
         'angle\' := ' + str(np.pi / 2) + ' + angle']

    # (theta_r, TURN_ANGLE)
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 3)] = {}
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 3)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi),
         'temp2 >= 0',
         'angle <= ' + str(TURN_ANGLE)]
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 3)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := 0',
         'angle\' := ' + str(np.pi/2 - TURN_ANGLE) + ' + angle']

    # (TURN_ANGLE, theta_l]
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 4)] = {}
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 4)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(TURN_ANGLE),
         'angle <= ' + str(np.pi),
         'temp1 <= 0']
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 4)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := 0',
         'angle\' := ' + str(np.pi/2 + TURN_ANGLE) + ' - angle']

    # (theta_l, LIDAR_RANGE]
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 5)] = {}
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 5)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi),
         'angle <= ' + str(np.pi),
         'temp1 >= 0']
    plant[mode1_reg2 + index + 1]['transitions'][(mode1_reg2 + index + 1, mode1_reg2 + index + 5)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := 0',
         'angle\' := ' + str(np.pi/2) + ' - angle']

    # compute cos(angle) [-LIDAR_RANGE, theta_r]
    plant[mode1_reg2 + index + 2] = {}
    plant[mode1_reg2 + index + 2]['name'] = ''
    plant[mode1_reg2 + index + 2]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 2]['dynamics'] = {}
    plant[mode1_reg2 + index + 2]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 2]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 2]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 2]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 2]['transitions'] = {}
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 2, mode1_reg2 + index + 6)] = {}
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 2, mode1_reg2 + index + 6)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index +
          1]['transitions'][(mode1_reg2 + index + 2, mode1_reg2 + index + 6)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg2 + index + 6] = {}
    plant[mode1_reg2 + index + 6]['name'] = '_cos_0_0_'
    plant[mode1_reg2 + index + 6]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 6]['dynamics'] = {}
    plant[mode1_reg2 + index + 6]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 6]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 6]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 6]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 6]['transitions'] = {}
    plant[mode1_reg2 + index +
          6]['transitions'][(mode1_reg2 + index + 6, mode1_reg2 + index + 7)] = {}
    plant[mode1_reg2 + index +
          6]['transitions'][(mode1_reg2 + index + 6, mode1_reg2 + index + 7)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index + 6]['transitions'][(mode1_reg2 + index + 6, mode1_reg2 + index + 7)]['reset1'] =\
                                 ['temp1\' := angle * ' +
                                     str(LIDAR_MAX_DISTANCE) + ' - ' + str(HALLWAY_WIDTH) + ' + y1']

    plant[mode1_reg2 + index + 7] = {}
    plant[mode1_reg2 + index + 7]['name'] = '_div_0_0_'
    plant[mode1_reg2 + index + 7]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 7]['dynamics'] = {}
    plant[mode1_reg2 + index + 7]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 7]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 7]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 7]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 7]['transitions'] = {}
    plant[mode1_reg2 + index +
          7]['transitions'][(mode1_reg2 + index + 7, mode1_reg2 + index + 14)] = {}
    plant[mode1_reg2 + index + 7]['transitions'][(mode1_reg2 + index + 7,
                                                  mode1_reg2 + index + 14)]['guards1'] = ['clock = 0', ' temp1 >= 0']
    plant[mode1_reg2 + index + 7]['transitions'][(mode1_reg2 + index + 7, mode1_reg2 + index + 14)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := (' +
         str(HALLWAY_WIDTH) + ' - y1) * angle']
    plant[mode1_reg2 + index + 7]['transitions'][(mode1_reg2 + index + 7,
                                                  mode1_reg2 + index + 14)]['guards2'] = ['clock = 0', ' temp1 <= 0']
    plant[mode1_reg2 + index + 7]['transitions'][(mode1_reg2 + index + 7, mode1_reg2 + index + 14)]['reset2'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2) //
                                    NUM_MODES_REG2) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # compute cos(angle) (theta_r, -90)
    plant[mode1_reg2 + index + 3] = {}
    plant[mode1_reg2 + index + 3]['name'] = ''
    plant[mode1_reg2 + index + 3]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 3]['dynamics'] = {}
    plant[mode1_reg2 + index + 3]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 3]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 3]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 3]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 3]['transitions'] = {}
    plant[mode1_reg2 + index +
          2]['transitions'][(mode1_reg2 + index + 3, mode1_reg2 + index + 8)] = {}
    plant[mode1_reg2 + index +
          2]['transitions'][(mode1_reg2 + index + 3, mode1_reg2 + index + 8)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index +
          2]['transitions'][(mode1_reg2 + index + 3, mode1_reg2 + index + 8)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg2 + index + 8] = {}
    plant[mode1_reg2 + index + 8]['name'] = '_cos_0_0_'
    plant[mode1_reg2 + index + 8]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 8]['dynamics'] = {}
    plant[mode1_reg2 + index + 8]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 8]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 8]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 8]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 8]['transitions'] = {}
    plant[mode1_reg2 + index +
          8]['transitions'][(mode1_reg2 + index + 8, mode1_reg2 + index + 9)] = {}
    plant[mode1_reg2 + index +
          8]['transitions'][(mode1_reg2 + index + 8, mode1_reg2 + index + 9)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index + 8]['transitions'][(mode1_reg2 + index + 8, mode1_reg2 + index + 9)]['reset1'] =\
        ['temp1\' := angle * ' + str(LIDAR_MAX_DISTANCE) + ' - ' + str(HALLWAY_WIDTH) + ' + '
         + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1']

    plant[mode1_reg2 + index + 9] = {}
    plant[mode1_reg2 + index + 9]['name'] = '_div_0_0_'
    plant[mode1_reg2 + index + 9]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 9]['dynamics'] = {}
    plant[mode1_reg2 + index + 9]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 9]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 9]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 9]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 9]['transitions'] = {}
    plant[mode1_reg2 + index +
          9]['transitions'][(mode1_reg2 + index + 9, mode1_reg2 + index + 14)] = {}
    plant[mode1_reg2 + index + 9]['transitions'][(mode1_reg2 + index + 9,
                                                  mode1_reg2 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg2 + index + 9]['transitions'][(mode1_reg2 + index + 9, mode1_reg2 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) +
         '\' := angle * (' + str(HALLWAY_WIDTH) + ' - ' + str(SIN_CORNER) + ' * y2 + ' + str(COS_CORNER) + ' * y1)']
    plant[mode1_reg2 + index + 9]['transitions'][(mode1_reg2 + index + 9,
                                                  mode1_reg2 + index + 14)]['guards2'] = ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg2 + index + 9]['transitions'][(mode1_reg2 + index + 9, mode1_reg2 + index + 14)]['reset2'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2) //
                                    NUM_MODES_REG2) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # compute cos(angle) (-90, theta_l]
    plant[mode1_reg2 + index + 4] = {}
    plant[mode1_reg2 + index + 4]['name'] = ''
    plant[mode1_reg2 + index + 4]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 4]['dynamics'] = {}
    plant[mode1_reg2 + index + 4]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 4]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 4]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 4]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 4]['transitions'] = {}
    plant[mode1_reg2 + index +
          4]['transitions'][(mode1_reg2 + index + 4, mode1_reg2 + index + 10)] = {}
    plant[mode1_reg2 + index +
          4]['transitions'][(mode1_reg2 + index + 4, mode1_reg2 + index + 10)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index +
          4]['transitions'][(mode1_reg2 + index + 4, mode1_reg2 + index + 10)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg2 + index + 10] = {}
    plant[mode1_reg2 + index + 10]['name'] = '_cos_0_0_'
    plant[mode1_reg2 + index + 10]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 10]['dynamics'] = {}
    plant[mode1_reg2 + index + 10]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 10]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 10]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 10]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 10]['transitions'] = {}
    plant[mode1_reg2 + index +
          10]['transitions'][(mode1_reg2 + index + 10, mode1_reg2 + index + 11)] = {}
    plant[mode1_reg2 + index +
          10]['transitions'][(mode1_reg2 + index + 10, mode1_reg2 + index + 11)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index + 10]['transitions'][(mode1_reg2 + index + 10, mode1_reg2 + index + 11)]['reset1'] =\
        ['temp1\' := angle * ' + str(LIDAR_MAX_DISTANCE) + ' - ' +
         str(SIN_CORNER) + ' * y2 + ' + str(COS_CORNER) + ' * y1']

    plant[mode1_reg2 + index + 11] = {}
    plant[mode1_reg2 + index + 11]['name'] = '_div_0_0_'
    plant[mode1_reg2 + index + 11]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 11]['dynamics'] = {}
    plant[mode1_reg2 + index + 11]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 11]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 11]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 11]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 11]['transitions'] = {}
    plant[mode1_reg2 + index +
          11]['transitions'][(mode1_reg2 + index + 11, mode1_reg2 + index + 14)] = {}
    plant[mode1_reg2 + index + 11]['transitions'][(
        mode1_reg2 + index + 11, mode1_reg2 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg2 + index + 11]['transitions'][(mode1_reg2 + index + 11, mode1_reg2 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := angle * (' + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1)']
    plant[mode1_reg2 + index + 11]['transitions'][(
        mode1_reg2 + index + 11, mode1_reg2 + index + 14)]['guards2'] = ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg2 + index + 11]['transitions'][(mode1_reg2 + index + 11, mode1_reg2 + index + 14)]['reset2'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2) //
                                    NUM_MODES_REG2) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # compute cos(angle) (theta_l, 180]
    plant[mode1_reg2 + index + 5] = {}
    plant[mode1_reg2 + index + 5]['name'] = ''
    plant[mode1_reg2 + index + 5]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 5]['dynamics'] = {}
    plant[mode1_reg2 + index + 5]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 5]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 5]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 5]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 5]['transitions'] = {}
    plant[mode1_reg2 + index +
          5]['transitions'][(mode1_reg2 + index + 5, mode1_reg2 + index + 12)] = {}
    plant[mode1_reg2 + index +
          5]['transitions'][(mode1_reg2 + index + 5, mode1_reg2 + index + 12)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index +
          5]['transitions'][(mode1_reg2 + index + 5, mode1_reg2 + index + 12)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg2 + index + 12] = {}
    plant[mode1_reg2 + index + 12]['name'] = '_cos_0_0_'
    plant[mode1_reg2 + index + 12]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 12]['dynamics'] = {}
    plant[mode1_reg2 + index + 12]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 12]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 12]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 12]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 12]['transitions'] = {}
    plant[mode1_reg2 + index +
          12]['transitions'][(mode1_reg2 + index + 12, mode1_reg2 + index + 13)] = {}
    plant[mode1_reg2 + index +
          12]['transitions'][(mode1_reg2 + index + 12, mode1_reg2 + index + 13)]['guards1'] = ['clock = 0']
    plant[mode1_reg2 + index + 12]['transitions'][(mode1_reg2 + index + 12, mode1_reg2 + index + 13)]['reset1'] =\
                                                 ['temp1\' := angle * ' +
                                                     str(LIDAR_MAX_DISTANCE) + ' - y1']

    plant[mode1_reg2 + index + 13] = {}
    plant[mode1_reg2 + index + 13]['name'] = '_div_0_0_'
    plant[mode1_reg2 + index + 13]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 13]['dynamics'] = {}
    plant[mode1_reg2 + index + 13]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg2 + index + 13]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 13]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 13]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 13]['transitions'] = {}
    plant[mode1_reg2 + index +
          13]['transitions'][(mode1_reg2 + index + 13, mode1_reg2 + index + 14)] = {}
    plant[mode1_reg2 + index + 13]['transitions'][(
        mode1_reg2 + index + 13, mode1_reg2 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg2 + index + 13]['transitions'][(mode1_reg2 + index + 13, mode1_reg2 + index + 14)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2)//NUM_MODES_REG2) + '\' := y1 * angle']
    plant[mode1_reg2 + index + 13]['transitions'][(mode1_reg2 + index + 13, mode1_reg2 + index + 14)]['guards2'] =\
                                                 ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg2 + index + 13]['transitions'][(mode1_reg2 + index + 13, mode1_reg2 + index + 14)]['reset2'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG2) //
                                    NUM_MODES_REG2) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # last mode
    plant[mode1_reg2 + index + 14] = {}
    plant[mode1_reg2 + index + 14]['name'] = ''
    plant[mode1_reg2 + index + 14]['odetype'] = 'lti ode'
    plant[mode1_reg2 + index + 14]['dynamics'] = {}
    plant[mode1_reg2 + index + 14]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg2 + index + 14]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg2 + index + 14]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg2 + index + 14]['invariants'] = ['clock <= 0']
    plant[mode1_reg2 + index + 14]['transitions'] = {}

    # if last ray, need to transition to m0 (in composed transitions)
    # if nextAngle == LIDAR_RANGE + LIDAR_OFFSET:
    if curRay == NUM_RAYS:
        break

    plant[mode1_reg2 + index +
          14]['transitions'][(mode1_reg2 + index + 14, mode1_reg2 + index + NUM_MODES_REG2)] = {}
    plant[mode1_reg2 + index + 14]['transitions'][(mode1_reg2 + index + 14, mode1_reg2 + index + NUM_MODES_REG2)]['guards1'] =\
        ['clock = 0']
    plant[mode1_reg2 + index + 14]['transitions'][(mode1_reg2 + index + 14, mode1_reg2 + index + NUM_MODES_REG2)]['reset1'] =\
        ['angle\' := y4 + ' + str(nextAngle),
         'temp1\' := y4 + ' + str(nextAngle) + ' - theta_l',
         'temp2\' := y4 + ' + str(nextAngle) + ' - theta_r']

    nextAngle += LIDAR_OFFSET
    index += NUM_MODES_REG2
    curRay += 1

# Region 3
nextAngle = -LIDAR_RANGE + LIDAR_OFFSET
index = 0
curRay = 1

# while nextAngle <= LIDAR_RANGE + LIDAR_OFFSET:
while curRay <= NUM_RAYS:

    # first mode
    plant[mode1_reg3 + index] = {}
    plant[mode1_reg3 + index]['name'] = ''
    plant[mode1_reg3 + index]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index]['dynamics'] = {}
    plant[mode1_reg3 + index]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index]['dynamics']['_f' + str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index]['transitions'] = {}

    # transitions to convert angle to (-180, 180), i.e., (-pi, pi)
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 + index, mode1_reg3 + index + 1)] = {}
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 + index, mode1_reg3 + index + 1)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(np.pi)]
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 + index, mode1_reg3 + index + 1)]['reset1'] =\
        ['angle\' := angle - ' + str(2 * np.pi),
         'temp1\' := temp1 - ' + str(2 * np.pi),
         'temp2\' := temp2 - ' + str(2 * np.pi)]
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 + index, mode1_reg3 + index + 1)]['guards2'] =\
        ['clock = 0', 'angle <= ' + str(-np.pi)]
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 + index, mode1_reg3 + index + 1)]['reset2'] =\
        ['angle\' := angle + ' + str(2 * np.pi),
         'temp1\' := temp1 + ' + str(2 * np.pi),
         'temp2\' := temp2 + ' + str(2 * np.pi)]
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 + index, mode1_reg3 + index + 1)]['guards3'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi)]
    plant[mode1_reg3 + index]['transitions'][(mode1_reg3 +
                                              index, mode1_reg3 + index + 1)]['reset3'] = []

    # transition to correct wall
    plant[mode1_reg3 + index + 1] = {}
    plant[mode1_reg3 + index + 1]['name'] = ''
    plant[mode1_reg3 + index + 1]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 1]['dynamics'] = {}
    plant[mode1_reg3 + index + 1]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 1]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 1]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 1]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 1]['transitions'] = {}

    # [-LIDAR_RANGE, TURN_ANGLE - 0.6)

    # NB: the hardcoded number means that those rays
    # cannot possibly hit the wall within 10m and for 0.15m clearance;
    # tighter bounds can be obtained for 5m LIDAR range and 0.3m
    # clearance

    plant[mode1_reg3 + index +
          1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 2)] = {}
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 2)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi),
         'angle <= ' + str(np.pi), 'angle <= ' + str(TURN_ANGLE - 0.6 * np.pi / 180)]
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 2)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := 0',
         'angle\' := ' + str(np.pi/2 - TURN_ANGLE) + ' + angle']

    # [TURN_ANGLE-0.6, TURN_ANGLE + 0.6]
    plant[mode1_reg3 + index +
          1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 14)] = {}
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 14)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(TURN_ANGLE - 0.6 * np.pi / 180),
         'angle <= ' + str(TURN_ANGLE + 0.6 * np.pi / 180)]
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) +
         '\' := ' + str(LIDAR_MAX_DISTANCE),
         'angle\' := angle']

    # (TURN_ANGLE + 0.6, theta_l]
    plant[mode1_reg3 + index +
          1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 3)] = {}
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 3)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi), 'angle <= ' + str(np.pi),
         'temp1 <= 0', 'angle >= ' + str(TURN_ANGLE + 0.6 * np.pi / 180)]
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 3)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := 0',
         'angle\' := ' + str(np.pi/2 + TURN_ANGLE) + ' - angle']

    # (theta_l, theta_r]
    plant[mode1_reg3 + index +
          1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 4)] = {}
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 4)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi),
         'angle <= ' + str(np.pi), 'temp1 >= 0', 'temp2 <= 0']
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 4)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := 0',
         'angle\' := ' + str(np.pi/2) + ' - angle']

    # (theta_r, LIDAR_RANGE]
    plant[mode1_reg3 + index +
          1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 5)] = {}
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 5)]['guards1'] =\
        ['clock = 0', 'angle >= ' + str(-np.pi),
         'angle <= ' + str(np.pi), 'temp2 >= 0']
    plant[mode1_reg3 + index + 1]['transitions'][(mode1_reg3 + index + 1, mode1_reg3 + index + 5)]['reset1'] =\
        ['clock\' := 0', '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := 0',
         'angle\' := ' + str(np.pi/2 - TURN_ANGLE) + ' + angle']

    # compute cos(angle) [-LIDAR_RANGE, TURN_ANGLE)
    plant[mode1_reg3 + index + 2] = {}
    plant[mode1_reg3 + index + 2]['name'] = ''
    plant[mode1_reg3 + index + 2]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 2]['dynamics'] = {}
    plant[mode1_reg3 + index + 2]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 2]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 2]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 2]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 2]['transitions'] = {}
    plant[mode1_reg3 + index +
          2]['transitions'][(mode1_reg3 + index + 2, mode1_reg3 + index + 6)] = {}
    plant[mode1_reg3 + index +
          2]['transitions'][(mode1_reg3 + index + 2, mode1_reg3 + index + 6)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index +
          2]['transitions'][(mode1_reg3 + index + 2, mode1_reg3 + index + 6)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg3 + index + 6] = {}
    plant[mode1_reg3 + index + 6]['name'] = '_cos_0_0_'
    plant[mode1_reg3 + index + 6]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 6]['dynamics'] = {}
    plant[mode1_reg3 + index + 6]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 6]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 6]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 6]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 6]['transitions'] = {}
    plant[mode1_reg3 + index +
          6]['transitions'][(mode1_reg3 + index + 6, mode1_reg3 + index + 7)] = {}
    plant[mode1_reg3 + index +
          6]['transitions'][(mode1_reg3 + index + 6, mode1_reg3 + index + 7)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index + 6]['transitions'][(mode1_reg3 + index + 6, mode1_reg3 + index + 7)]['reset1'] =\
        ['temp1\' := angle * ' + str(LIDAR_MAX_DISTANCE) + ' - ' + str(HALLWAY_WIDTH) + ' + '
         + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1']

    plant[mode1_reg3 + index + 7] = {}
    plant[mode1_reg3 + index + 7]['name'] = '_div_0_0_'
    plant[mode1_reg3 + index + 7]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 7]['dynamics'] = {}
    plant[mode1_reg3 + index + 7]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 7]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 7]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 7]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 7]['transitions'] = {}
    plant[mode1_reg3 + index +
          7]['transitions'][(mode1_reg3 + index + 7, mode1_reg3 + index + 14)] = {}
    plant[mode1_reg3 + index + 7]['transitions'][(mode1_reg3 + index + 7,
                                                  mode1_reg3 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg3 + index + 7]['transitions'][(mode1_reg3 + index + 7, mode1_reg3 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) +
         '\' := angle * (' + str(HALLWAY_WIDTH) + ' - ' + str(SIN_CORNER) + ' * y2 + ' + str(COS_CORNER) + ' * y1)']
    plant[mode1_reg3 + index + 7]['transitions'][(mode1_reg3 + index + 7,
                                                  mode1_reg3 + index + 14)]['guards2'] = ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg3 + index + 7]['transitions'][(mode1_reg3 + index + 7, mode1_reg3 + index + 14)]['reset2'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # compute cos(angle) (TURN_ANGLE, theta_l]
    plant[mode1_reg3 + index + 3] = {}
    plant[mode1_reg3 + index + 3]['name'] = ''
    plant[mode1_reg3 + index + 3]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 3]['dynamics'] = {}
    plant[mode1_reg3 + index + 3]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 3]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 3]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 3]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 3]['transitions'] = {}
    plant[mode1_reg3 + index +
          3]['transitions'][(mode1_reg3 + index + 3, mode1_reg3 + index + 8)] = {}
    plant[mode1_reg3 + index +
          3]['transitions'][(mode1_reg3 + index + 3, mode1_reg3 + index + 8)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index +
          3]['transitions'][(mode1_reg3 + index + 3, mode1_reg3 + index + 8)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg3 + index + 8] = {}
    plant[mode1_reg3 + index + 8]['name'] = '_cos_0_0_'
    plant[mode1_reg3 + index + 8]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 8]['dynamics'] = {}
    plant[mode1_reg3 + index + 8]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 8]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 8]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 8]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 8]['transitions'] = {}
    plant[mode1_reg3 + index +
          8]['transitions'][(mode1_reg3 + index + 8, mode1_reg3 + index + 9)] = {}
    plant[mode1_reg3 + index +
          8]['transitions'][(mode1_reg3 + index + 8, mode1_reg3 + index + 9)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index + 8]['transitions'][(mode1_reg3 + index + 8, mode1_reg3 + index + 9)]['reset1'] =\
        ['temp1\' := angle * ' + str(LIDAR_MAX_DISTANCE) + ' - ' +
         str(SIN_CORNER) + ' * y2 + ' + str(COS_CORNER) + ' * y1']

    plant[mode1_reg3 + index + 9] = {}
    plant[mode1_reg3 + index + 9]['name'] = '_div_0_0_'
    plant[mode1_reg3 + index + 9]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 9]['dynamics'] = {}
    plant[mode1_reg3 + index + 9]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 9]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 9]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 9]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 9]['transitions'] = {}
    plant[mode1_reg3 + index +
          9]['transitions'][(mode1_reg3 + index + 9, mode1_reg3 + index + 14)] = {}
    plant[mode1_reg3 + index + 9]['transitions'][(mode1_reg3 + index + 9,
                                                  mode1_reg3 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg3 + index + 9]['transitions'][(mode1_reg3 + index + 9, mode1_reg3 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := angle * (' + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1)']
    plant[mode1_reg3 + index + 9]['transitions'][(mode1_reg3 + index + 9,
                                                  mode1_reg3 + index + 14)]['guards2'] = ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg3 + index + 9]['transitions'][(mode1_reg3 + index + 9, mode1_reg3 + index + 14)]['reset2'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # compute cos(angle) (theta_l, theta_r]
    plant[mode1_reg3 + index + 4] = {}
    plant[mode1_reg3 + index + 4]['name'] = ''
    plant[mode1_reg3 + index + 4]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 4]['dynamics'] = {}
    plant[mode1_reg3 + index + 4]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 4]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 4]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 4]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 4]['transitions'] = {}
    plant[mode1_reg3 + index +
          4]['transitions'][(mode1_reg3 + index + 4, mode1_reg3 + index + 10)] = {}
    plant[mode1_reg3 + index +
          4]['transitions'][(mode1_reg3 + index + 4, mode1_reg3 + index + 10)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index +
          4]['transitions'][(mode1_reg3 + index + 4, mode1_reg3 + index + 10)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg3 + index + 10] = {}
    plant[mode1_reg3 + index + 10]['name'] = '_cos_0_0_'
    plant[mode1_reg3 + index + 10]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 10]['dynamics'] = {}
    plant[mode1_reg3 + index + 10]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 10]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 10]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 10]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 10]['transitions'] = {}
    plant[mode1_reg3 + index +
          10]['transitions'][(mode1_reg3 + index + 10, mode1_reg3 + index + 11)] = {}
    plant[mode1_reg3 + index +
          10]['transitions'][(mode1_reg3 + index + 10, mode1_reg3 + index + 11)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index + 10]['transitions'][(mode1_reg3 + index + 10, mode1_reg3 + index + 11)]['reset1'] =\
                                                 ['temp1\' := angle * ' +
                                                     str(LIDAR_MAX_DISTANCE) + ' - y1']

    plant[mode1_reg3 + index + 11] = {}
    plant[mode1_reg3 + index + 11]['name'] = '_div_0_0_'
    plant[mode1_reg3 + index + 11]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 11]['dynamics'] = {}
    plant[mode1_reg3 + index + 11]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 11]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 11]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 11]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 11]['transitions'] = {}
    plant[mode1_reg3 + index +
          11]['transitions'][(mode1_reg3 + index + 11, mode1_reg3 + index + 14)] = {}
    plant[mode1_reg3 + index + 11]['transitions'][(
        mode1_reg3 + index + 11, mode1_reg3 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg3 + index + 11]['transitions'][(mode1_reg3 + index + 11, mode1_reg3 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := y1 * angle']
    plant[mode1_reg3 + index + 11]['transitions'][(
        mode1_reg3 + index + 11, mode1_reg3 + index + 14)]['guards2'] = ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg3 + index + 11]['transitions'][(mode1_reg3 + index + 11, mode1_reg3 + index + 14)]['reset2'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # compute cos(angle) (theta_r, LIDAR_RANGE)
    plant[mode1_reg3 + index + 5] = {}
    plant[mode1_reg3 + index + 5]['name'] = ''
    plant[mode1_reg3 + index + 5]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 5]['dynamics'] = {}
    plant[mode1_reg3 + index + 5]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 5]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 5]['dynamics']['_f' +
                                                  str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 5]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 5]['transitions'] = {}
    plant[mode1_reg3 + index +
          5]['transitions'][(mode1_reg3 + index + 5, mode1_reg3 + index + 12)] = {}
    plant[mode1_reg3 + index +
          5]['transitions'][(mode1_reg3 + index + 5, mode1_reg3 + index + 12)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index +
          5]['transitions'][(mode1_reg3 + index + 5, mode1_reg3 + index + 12)]['reset1'] = ['clock\' := 0']

    #
    plant[mode1_reg3 + index + 12] = {}
    plant[mode1_reg3 + index + 12]['name'] = '_cos_0_0_'
    plant[mode1_reg3 + index + 12]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 12]['dynamics'] = {}
    plant[mode1_reg3 + index + 12]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 12]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 12]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 12]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 12]['transitions'] = {}
    plant[mode1_reg3 + index +
          12]['transitions'][(mode1_reg3 + index + 12, mode1_reg3 + index + 13)] = {}
    plant[mode1_reg3 + index +
          12]['transitions'][(mode1_reg3 + index + 12, mode1_reg3 + index + 13)]['guards1'] = ['clock = 0']
    plant[mode1_reg3 + index + 12]['transitions'][(mode1_reg3 + index + 12, mode1_reg3 + index + 13)]['reset1'] =\
        ['temp1\' := angle * ' + str(LIDAR_MAX_DISTANCE) + ' - ' + str(HALLWAY_WIDTH) + ' + '
         + str(SIN_CORNER) + ' * y2 - ' + str(COS_CORNER) + ' * y1']

    plant[mode1_reg3 + index + 13] = {}
    plant[mode1_reg3 + index + 13]['name'] = '_div_0_0_'
    plant[mode1_reg3 + index + 13]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 13]['dynamics'] = {}
    plant[mode1_reg3 + index + 13]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['temp2'] = 'temp2\' = 0\n'
    plant[mode1_reg3 + index + 13]['dynamics']['angle'] = 'angle\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 13]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 13]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 13]['transitions'] = {}
    plant[mode1_reg3 + index +
          13]['transitions'][(mode1_reg3 + index + 13, mode1_reg3 + index + 14)] = {}
    plant[mode1_reg3 + index + 13]['transitions'][(
        mode1_reg3 + index + 13, mode1_reg3 + index + 14)]['guards1'] = ['clock = 0', 'temp1 >= 0']
    plant[mode1_reg3 + index + 13]['transitions'][(mode1_reg3 + index + 13, mode1_reg3 + index + 14)]['reset1'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) +
         '\' := angle * (' + str(HALLWAY_WIDTH) + ' - ' + str(SIN_CORNER) + ' * y2 + ' + str(COS_CORNER) + ' * y1)']
    plant[mode1_reg3 + index + 13]['transitions'][(
        mode1_reg3 + index + 13, mode1_reg3 + index + 14)]['guards2'] = ['clock = 0', 'temp1 <= 0']
    plant[mode1_reg3 + index + 13]['transitions'][(mode1_reg3 + index + 13, mode1_reg3 + index + 14)]['reset2'] =\
        ['clock\' := 0',
         '_f' + str((index + NUM_MODES_REG3)//NUM_MODES_REG3) + '\' := ' + str(LIDAR_MAX_DISTANCE)]

    # last mode
    plant[mode1_reg3 + index + 14] = {}
    plant[mode1_reg3 + index + 14]['name'] = ''
    plant[mode1_reg3 + index + 14]['odetype'] = 'lti ode'
    plant[mode1_reg3 + index + 14]['dynamics'] = {}
    plant[mode1_reg3 + index + 14]['dynamics']['y1'] = 'y1\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['y2'] = 'y2\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['y3'] = 'y3\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['y4'] = 'y4\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['k'] = 'k\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['u'] = 'u\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['angle'] = 'angle\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['theta_l'] = 'theta_l\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['theta_r'] = 'theta_r\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['temp1'] = 'temp1\' = 0\n'
    plant[mode1_reg3 + index + 14]['dynamics']['temp2'] = 'temp2\' = 0\n'
    for i in range(NUM_RAYS):
        plant[mode1_reg3 + index + 14]['dynamics']['_f' +
                                                   str(i + 1)] = '_f' + str(i + 1) + '\' = 0\n'

    plant[mode1_reg3 + index + 14]['invariants'] = ['clock <= 0']
    plant[mode1_reg3 + index + 14]['transitions'] = {}

    # if last ray, need to transition to m0 (in composed transitions)
    # if nextAngle == LIDAR_RANGE + LIDAR_OFFSET:
    if curRay == NUM_RAYS:
        break

    plant[mode1_reg3 + index +
          14]['transitions'][(mode1_reg3 + index + 14, mode1_reg3 + index + NUM_MODES_REG3)] = {}

    plant[mode1_reg3 + index + 14]['transitions'][(mode1_reg3 + index + 14, mode1_reg3 + index + NUM_MODES_REG3)]['guards1'] =\
        ['clock = 0']
    plant[mode1_reg3 + index + 14]['transitions'][(mode1_reg3 + index + 14, mode1_reg3 + index + NUM_MODES_REG3)]['reset1'] =\
        ['angle\' := y4 + ' + str(nextAngle),
         'temp1\' := y4 + ' + str(nextAngle) + ' - theta_l',
         'temp2\' := y4 + ' + str(nextAngle) + ' - theta_r']

    nextAngle += LIDAR_OFFSET
    index += NUM_MODES_REG3
    curRay += 1

filename = 'dynamics_' + name + str(NUM_RAYS) + '.pickle'

try:
    with open(filename, 'wb') as f:
        pickle.dump(plant, f, pickle.HIGHEST_PROTOCOL)
except Exception as e:
    print('Unable to save data to', filename, ':', e)
