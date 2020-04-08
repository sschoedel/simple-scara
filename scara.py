import pyglet
from pyglet.gl import * # opengl functions
import numpy as np
import math
from circle import circle
from rotationMat import ZTransformMatrix  # pos (3x1), theta(1x3), rads=True/False

window = pyglet.window.Window()
label = pyglet.text.Label('Scara Workspace 2D',color=(130,130,200,255),font_name='Noto Sans JP',font_size=15,x=110,y=window.height-20,anchor_x='center',anchor_y='center')
image = pyglet.resource.image('grid.JPG')

manualMode = False
image.width = window.width
image.height = window.height

# Define link properties
# In mm
link1Len = 150
link2Len = 100

# In kg
link1Mass = 2
link2Mass = 1

# Instantiate arm parameter globals
# In radians
goal1Theta = 0
goal2Theta = 0
goalPos = (0,0)

joint1Theta = 0
joint2Theta = 0

# In rad/s
joint1Speed = 0
joint2Speed = 0

# In Nm
joint1Torque = 0
joint2Torque = 0

thetaThreshold = 0.01
velThreshold = 0.01

framesPerSecond = 30

# Define goaltheta thresholds
cwThreshold_1 = 0
ccwThreshold_1 = np.pi
cwThreshold_2 = -np.pi/2 + 0.5
ccwThreshold_2 = np.pi/2 - 0.5



# Changes xy position from window to coordinate (grid)
def winToCoord(x, y):
    return (x/2.05 + window.width/2, y/2.05 + window.height/2)

# Changes xy position from coordinate (grid) to window
def coordToWin(x, y):
    return ((x - window.width/2) * 2.05, (y - window.height/2) * 2.05)


def pointsInCircle(n=25, r=50):
    return [ winToCoord(math.cos(2*np.pi/n*x)*r, math.sin(2*np.pi/n*x)*r) for x in range(n+1) ]
circlePts = np.array(pointsInCircle(25, link1Len))

# Starting coordinates
joint1 = winToCoord(0,0)
joint2 = winToCoord(0,0)
joint3 = winToCoord(0,0)

# initialize arm vertex list:
arm_vertices = pyglet.graphics.vertex_list(4,
    ('v2f', (joint1[0], joint1[1],
             joint2[0], joint2[1],
             joint2[0], joint2[1],
             joint3[0], joint3[1])),
    ('c3B', (0, 0, 0,
             0, 0, 0,
             0, 0, 0,
             0, 0, 0))
)

@window.event
def on_draw():
    clearScreen()
    drawBackground()
    drawlines()

def clearScreen():
    window.clear()
    glClear(GL_COLOR_BUFFER_BIT)

def drawBackground():
    image.blit(0,0)
    label.draw()

def drawlines():
    arm_vertices.draw(pyglet.gl.GL_LINES)

v = 0
t = 0
buffer = ''
enteringInput = False
@window.event
def on_key_press(symbol, modifiers):
    global joint2
    global joint3
    global buffer
    global enteringInput
    global v
    global t

    inputEntered = False
    if symbol == 112 and enteringInput != True:
        enteringInput = True
        print("ENTERING INPUT")
    elif symbol == 65293:  # Reset buffer and input bools
        enteringInput = False
        inputEntered = True
        determineJointPositions(buffer)
        buffer = ''
    elif enteringInput and symbol >= 97 and symbol <= 122 or symbol == 44 or symbol == 32 or (symbol == 59 and modifiers == 1) or (symbol >= 45 and symbol <= 57):
        buffer = buffer + str(chr(symbol))
        print(buffer)
    elif symbol == 65288 and len(buffer) > 0:
        buffer = buffer[:-1]
        print(buffer)

    if manualMode:
        if symbol == 114:
            v += 10
        elif symbol == 115:
            v -= 10
        if symbol == 116:
            t += 10
        elif symbol == 100:
            t -= 10

        # use v and t as degrees
        joint2Theta = v * 2 * np.pi
        joint3Theta = t * 2 * np.pi

        joint2 = winToCoord(math.cos(joint2Theta/360)*link1Len, math.sin(joint2Theta/360)*link1Len)
        joint2Window = coordToWin(joint2[0],joint2[1])
        joint3 = winToCoord(joint2Window[0] + math.cos(joint3Theta/360)*link2Len, joint2Window[1] + math.sin(joint3Theta/360)*link2Len)

    updateJoint(joint1, joint2, joint3)

goal1Theta = 0
goal2Theta = 0  # in radians counterclockwise from horizontal
def determineJointPositions(input=''):
    global joint1, joint2, joint3
    if input:
        print("Input: {}".format(input))
        parseCommand(input)
    else:
        print("No input")

    # circleLoc = winToCoord(200,200)
    # circle(circleLoc[0], circleLoc[1], 50)

def parseCommand(input):
    global goal1Theta, goal2Theta, goalPos
    global joint1Speed, joint2Speed
    global joint1Torque, joint2Torque
    global joint1Theta, joint2Theta
    # Break input into commands
    commands = input.split(',')
    if commands[0] == 'goalthetas':
        try:
            goal1Theta = float(commands[1].strip())#%(2*np.pi)
            goal2Theta = float(commands[2].strip())#%(2*np.pi)
            print('goal1Theta: {}  goal2Theta: {}'.format(goal1Theta, goal2Theta))
        except:
            print("Unacceptable input angles (check that they are integers)")
    elif commands[0] == 'goalpos':
        try:
            goalPos = (float(commands[1].strip()), float(commands[2].strip()))
            print('goalPos: {}'.format(goalPos))
        except:
            print("Unacceptable input positions (check that they are integers)")
        determineGoalThetas()
    elif commands[0] == 'vels': # for testing
        try:
            joint1Speed = float(commands[1].strip())
            joint2Speed = float(commands[2].strip())
            print("joint vels (1,2): {}, {}".format(joint1Speed, joint2Speed))
        except:
            print("Unacceptable input velocities (check that they are integers)")
    elif commands[0] == 'torques':
        try:
            joint1Torque = float(commands[1].strip())
            joint2Torque = float(commands[2].strip())
            print("joint torques (1,2): {} {}".format(joint1Torque, joint2Torque))
        except:
            print("Unacceptable input torques (check that they are integers)")
    elif commands[0] == 'setthetas':
        try:
            joint1Theta = float(commands[1].strip())
            joint2Theta = float(commands[2].strip())
            print("joint torques (1,2): {} {}".format(joint1Theta, joint2Theta))
            joint2, joint3 = posFromTheta(joint1Theta, joint2Theta)
            updateJoint(j2=joint2, j3=joint3)
        except:
            print("Unacceptable input torques (check that they are integers)")

def workspaceCircle():
    n = ccwThreshold_2 - cwThreshold_2
    return [ winToCoord(link2Len*math.cos(2*np.pi/n*x) + link1Len*math.cos(2*np.pi/n*p), link2Len*math.sin(2*np.pi/n*x) + link1Len*math.sin(2*np.pi/n*p)) for x in range(cwThreshold_2, ccwThreshold_2+1) for p in range(cwThreshold_1, ccwThreshold_1+1) ]


# Draw dense points through workspace
def drawWorkSpace():
    workspacePoints = np.array(workspaceCircle())

# inverse solver given 2D end effector location
    # two solutions for many goal positions - use solution nearest (in terms of thetas) current thetas
def determineGoalThetas():
    global goalPos
    global goal1Theta, goal2Theta

    goalx = goalPos[0]
    goaly = goalPos[1]

    goalDist = math.sqrt(goalx**2 + goaly**2)

    if goalx > 0:
        alpha1 = math.atan(goaly/goalx)%(2*np.pi)
    elif goalx == 0 and abs(goaly) > 0:
        alpha1 = (goaly/abs(goaly) * np.pi/2)%(2*np.pi)
    elif goalx < 0:
        alpha1 = (math.atan(goaly/goalx)+np.pi)%(2*np.pi)
    else: # goalx == 0 and goaly == 0
        alpha1 = 0

    # a^2 = b^2 + c^2 -2bc*cosA
    # a is link2
    # b is link1
    # c in hypotneuse
    # Atheta = first angle
    # Ctheta = second angle
    # goal1Theta = alpha1 + Atheta (in quad 1 this is concave down) OR goal1Theta = alpha1 - Atheta (in quad1 this is concave up)
    # goal2Theta = Ctheta - 180 (quad 1 - concave down) OR goal2Theta = np.pi - Ctheta (quad 2 - concave up)


    if goalx > 0:
        Atheta = math.acos((link1Len**2 + goalDist**2 - link2Len**2)/(2*link1Len*goalDist))
        Ctheta = math.acos((link1Len**2 + link2Len**2 - goalDist**2)/(2*link1Len*link2Len))
        # Assume concave up at first. If goal1Theta is beyond clockwise angle, use concave down
        goal1Theta = alpha1 - Atheta
        goal2Theta = np.pi - Ctheta
        if goal1Theta < cwThreshold_1 or goal1Theta > ccwThreshold_1: # Use concave down if theta1 beyond threshold
            goal1Theta = Atheta + alpha1
            goal2Theta = Ctheta - np.pi
    elif goalx <= 0:
        Atheta = math.acos((link1Len**2 + goalDist**2 - link2Len**2)/(2*link1Len*goalDist))
        Ctheta = math.acos((link1Len**2 + link2Len**2 - goalDist**2)/(2*link1Len*link2Len))
        # Assume concave down at first. If goal1Theta is beyond counterclockwise angle, use concave up
        goal1Theta = Atheta + alpha1
        goal2Theta = Ctheta - np.pi
        if goal1Theta < cwThreshold_1 or goal1Theta > ccwThreshold_1: # Use concave up if theta1 beyond threshold
            goal1Theta = alpha1 - Atheta
            goal2Theta = np.pi - Ctheta
    elif goalx == 0 and goaly == 0:
        goal2Theta = np.pi

    print(goal1Theta, goal2Theta)

# updates frame at framesPerSecond Hz
def update_frame(x, y):
    global joint1Speed, joint2Speed
    global joint2, joint3
    global joint1Torque, joint2Torque
    global joint1Theta, joint2Theta
    global goal1Theta, goal2Theta

    if abs(joint1Speed) > velThreshold or abs(joint2Speed) > velThreshold or abs(goal1Theta - joint1Theta) > thetaThreshold or abs(goal2Theta - joint2Theta) > thetaThreshold:
        calcTorques() # PID
    else:
        joint1Torque = 0
        joint2Torque = 0

    if abs(joint1Torque) > 0 or abs(joint2Torque) > 0:
        joint1Speed, joint2Speed = calculateNextVels(joint1Torque, joint2Torque)
        joint2, joint3 = calculateNextPositions(joint1Speed, joint2Speed)
        updateJoint(j2=joint2, j3=joint3)

prevJoint1Theta = 0
prevJoint2Theta = 0
# TODO: implement option to use values of joint theta between 0 and 2pi only
def calcTorques():
    global goal1Theta, goal2Theta
    global joint1Theta, joint2Theta
    global joint1Torque, joint2Torque
    global framesPerSecond
    global prevJoint1Theta, prevJoint2Theta

    Pbias = 3
    Ibias = 1
    Dbias = 1000

    # TODO: implement this if want to use values of joint theta only between 0 and 2pi
    # if joint2Theta < np.pi and joint2Theta >= 0 and goal2Theta > joint2Theta + np.pi:
    #     joint2ThetaTemp = joint2Theta + 2 * np.pi
    #     print("one")
    # else:
    #     joint2ThetaTemp = joint2Theta
    #     print("two")
    # print(joint2Theta)
    joint2ThetaTemp = joint2Theta
    joint2Diff = (prevJoint2Theta - joint2Theta)/framesPerSecond # should be using a different time step for calculating derivatives and integrals.
                                                             # Use framesPerSecond only for displaying everything
    joint2Torque = Pbias*(goal2Theta - joint2Theta) + 1 * Dbias*(joint2Diff)


    joint1Diff = (prevJoint1Theta - joint1Theta)/framesPerSecond
    joint1Torque = Pbias*(goal1Theta - joint1Theta) + 1 * Dbias*(joint1Diff)

    prevJoint1Theta = joint1Theta
    prevJoint2Theta = joint2ThetaTemp

def holdRotations():
    global joint1Theta, joint2Theta, goal1Theta, goal2Theta
    joint1Theta = goal1Theta
    joint2Theta = goal2Theta

# The following two functions determine joint thetas from either joint speeds or joint torques
# frames per second is 1/the time step, or Hz
def calculateNextVels(j1Torque, j2Torque):
    global joint1Speed, joint2Speed
    global link1Len, link2Len
    global link1Mass, link2Mass
    global framesPerSecond
    global joint1Theta, joint2Theta

    totalMass = link1Mass + link2Mass

    # contribution to the center of mass in each direction for each link is weighted by the mass of each link
    moment1x = ( link1Len/2*math.cos(joint1Theta) * link1Mass + link2Len/2*math.cos(joint2Theta) * link2Mass ) / totalMass
    moment1y = ( link1Len/2*math.sin(joint1Theta) * link1Mass + link2Len/2*math.sin(joint2Theta) * link2Mass ) / totalMass

    moment1Len = math.sqrt(moment1x**2 + moment1y ** 2)/1000 # Center of link and converting from mm to meters
    moment2Len = (link2Len/2)/1000
    accel1 = j1Torque/((link1Mass + link2Mass)*moment1Len)
    accel2 = j2Torque/(link2Mass*moment2Len)

    joint1Speed += accel1/framesPerSecond
    joint2Speed += accel2/framesPerSecond

    return joint1Speed, joint2Speed

def calculateNextPositions(joint2vel, joint3vel):
    global framesPerSecond
    global joint1Theta, joint2Theta

    joint1Theta += joint2vel/framesPerSecond
    joint2Theta += joint3vel/framesPerSecond
    # joint1Theta = joint1Theta%(2*np.pi)
    # joint2Theta = joint2Theta%(2*np.pi)

    return posFromTheta(joint1Theta, joint2Theta)


# force = torque/radius
# accel = force/mass
# accel = torque/(mass*radius) = torque/(mass*momentLen)

# TODO: update link 1 dynamics to include mass and length of second link (calculate location of center of mass and apply torque based on that length and the total mass)

# returns (x,y) in grid coordinates for links 1 and 2
# A note: joint1 refers to the point (x,y) at the end of link 1. joint1Theta refers to the angle of the first link to the horizontal
def posFromTheta(joint1Theta, joint2Theta):
    global link1Len, link2Len
    virtualTheta2 = joint1Theta + joint2Theta
    # matrix from points 0 to 1
    transform01 = ZTransformMatrix(joint1Theta, np.array([0,0,0]))
    # matrix from points 1 to 2
    transform12 = ZTransformMatrix(joint2Theta, np.array([link1Len,0,0]))
    # matrix from points 2 to 3
    transform23 = ZTransformMatrix(0, np.array([link2Len,0,0]))

    # matrices from points 0 to 2 and points 0 to 3. The right-most column is the final joint position
    transform02 = transform01 @ transform12
    transform03 = transform02 @ transform23
    joint2XYZ = transform02[:-1,3] # ignore row four, column four filler value
    joint3XYZ = transform03[:-1,3]
    # TODO: only working in 2 dimensions right now. remove the following lines to keep joints in 3D
    joint2XY = joint2XYZ[:-1]
    joint3XY = joint3XYZ[:-1]

    joint2XY = winToCoord(joint2XY[0], joint2XY[1])
    joint3XY = winToCoord(joint3XY[0], joint3XY[1])

    return joint2XY, joint3XY

# inputs must be converted to grid coordinates before being passed to this function
def updateJoint(j1 = joint1, j2 = joint2, j3 = joint3):
    arm_vertices.vertices[:2] = [j1[0], j1[1]]
    arm_vertices.vertices[2:6] = [j2[0], j2[1], j2[0], j2[1]]
    arm_vertices.vertices[6:8] = [j3[0], j3[1]]

def getEndPosGrid():
    return coordToWin(arm_vertices.vertices[6],arm_vertices.vertices[7])


pyglet.clock.schedule(update_frame, 1/framesPerSecond)
pyglet.app.run()
