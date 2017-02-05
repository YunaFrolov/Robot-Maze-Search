#coding : utf-8
__author__ = 'Yuna Frolov'

# Import Libraries:
import vrep                # V-rep library
import sys
import time                # used to keep track of time
import numpy as np         # array library
import math

# Pre-Allocation

PI = math.pi  # constant pi
GOAL_POSITION = [1.9, -1.84, 0.077]

# orientation of all the sensors - to know which way they face
sensor_loc = np.array(
    [-PI / 2, -50 / 180.0 * PI, -30 / 180.0 * PI, -10 / 180.0 * PI, 10 / 180.0 * PI, 30 / 180.0 * PI,
     50 / 180.0 * PI, PI / 2, PI / 2, 130 / 180.0 * PI, 150 / 180.0 * PI, 170 / 180.0 * PI, -170 / 180.0 * PI,
     -150 / 180.0 * PI, -130 / 180.0 * PI, -PI / 2])


class World(object):
    '''
    robot simulator class to communicate with the simulation environment
    '''
    clientID = 0
    left_motor_handle = 0
    right_motor_handle = 0

    def __init__(self, host='127.0.0.1', portNumber=19999):
        self._host = host
        self._port = portNumber
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)
        if self.clientID != -1:  # check if client connection successful
            print('Connected to remote API server')
        else:
            print('Connection not successful')
            sys.exit('Could not connect')

    def sense(self):
        '''
        implements sensor reading from the simulator
        '''

        errorCode, sensor_handle = vrep.simxGetObjectHandle(self.clientID, 'dr12_proximitySensor_',
                                                                vrep.simx_opmode_oneshot_wait)
        errorCode, wall_sensor_handle = vrep.simxGetObjectHandle(self.clientID, 'dr12_proximitySensorWall_',
                                                                vrep.simx_opmode_oneshot_wait)
        # retrieve motor  handles
        errorCode, self.left_motor_handle = vrep.simxGetObjectHandle(self.clientID, 'dr12_leftJoint_',
                                                                vrep.simx_opmode_oneshot_wait)
        errorCode, self.right_motor_handle = vrep.simxGetObjectHandle(self.clientID, 'dr12_rightJoint_',
                                                                 vrep.simx_opmode_oneshot_wait)
        return sensor_handle, wall_sensor_handle

    def act(self, vl, vr):
        '''
        implements action command in the robot world
        paprms: vl, vr - left and right velocities
        ;return: True if action was actuated
        '''
        # print("Velocity of left motor =", vl)
        # print("Velocity of right motor =", vr)
        # steer straight or away from obstacles
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor_handle, vl, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor_handle, vr, vrep.simx_opmode_streaming)
        return True

    def close(self):
        '''
        close connection
        '''
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(self.clientID)
        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)


class RobotBrain(object):

    '''
    The main intelligent controller of the simulated robot
    '''
    def __init__(self):
        pass

    def perception(self, myWorld, myHeuristic):
        '''
        Read state and build a representation
        param: myWorld , which represents the world of the robot
                myHeuristic, which represents the heuristic function the robot works with
        '''
        # get representation of the maze world and state
        errorCode, robot_handle = vrep.simxGetObjectHandle(myWorld.clientID, 'dr12',
                                                                vrep.simx_opmode_oneshot_wait)
        position = vrep.simxGetObjectPosition(myWorld.clientID, robot_handle, -1, vrep.simx_opmode_streaming)
        speed = 0
        heuristic = myHeuristic
        myMazeWorld = MazeWorld(position, speed, heuristic)
        return myMazeWorld

    def decision(self, myWorld, action):
        '''
        The state contains the world representation
        param: myWorld , which represents the world of the robot
                action, which decides how to act according to action sent
        '''

        # action 0 means go straight
        if(action == 0):
            print("going straight")
            myWorld.act(4, 4)
            time.sleep(0.1)
        # action 1 means adjust left until you hug the wall again
        elif (action == 1):
            print("try to find the wall")
            myWorld.act(1, 3)
            time.sleep(0.1)
        # action 2 means go right, while maintaining contact with left wall
        elif (action == 2):
            print("go right")
            myWorld.act(4.2, 0)
            time.sleep(0.3)
        # action 3 means reached the end, stop
        elif (action == 3):
            print("Robot Stops")
            myWorld.act(0, 0)
            myWorld.close()

    def think(self, myWorld, myHeuristic):
        '''
        It decides which action to take given the sensor reading
        param: myWorld , which represents the world of the robot
                myHeuristic, which represents the heuristic function the robot works with
        '''
        mazeWorld = self.perception(myWorld, myHeuristic)

        while True:
            # read sensor data
            sensor_handle, wall_sensor_handle = myWorld.sense()

            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                myWorld.clientID, sensor_handle, vrep.simx_opmode_streaming)

            returnCodeWall, detectionStateWall, detectedPointWall, detectedObjectHandleWall, detectedSurfaceNormalVectorWall = vrep.simxReadProximitySensor(
                myWorld.clientID, wall_sensor_handle, vrep.simx_opmode_streaming)

            print("following the wall? " + str(detectionStateWall))
            # go to heurstic
            heu = mazeWorld.getState().getH()
            heu.H(self, detectionState, detectionStateWall, myWorld)
            # if reached the end - stop
            if mazeWorld.solution(mazeWorld.getState()):
                self.decision(myWorld, 3)

# ------------------ maze model functions ------------- #
class RobotState:

    def __init__(self, position, speed, heuristic):
        self.position = position
        self.speed = speed
        self.H = heuristic

    def getH(self):
        return self.H


class MazeWorld(object):

    def __init__(self, position, speed, heuristic):
        self.state = RobotState(position, speed, heuristic)
        self.heuristic = heuristic

    # def neighbors(self, state):
    #     out = []
    #     n = RobotState(state.position, state.speed, state.H)
    #     out.append(n)
    #     return out

    def getState(self):
        return self.state

    def solution(self, state):
        # when the robot reaches the goal position can stop
        if state.position[1] == GOAL_POSITION:
            print("SOLUTION FOUND")
            return True
        return False


# ------------------  heuristic functions ------------- #


class Heuristic:

    def __init__(self):
        pass

    def H(self, brain, detectionState, detectionStateWall, myWorld):
        return 1


class RobotHeuristic(Heuristic):

    # prefer going straight while hugging the left wall, if not possible - go right
    def H(self, brain, detectionState, detectionStateWall, myWorld):
        # if no obstacles and near a left wall
        if detectionState == False and detectionStateWall:
            brain.decision(myWorld, 0)
        # if losing left wall
        elif detectionStateWall == False:
            print("wall lost")
            brain.decision(myWorld, 1)
        # if there is an obstacle in front
        elif detectionState:
            brain.decision(myWorld, 2)

        return
