#coding : utf-8
__author__ = 'Yuna Phrolov'

import RobotWorld
import time

DELAY = 0.1  # in seconds

myWorld = RobotWorld.World(host='127.0.0.1', portNumber=19999)
myRobotBrain = RobotWorld.RobotBrain()
myHeuristic = RobotWorld.RobotHeuristic()

while True:
    myRobotBrain.think(myWorld, myHeuristic)
    time.sleep(DELAY)
