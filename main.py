__author__ = 'Martin Haeberli'
#but see 2014_FRC_3045_Auton for original authors

import vision as vision
#from ntClient import * # python native networktables implementation
from pynetworktables import * # python bindings to C++ native networktables implementation

import time

################################################################################
#
# main
#
################################################################################
# if this script is run directly by python, then __name__ is '__main__'. If it
# is run because it is imported, then __name__ is the module name.

if __name__ == '__main__':

    # Table name = team number, used for ip address
    tableName = "3045RobotVision"
    NetworkTable.SetIPAddress("127.0.0.1")
    #NetworkTable.SetIPAddress("10.30.45.2")
    NetworkTable.SetClientMode()
    NetworkTable.Initialize()
    table = NetworkTable.GetTable(tableName)
    table.PutNumber('test', 3.14159)
    #table = NetworkTableClient(tableName)
    #tableDirectory = '/' + tableName + '/'

    # Id's for reading data from networktable
    isGoalHotId = "isGoalHot"
    isGoalHorzId = "isGoalHorz"
    goalDistanceId = "goalDistance"

    # Id's for setting data to networktable
    gyroAngleId = "gyroAngle"
    gyroAngle = 0.0

    deltaTime = 0.9

    while True:
        #gyroAngle = table.getValue(tableDirectory + gyroAngleId)
        gyroAngle = 0
        start= time.time()
        vision.update(table, gyroAngle, deltaTime)
        end= time.time()
        deltaTime = (end - start)
        #print "delta : " + str(delta)
        #print "fps : " + str(1 / (delta))
        #table.setValue(tableDirectory + goalDistanceId, vision.getDistace())
        #table.setValue(tableDirectory + isGoalHotId, vision.getFoundHotTarget())
        #table.setValue(tableDirectory + isGoalHorzId, vision.getFoundHorzTarget())

        #time.sleep(1.0 / 4.0)