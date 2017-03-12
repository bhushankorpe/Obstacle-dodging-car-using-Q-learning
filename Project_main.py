# -*- coding: utf-8 -*-
"""
Created on Mon Nov 28 19:01:35 2016

@author: Bhushan Korpe
"""
import vrep
import time

# Setting up connection with VREP
vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Remote API function call returned with error code: ')

# Get all Object handles here      
errorCode, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)

errorCode, sensor2 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_oneshot_wait)
errorCode, sensor3 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_oneshot_wait)
errorCode, sensor4 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_oneshot_wait)
errorCode, sensor5 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_oneshot_wait)
errorCode, sensor6 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_oneshot_wait)
errorCode, sensor7 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_oneshot_wait)

# Read sensor values once

returnCode,detectionState2,detectedPoint2,detectedObjectHandle2,detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, sensor2,  vrep.simx_opmode_streaming)
returnCode,detectionState3,detectedPoint3,detectedObjectHandle3,detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(clientID, sensor3,  vrep.simx_opmode_streaming)
returnCode,detectionState4,detectedPoint4,detectedObjectHandle4,detectedSurfaceNormalVector4 = vrep.simxReadProximitySensor(clientID, sensor4,  vrep.simx_opmode_streaming)
returnCode,detectionState5,detectedPoint5,detectedObjectHandle5,detectedSurfaceNormalVector5 = vrep.simxReadProximitySensor(clientID, sensor5,  vrep.simx_opmode_streaming)
returnCode,detectionState6,detectedPoint6,detectedObjectHandle6,detectedSurfaceNormalVector6 = vrep.simxReadProximitySensor(clientID, sensor6,  vrep.simx_opmode_streaming)
returnCode,detectionState7,detectedPoint7,detectedObjectHandle7,detectedSurfaceNormalVector7 = vrep.simxReadProximitySensor(clientID, sensor7,  vrep.simx_opmode_streaming)


#start read state function
def readState():
    
    # Read sensor values 
    returnCode,detectionState2,detectedPoint2,detectedObjectHandle2,detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, sensor2,  vrep.simx_opmode_buffer)
    returnCode,detectionState3,detectedPoint3,detectedObjectHandle3,detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(clientID, sensor3,  vrep.simx_opmode_buffer)
    returnCode,detectionState4,detectedPoint4,detectedObjectHandle4,detectedSurfaceNormalVector4 = vrep.simxReadProximitySensor(clientID, sensor4,  vrep.simx_opmode_buffer)
    returnCode,detectionState5,detectedPoint5,detectedObjectHandle5,detectedSurfaceNormalVector5 = vrep.simxReadProximitySensor(clientID, sensor5,  vrep.simx_opmode_buffer)
    returnCode,detectionState6,detectedPoint6,detectedObjectHandle6,detectedSurfaceNormalVector6 = vrep.simxReadProximitySensor(clientID, sensor6,  vrep.simx_opmode_buffer)
    returnCode,detectionState7,detectedPoint7,detectedObjectHandle7,detectedSurfaceNormalVector7 = vrep.simxReadProximitySensor(clientID, sensor7,  vrep.simx_opmode_buffer)


    # Use sensor values as average of two sensors
    leftSensor = (detectedPoint2[2]+detectedPoint3[2])/2
    frontSensor = (detectedPoint4[2]+detectedPoint5[2])/2
    rightSensor = (detectedPoint6[2]+detectedPoint7[2])/2

    # Read sensors and determine state
    if leftSensor<0.25:
        return "Left"
    elif rightSensor<0.25:
        return "Right"
    elif frontSensor<0.25:
        return "Front"
    elif leftSensor<0.25 and frontSensor<0.25:
        return "LeftFront"
    elif rightSensor<0.25 and frontSensor<0.25:
        return "RightFront"
    elif leftSensor<0.25 and rightSensor<0.25:
        return "LeftRight"
    elif leftSensor>0.25 and rightSensor>0.25 and frontSensor>0.25:
        return "Clear"
    elif leftSensor<0.1 or rightSensor<0.1 or frontSensor<0.1:
        return "Collision"    

#Create rewards table
#Initialize lists for every state
# R_L : Rewards in Left state
# The other lists are analogous to states
# Each column in turn represents states

R_L = [-2, -1, -1, -2, -2, -3, 20, -20]
R_F = [-1, -2, -1, -2, -2, -3, 10, -20]
R_R = [-1, -1, -2, -2, -2, -3, 20, -20]
R_LF = [-2, -2, -1, -3, -3, -3, 20, -20]
R_RF = [-1, -2, -2, -3, -3, -3, 20, -20]
R_LR = [-2, -2, -2, -3, -3, -3, 20, -20]
R_CL = [-3, -3, -3, -3, -3, -3, 20, -20]
R_C = [0, 0, 0, 0, 0, 0, 20, -20]


# Create Q-table (lists for each state)
# Initialize a separate Q Value list for every state
# Q_CL - Qvalues when in Clear state
# Each of the column indices represent an action:
# 0 - Turn left
# 1 - Turn Right
# 2 - Move Forward

Q_L = [0,0,0]
Q_F = [0,0,0]
Q_R = [0,0,0]
Q_LF = [0,0,0]
Q_RF = [0,0,0]
Q_LR = [0,0,0]
Q_CL = [0,0,0]
Q_C = [0,0,0]

# Define co-efficients for Q value calculation
alpha = 0.4
gamma = 0.9

#Create memory to track previous and current - state and actions, Qvalue

memory = {
    "previousState" : "Clear",
    "previousAction" : 2,
    "currentState" : "Clear",
    "currentAction" : 2,
    "qValue" : 0
}

memoryList = []
memoryList.append(memory)

# This method gets the index of the max value in a state from the Qvalue lists
# Each index maps to a corresponding action

def getAction(state):
    if state == "Left":
        return Q_L.index(max(Q_L))
    elif state == "Right":
        return Q_R.index(max(Q_R))
    elif state == "Front":
        return Q_F.index(max(Q_F))
    elif state == "LeftFront":
        return Q_LF.index(max(Q_LF))
    elif state == "RightFront":
        return Q_RF.index(max(Q_RF))
    elif state == "LeftRight":
        return Q_LR.index(max(Q_LR))
    elif state == "Clear":
        return Q_CL.index(max(Q_CL))
    elif state == "Collision":
        return Q_C.index(max(Q_C))

# This method is to move the robot in a particular direction
# The direction is calculated from the above getAction() method

def moveRobot(direction):
    if direction == 0:
        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0.1, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0.5, vrep.simx_opmode_streaming)
    elif direction == 1:
        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0.5, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0.1, vrep.simx_opmode_streaming)
    elif direction == 2:
        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0.6, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0.6, vrep.simx_opmode_streaming)

# Map column to the corresponding state

def getColumnOf(state):
    if state == "Left":
        return 0
    elif state == "Front":
        return 1
    elif state == "Right":
        return 2
    elif state == "LeftFront":
        return 3
    elif state == "RightFront":
        return 4
    elif state == "LeftRight":
        return 5
    elif state == "Clear":
        return 6
    elif state == "Collision":
        return 7

# getReward() is to get the reward value from the reward lists 
    
def getReward(previousState, currentState):
    if previousState == "Left":
        return R_L[getColumnOf(currentState)]
    if previousState == "Front":
        return R_F[getColumnOf(currentState)]
    if previousState == "Right":
        return R_R[getColumnOf(currentState)]
    if previousState == "LeftFront":
        return R_LF[getColumnOf(currentState)]
    if previousState == "RightFront":
        return R_RF[getColumnOf(currentState)]
    if previousState == "LeftRight":
        return R_LR[getColumnOf(currentState)]
    if previousState == "Clear":
        return R_CL[getColumnOf(currentState)]
    if previousState == "Collision":
        return R_C[getColumnOf(currentState)]

# This is to get the maximum Q value for a state 
# This is used for the calculation of a new Qvalue
        
def getCurrentMaxQvalue(state):
    if state == "Left":
        return max(Q_L)
    elif state == "Right":
        return max(Q_R)
    elif state == "Front":
        return max(Q_F)
    elif state == "LeftFront":
        return max(Q_LF)
    elif state == "LeftRight":
        return max(Q_LR)
    elif state == "RightFront":
        return max(Q_RF)
    elif state == "Clear":
        return max(Q_CL)
    elif state == "Collision":
        return max(Q_C)
        
# This method is used to calculate the new Qvalue
# Based on the formula mentioned in the report  
    
def calculateQvalue(previousQvalue):
    newQvalue = gamma*getCurrentMaxQvalue(memory["currentState"])
    newQvalue -= previousQvalue
    newQvalue += getReward(memory["previousState"],memory["currentState"])
    newQvalue *= alpha
    return newQvalue


# This function is to get the new state, action and Qvalue, every iteration

def updateMemory():
    memory["previousState"] = memory["currentState"]
    memory["previousAction"] = memory["currentAction"]
    memory["currentState"] = readState()
    memory["currentAction"] = getAction(memory["currentState"])
    memory["qValue"] += calculateQvalue(memory["qValue"])
    return

# This method is to constantly monitor the robot sensors to look for obslacles
# If there is an obstacle, the method returns boolean value TRUE, else FALSE

def checkForObstacle():
    # Read sensor values in loops
    returnCode,detectionState2,detectedPoint2,detectedObjectHandle2,detectedSurfaceNormalVector2 = vrep.simxReadProximitySensor(clientID, sensor2,  vrep.simx_opmode_buffer)
    returnCode,detectionState3,detectedPoint3,detectedObjectHandle3,detectedSurfaceNormalVector3 = vrep.simxReadProximitySensor(clientID, sensor3,  vrep.simx_opmode_buffer)
    returnCode,detectionState4,detectedPoint4,detectedObjectHandle4,detectedSurfaceNormalVector4 = vrep.simxReadProximitySensor(clientID, sensor4,  vrep.simx_opmode_buffer)
    returnCode,detectionState5,detectedPoint5,detectedObjectHandle5,detectedSurfaceNormalVector5 = vrep.simxReadProximitySensor(clientID, sensor5,  vrep.simx_opmode_buffer)
    returnCode,detectionState6,detectedPoint6,detectedObjectHandle6,detectedSurfaceNormalVector6 = vrep.simxReadProximitySensor(clientID, sensor6,  vrep.simx_opmode_buffer)
    returnCode,detectionState7,detectedPoint7,detectedObjectHandle7,detectedSurfaceNormalVector7 = vrep.simxReadProximitySensor(clientID, sensor7,  vrep.simx_opmode_buffer)



    # Use sensor values as average of two sensors
    leftSensor = (detectedPoint2[2]+detectedPoint3[2])/2
    frontSensor = (detectedPoint4[2]+detectedPoint5[2])/2
    rightSensor = (detectedPoint6[2]+detectedPoint7[2])/2
    
    print leftSensor
    print rightSensor
    print frontSensor
    
    if leftSensor<0.3 or rightSensor<0.3 or frontSensor<0.3:
        return True
    else:
        return False
        
# An infinite loop that keep training the robot as it moves through its environment
# If there is no obstacle, the robot keeps moving straight
# If an obstacle is spotted nearby, go into re-inforcement learning logic

while True:
    while (checkForObstacle()):

        for _ in range(5):
            time.sleep(1)
            updateMemory()
            moveRobot(memory["currentAction"])
            memoryList.append(memory)

        for i in range(0,4):
            updateQLists(memoryList[i]["previousState"],memoryList[i]["previousAction"],memoryList[i]["qValue"])



    moveRobot(2)
    memoryList = []

