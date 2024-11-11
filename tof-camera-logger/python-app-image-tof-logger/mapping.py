import cflib.crtp  # noqa
import numpy as np
import time
import cache
import math
from objects import ToF_Frame
import helper as h
from collections import deque
import pandas as pd

def classifyMovingObject():

    # general Task Inits
    time.sleep(7)
    np.set_printoptions(threshold=np.inf)
    firstActive = True
    firstInactive = True
    updateInterval = 0.01
    defaultStoreCounter = (1/updateInterval)
    storeCounter = defaultStoreCounter
    PositionStorage = []

    # Mapping Inits
    queueLength = int(1/updateInterval)
    objectQueues = {"ToF Front":deque(maxlen=queueLength),
                    "ToF Left":deque(maxlen=queueLength),
                    "ToF Right":deque(maxlen=queueLength),
                    "ToF Back":deque(maxlen=queueLength)}
    DistanceQueues = {"ToF Front": deque(maxlen=queueLength),
                    "ToF Left": deque(maxlen=queueLength),
                    "ToF Right": deque(maxlen=queueLength),
                    "ToF Back": deque(maxlen=queueLength)}
    movementFound = {"ToF Front":False,
                     "ToF Left":False,
                     "ToF Right":False,
                     "ToF Back":False}
    movementSpeed = {"ToF Front":0,
                     "ToF Left":0,
                     "ToF Right":0,
                     "ToF Back":0}

    while True:
        if cache.controlState == "search":
            if firstActive:
                print("Try to classify moving Object.")
                firstActive = False
                firstInactive = True
            if cache.forward_vel == 0 and cache.sideways_vel == 0:
                cache.yaw_rate = -15
                mapObjectMovement(objectQueues,DistanceQueues,movementFound,movementSpeed,updateInterval,queueLength,storeCounter,PositionStorage)
                storeCounter -= 1
                if storeCounter < 0:
                    storeCounter = defaultStoreCounter
                #print(f"Object Queues: {objectQueues}")
                #print(f"Movements: {movementFound}")
                #print(f"Speeds: {movementSpeed}")
            else:
                objectQueues["ToF Front"].clear()
                objectQueues["ToF Left"].clear()
                objectQueues["ToF Right"].clear()
                objectQueues["ToF Back"].clear()
                DistanceQueues["ToF Front"].clear()
                DistanceQueues["ToF Left"].clear()
                DistanceQueues["ToF Right"].clear()
                DistanceQueues["ToF Back"].clear()
                movementFound["ToF Front"] = False
                movementFound["ToF Left"] = False
                movementFound["ToF Right"] = False
                movementFound["ToF Back"] = False
                movementSpeed["ToF Front"] = 0
                movementSpeed["ToF Left"] = 0
                movementSpeed["ToF Right"] = 0
                movementSpeed["ToF Back"] = 0
            time.sleep(updateInterval)
        else:
            if firstInactive:
                # reset all values
                objectQueues["ToF Front"].clear()
                objectQueues["ToF Left"].clear()
                objectQueues["ToF Right"].clear()
                objectQueues["ToF Back"].clear()
                DistanceQueues["ToF Front"].clear()
                DistanceQueues["ToF Left"].clear()
                DistanceQueues["ToF Right"].clear()
                DistanceQueues["ToF Back"].clear()
                movementFound["ToF Front"] = False
                movementFound["ToF Left"] = False
                movementFound["ToF Right"] = False
                movementFound["ToF Back"] = False
                movementSpeed["ToF Front"] = 0
                movementSpeed["ToF Left"] = 0
                movementSpeed["ToF Right"] = 0
                movementSpeed["ToF Back"] = 0
                print("Moving object classification is deactivated.")
                firstInactive = False
                firstActive = True
            time.sleep(0.1)

def mapObjectMovement(objectQueues,DistanceQueues, movementFound, movementSpeed, updateInterval, queueLength, storeCounter, positionStorage):
    for direction in cache.directions:
        #direction = "ToF Front"
        yaw = h.getYawFromDirection(direction) + cache.yaw
        matrix = cache.display_matrix[direction].copy()
        frame = ToF_Frame(matrix)
        if direction == "ToF Front":
            if frame.Wall:
                print("Wall detected")
        if frame.istObject():
            #print(frame.getObjectDistance())
            alpha = h.convertColInDegree(frame.getCenterCol())
            x = int(math.cos(math.radians(yaw + alpha)) * frame.getObjectDistance() * 10)
            y = int(math.sin(math.radians(yaw + alpha)) * frame.getObjectDistance() * 10)
            positionStorage.append((x, y,cache.yaw))
            if abs(x)<40 and abs(y)<40:
                objectQueues[direction].append((x,y))
                DistanceQueues[direction].append(frame.getObjectDistance())
                if len(objectQueues[direction]) == queueLength:
                    movementFound[direction],movementSpeed[direction]=findMovingObject(objectQueues[direction],DistanceQueues[direction],updateInterval,direction)
        else:
            positionStorage.append((0, 0, cache.yaw))
            if len(objectQueues[direction])!=0:
                objectQueues[direction].append(objectQueues[direction][-1])
                for a in range(10):
                    DistanceQueues[direction].append(DistanceQueues[direction][-1])
    if storeCounter <= 0:
        positionStorageNP = np.array(positionStorage.copy())
        df = pd.DataFrame(positionStorageNP)
        df.to_csv("Position.csv")
    if any(movementFound.values()):
        fastestSpeed = 0
        fastestDirection = None
        for key in movementFound.keys():
            if movementFound[key]:
                if movementSpeed[key] > fastestSpeed:
                    fastestSpeed = movementSpeed[key]
                    fastestDirection = key
        cache.movingObjectDirection = fastestDirection
        cache.yaw_rate = 0
        cache.controlState = "focus"

def findMovingObject(objectQueue,DistanceQueue,updateInterval,direction):
    #TODO in C code add second max and second min to counter fluctuation
    meanDistanceToDrone = 0
    for i in range(0, len(DistanceQueue) - 1):
        distance = abs(DistanceQueue[i]-DistanceQueue[i+1])
        if distance<0.5:
            meanDistanceToDrone += distance
    meanDistanceToDrone /= (len(objectQueue)-1)
    meanDistanceToDrone /= updateInterval
    meanDistanceToDrone /= 10
    meanDistanceBetweenObject = 0
    for i in range(0,len(objectQueue)-1):
        distance = h.pythagoras(objectQueue[i][0],objectQueue[i+1][0],objectQueue[i][1],objectQueue[i+1][1])
        meanDistanceBetweenObject += distance

    # add the distance between first and last point to counter fluctuation
    meanDistanceBetweenObject += h.pythagoras(objectQueue[0][0],objectQueue[-1][0],objectQueue[0][1],objectQueue[-1][1])
    # same for second and second last point
    meanDistanceBetweenObject += h.pythagoras(objectQueue[1][0],objectQueue[-2][0],objectQueue[1][1],objectQueue[-2][1])
    # calculate mean distance
    meanDistanceBetweenObject /= (len(objectQueue)+1)
    # calculate mean speed from mean distance and update interval
    meanSpeedBetweenObjects = meanDistanceBetweenObject / updateInterval / 10
    print(f"Distance to Drone: {meanDistanceToDrone}")
    #print(f"Distance queue {DistanceQueue}")
    if meanDistanceToDrone > 0.032:
        return True,meanDistanceToDrone
    #TODO maybe change object detection to 80% of center column filled by object
    #if meanSpeedBetweenObjects > 4:
    #    print(f"Mean Speed Between Objects: {meanSpeedBetweenObjects}")
    #    print(f"Object queue {objectQueue}")
    #    return True,meanSpeedBetweenObjects
    else:
        return False,meanSpeedBetweenObjects
