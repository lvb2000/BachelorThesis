import cflib.crtp  # noqa
import numpy as np
from cflib.crazyflie import Commander

from custom_logger import Logger
import time
from datetime import datetime
import cv2
import cache
import math
import helper as h
import pandas as pd

def basicAvoidance():
    # time offset for starting
    time.sleep(4)
    firstActive = True
    firstInactive = True

    #
    updateInterval = 0.01
    areaInit = False
    lastWidestArea = 0
    defaultStoreCounter = (1/updateInterval)
    storeCounter = defaultStoreCounter
    velocity = []
    PoCStorage = []
    zeroCounter = 0

    while True:
        if cache.controlState == "focus":
            if firstActive:
                print("Basic avoidance in all directions is active.")
                firstActive = False
                firstInactive = True
            lastWidestArea,areaInit,storeCounter,lastFarestValue,zeroCounter = closestAreaAvoidance(areaInit,lastWidestArea,storeCounter,velocity,PoCStorage,zeroCounter)
            if storeCounter < 0:
                storeCounter = defaultStoreCounter
            time.sleep(updateInterval)
        else:
            if firstInactive:
                print("Basic avoidance in all directions is inactive.")
                firstInactive = False
                firstActive = True
            time.sleep(updateInterval)

def closestAreaAvoidance(areaInit, lastWidestArea,storeCounter,velocity,PoCStorage,zeroCounter):
    closestValue,widestArea,farestValue,storeCounter,lastFarestValue = PoC(storeCounter,PoCStorage,lastWidestArea)
    if areaInit:
        widestArea= h.smoothValue(lastWidestArea,lastFarestValue,farestValue,widestArea,closestValue)
    #print(f"widestArea {widestArea}")
    if closestValue > 0.7 and farestValue < 0.98:
        droneYaw = cache.yaw
        if droneYaw < 0:
            droneYaw += 360
        directionToFly = h.convertAreaInDegree(widestArea)
        #print(f"Direction to fly {directionToFly}")
        #angularDistance = directionToFly - droneYaw
        angularDistance = directionToFly
        #print(f"Angular Distance {angularDistance}")
        if angularDistance < 0:
            angularDistance += 360
        defaultMaxSpeed = [0.8,0.8]
        if closestValue < 0.8:
            defaultMaxSpeed[0] = 0.6
            defaultMaxSpeed[1] = 0.6
        if farestValue > 0.85:
            defaultMaxSpeed[0] = 0.3
            defaultMaxSpeed[1] = 0.3
        factorX = math.cos(math.radians(angularDistance))
        factorY = math.sin(math.radians(angularDistance))
        #print(f"FactorX {factorX}; FactorY {factorY}")
        defaultMaxSpeed[0]=abs(defaultMaxSpeed[0]*factorX)
        defaultMaxSpeed[1]=abs(defaultMaxSpeed[1]*factorY)
        if factorX > 0:
            cache.forward_vel = defaultMaxSpeed[0]
        else:
            cache.forward_vel = -defaultMaxSpeed[0]
        if factorY > 0:
            cache.sideways_vel = defaultMaxSpeed[1]
        else:
            cache.sideways_vel = -defaultMaxSpeed[1]
        if abs(cache.forward_vel) < 0.1:
            cache.forward_vel = 0
        if abs(cache.sideways_vel) < 0.1:
            cache.sideways_vel = 0
        if not areaInit:
            areaInit = True
        #print(f"sideways vel: {cache.sideways_vel} forward vel: {cache.forward_vel}")
    else:
        if zeroCounter >= 250:
            cache.controlState = "search"
            zeroCounter = 0
        zeroCounter += 1
        cache.forward_vel = 0
        cache.sideways_vel = 0
        areaInit = False
    #print(f"Widest Area: {widestArea}")
    #print(f"sideways vel: {cache.sideways_vel} forward vel: {cache.forward_vel}")
    velocity.append((cache.forward_vel,cache.sideways_vel))
    if storeCounter <= 0:
        velocityStore = np.array(velocity.copy())
        df = pd.DataFrame(velocityStore)
        df.to_csv("Velocity.csv")
    storeCounter -= 1
    return widestArea,areaInit,storeCounter,lastFarestValue,zeroCounter

def PoC(storeCounter,PoCStorage,lastWidestArea):
    PoC = np.ones(4 * 8)*-1
    # fill in PoC with Sensor Data
    fillSensorData(PoC)
    # apply smoothing of the values (maybe add values in the end and in the beginning
    PoC = smoothArray(PoC)
    # calculate index with highest value
    closestDirection = np.argmax(PoC)
    closestValue = np.max(PoC)
    oppositeDirection = closestDirection - 16
    if oppositeDirection < 0:
        oppositeDirection = 32 + oppositeDirection
    oppositeDirectionPoC = 0.6
    if oppositeDirectionPoC < 0.6:
        return closestValue, oppositeDirection, oppositeDirectionPoC, storeCounter, PoC[lastWidestArea]
    # replace all values which are -1 with 1 to calculate the widest area
    PoCStorage.append(PoC.copy())
    PoC[PoC == -1] = 1
    widestArea = np.argmin(PoC)
    #print(f"Widest Area: {widestArea}")
    farestValue = np.min(PoC)
    if storeCounter <= 0:
        PoCStorageStore = np.array(PoCStorage.copy())
        df = pd.DataFrame(PoCStorageStore)
        df.to_csv("PoCStorage.csv")
    return closestValue,widestArea,farestValue,storeCounter,PoC[lastWidestArea]



def fillSensorData(PoC):
    # find areas which are covered by ToF Sensors
    for dir in cache.directions:
        #sensorAngle = cache.yaw
        sensorAngle = 0
        if dir == "ToF Left":
            sensorAngle +=90
        elif dir == "ToF Right":
            sensorAngle +=270
        elif dir == "ToF Back":
            sensorAngle +=180
        matrix = cache.display_matrix[dir].copy()
        #matrix = matrix[2:-2, :]
        for col in range(8):
            # calculate angle of column
            alpha = h.convertColInDegree(col)
            colAngle = sensorAngle + alpha
            if colAngle < 0:
                colAngle += 360
            colAngle = colAngle % 360
            PoCIndex = int(colAngle / 11.25)
            if PoCIndex > 31:
                PoCIndex = 31
            if np.count_nonzero(matrix[:, col] == -1) < 7:
                colArray = removeInvalidAndAlign(matrix[:, col])
                # calculate PoC Value of column
                minDistance = np.min(colArray)
                meanDistance = np.mean(colArray)
                minPoc = 0
                meanPoc = 0
                if dir == cache.movingObjectDirection and cache.controlState == "focus":
                    minPoc = h.distance2PoC9(minDistance)
                    #minPoc = 1
                else:
                    minPoc = h.distance2PoC4(minDistance)
                if dir == cache.movingObjectDirection and cache.controlState == "focus":
                    meanPoc = h.distance2PoC9(meanDistance)
                    #meanPoc = 1
                else:
                    meanPoc = h.distance2PoC4(meanDistance)
                PoCValue=0.9*minPoc+0.1*meanPoc
                # set value in PoC
                if PoC[PoCIndex] != -1:
                    PoCValue = (PoC[PoCIndex]+PoCValue)/2
                PoC[PoCIndex]=PoCValue
            else:
                PoCValue = 0.4
                if PoC[PoCIndex] != -1:
                    PoCValue = (PoC[PoCIndex]+PoCValue)/2
                PoC[PoCIndex]=PoCValue

def smoothArray(PoC, smoothing=6):
    newPoC = np.ones(4 * 8) * -1
    # add values in the end and in the beginning
    PoC = np.append(PoC, PoC[0:smoothing])
    PoC = np.insert(PoC,0, PoC[-2*smoothing:-smoothing])
    # smooth array
    for i in range(smoothing, len(PoC) - smoothing):
        if PoC[i] != -1:
            #take mean of values before and after but exclude all values which are -1
            window = PoC[i - smoothing:i + smoothing]
            window = window[window != -1]
            if sum(window) != 0:
                maxPoc = np.max(window)
                newPoC[i-smoothing] = (PoC[i]+2*maxPoc)/3
    # remove added values
    #print(newPoC)
    return newPoC

def removeInvalidAndAlign(matrix):
    matrix_f = matrix.flatten()
    return matrix_f[matrix_f != -1]
