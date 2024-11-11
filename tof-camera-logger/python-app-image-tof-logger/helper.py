import numpy as np
import math

import cache


def recomputeVelocityAfterYaw(deltaYaw):
    x = cache.forward_vel
    y= cache.sideways_vel
    betrag = math.sqrt(math.pow(x,2)+math.pow(y,2))
    if x==0:
        x=0.0001
    alpha = math.atan(y/x)
    cache.forward_vel = math.cos(deltaYaw-alpha) * betrag
    cache.sideways_vel = math.sin(deltaYaw-alpha) * betrag
def convertColInDegree(col):
    # 0.558 = tan(45/2); 5.625 half zone
    return math.degrees(math.atan((7 - 2 * col) / 7 * 0.414))

def convertAreaInDegree(area):
    return 11.25*area-5.625

def distance2PoC4(distance):
    PoC = 0
    if distance <= 2:
        PoC = -(1/4)*math.pow(distance, 2)+1
    return PoC

def distance2PoC9(distance):
    PoC = 0
    if distance <= 2:
        PoC = -(1 / 9) * math.pow(distance, 2) + 1 +0.15
    if PoC>1:
        PoC=1
    return PoC

def getYawFromDirection(direction):
    yaw = 0
    if direction == "ToF Front":
        yaw = 0
    elif direction == "ToF Left":
        yaw = 90
    elif direction == "ToF Right":
        yaw = 270
    elif direction == "ToF Back":
        yaw = 180
    return yaw

def getEnvValue(x,y):
    envValue = 0
    if x > 0 and y > 0:
        envValue = "UpRight"
    elif x < 0 and y > 0:
        envValue = "UpLeft"
    elif x < 0 and y < 0:
        envValue = "DownLeft"
    elif x > 0 and y < 0:
        envValue = "DownRight"
    return envValue

def pythagoras(x1,x2,y1,y2):
    return math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))

def computeAproachingAngle(references):
    valid = True
    a=math.radians(5.625)
    c=references[0]/references[1]
    droneAngle = -1
    b = -2*c*math.cos(a)+math.pow(math.sin(a),2)+math.pow(math.cos(a),2)+c*c
    if b > 0:
        f = math.sqrt(b)
        if f != 0:
            d = math.sin(a) / f
            if abs(d)<1:
                droneAngle = math.acos(d)
            else:
                valid = False
        else:
            valid = False
    else:
        valid = False
    rectdist = references[0]*math.cos(droneAngle)
    return droneAngle,rectdist,valid

def getNextPixelDistance(droneAngle,rectdist):
    alpha = math.radians(5.625)
    return rectdist/math.cos(droneAngle+2*alpha)

def smoothValue(lastWidestArea,lastFarestValue,farestValue,widestArea,closestValue):
    #TODO check if threshold can also be 1-farestValue+0.1
    #print(f"FarestValue: {farestValue}, ClosestDistance: {closestDistance}")
    threshold = 4
    if farestValue <0.6:
        threshold = 0.3
    elif farestValue <0.7:
        threshold = 0.25
    elif farestValue <0.8:
        threshold = 0.25
    elif farestValue <0.9:
        threshold = 0.225
    elif farestValue <0.95:
        threshold = 0.2
    elif farestValue <0.98:
        threshold = 0.125
    elif farestValue <1:
        threshold = 0.10
    if closestValue < 0.87:
        threshold = 0.35
    if abs(lastFarestValue-farestValue) < threshold:
        return lastWidestArea
    else:
        return widestArea
    #return widestArea,farestValue

