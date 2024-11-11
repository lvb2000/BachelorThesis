import cflib.crtp  # noqa
import numpy as np
import time
import cache
import helper
from objects import ToF_Frame
import pandas as pd


def focusOnNearestObject():
    """
    To focus the drone on the nearest object the yaw rate adjusts that the object's center
    is in the middle of the ToF line of sight. The adjustment is controlled by a standard
    PD-controller with parameters:

    k = 0.1  and k_p = 0.01

    0   |   1   |   2   |   3   |   4   |   5   |   6   |   7
  left  |   half left   |     middle    |   half right  | right

    """
    AbortCounter1 = 0
    AbortCounter2 = 0

    sleeptime = 0.05
    # time offset for starting
    time.sleep(5)

    # last update of PD control
    lastMeasurement = 0
    lastErr = 0
    intErr = 0

    firstActive = True
    firstInactive = True

    defaultStoreCounter = (10 / sleeptime)
    storeCounter = defaultStoreCounter
    objects = []
    centerCols = []
    yaw_rates = []

    while True:
        if cache.controlState == "focus":
            if firstActive:
                print(f"Focus on nearest object in {cache.movingObjectDirection} Direction is active.")
                firstActive = False
                firstInactive = True
            lastMeasurement, lastErr, intErr, AbortCounter1, AbortCounter2,storeCounter = focus(lastMeasurement, lastErr, intErr,sleeptime, AbortCounter1,AbortCounter2, objects,centerCols, storeCounter,yaw_rates)
            if storeCounter < 0:
                storeCounter = defaultStoreCounter
            time.sleep(sleeptime)
        else:
            if firstInactive:
                print("Focus on nearest object is inactive.")
                firstInactive = False
                firstActive = True
            time.sleep(1)


def focus(lastMeasurement, lastErr, intErr, sleeptime, A1, A2, objects,centerCols, storeCounter,yaw_rates):
    matrix = cache.display_matrix[cache.movingObjectDirection].copy()
    CenterCol,ObjectDistance,storeCounter = fastObjectDetection(matrix, objects,centerCols, storeCounter)
    #print(f"Center Column: {CenterCol}")

    lastErr, intErr = yaw_controller(CenterCol,ObjectDistance, lastMeasurement, lastErr, intErr,yaw_rates,storeCounter)
    lastMeasurement = time.time()
    # print(f"lastErr: {lastErr}")
    if abs(lastErr) <= 1:
        A2 += 1
        if A2 > int(10 / sleeptime):
            A2 = 0
            cache.yaw_rate = 0
            cache.controlState = "search"
    else:
        A2 = 0
    A1 = 0

    return lastMeasurement, lastErr, intErr, A1, A2,storeCounter


def yaw_controller(centerCol,ObjectDistance, lastMeasurement, lastErr, intErr,yaw_rates,storeCounter):
    err = 0
    if lastMeasurement != 0:

        if centerCol == 7:
            err = -4
        if centerCol == 6:
            err = -2.5
        if centerCol == 5:
            err = -1
        if centerCol == 0:
            err = 4
        if centerCol == 1:
            err = 2.5
        if centerCol == 2:
            err = 1

        dt_err = (err - lastErr) / (time.time() - lastMeasurement)
        intErr += err * (time.time() - lastMeasurement)

        if centerCol == 3 or centerCol == 4:
            intErr = 0

        #print(f"err: {err}, dt_err: {dt_err}, intErr: {intErr}")
        yaw_rate = cache.yaw_k * err + cache.yaw_kd * dt_err + cache.yaw_ki * intErr
        if yaw_rate > 80:
            yaw_rate = 80
        if yaw_rate < -80:
            yaw_rate = -80

        yaw_rates.append(yaw_rate)
        if storeCounter <= 0:
            YawStore = np.array(yaw_rates.copy())
            dfObjects = pd.DataFrame(YawStore)
            dfObjects.to_csv("FocusYaws.csv")

        #helper.recomputeVelocityAfterYaw(yaw_rate)
        cache.yaw_rate = yaw_rate
    return err, intErr

def fastObjectDetection(matrix, objects,centerCols, storeCounter):
    object = np.zeros((8, 8), dtype=np.float32)
    # replace every invalid entry with distance 3m
    matrix[matrix == -1] = 3
    # get closest Pixel and closest value
    closestPixel = np.argmin(matrix)
    closestValue = np.min(matrix)
    objectDistance = closestValue
    # everything which has a discrepancy of 1dm to the closest pixel is considered to be part of the object
    upperBound = closestValue + 0.5
    center_row = int(closestPixel / 8)
    center_col = int(closestPixel % 8)
    RecursiveCluster(matrix, object, center_row, center_col, upperBound)
    centerCol = getCenterCol(object)
    objects.append(object.flatten())
    centerCols.append(centerCol)
    if storeCounter <= 0:
        ObjectsStore = np.array(objects.copy())
        ColsStore = np.array(centerCols.copy())
        dfObjects = pd.DataFrame(ObjectsStore)
        dfCols = pd.DataFrame(ColsStore)
        dfObjects.to_csv("FocusObjects.csv")
        dfCols.to_csv("FocusCenterCols.csv")
    storeCounter -= 1
    return centerCol,closestValue,storeCounter

def getCenterCol(object):
    leftMostPixelIdx = 0
    rightMostPixelIdx = 7
    leftBoarderReached = False

    for count, column in enumerate(object.T):
        if (np.sum(column) > 2) and not leftBoarderReached:
            leftMostPixelIdx = count
            leftBoarderReached = True
        if (np.sum(column) <= 2) and leftBoarderReached:
            rightMostPixelIdx = count - 1
            break

    return getOuterCenter(rightMostPixelIdx, leftMostPixelIdx)

def getOuterCenter(rightMostPixelIdx, leftMostPixelIdx):
    sum = rightMostPixelIdx + leftMostPixelIdx
    if sum >=8:
        if sum%2 == 1:
            sum +=1
    if sum < 8:
        if sum%2 == 1:
            sum -=1
    middle = int(sum/2)
    return middle
def RecursiveCluster(grid, solution, i, j, threshold):
    if i < 0 or i >= grid.shape[0] or j < 0 or j >= grid.shape[1]:
        return
    if solution[i, j] == 0 and grid[i, j] < threshold:
        solution[i, j] = 1
        RecursiveCluster(grid, solution, i + 1, j, threshold)
        RecursiveCluster(grid, solution, i - 1, j, threshold)
        RecursiveCluster(grid, solution, i, j + 1, threshold)
        RecursiveCluster(grid, solution, i, j - 1, threshold)