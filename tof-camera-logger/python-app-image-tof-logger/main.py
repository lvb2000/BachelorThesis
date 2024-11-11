import threading
import time
import logging
from datetime import datetime
import cv2
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from crazyflie_manager import new_packet_received
from controller import controller
from basicAvoidance import basicAvoidance
from mapping import classifyMovingObject
import cache
import numpy as np
from focus import focusOnNearestObject

FPSMeasurments = {  'ToF Front' : np.zeros(15, dtype=np.float32),
                    'ToF Back' : np.zeros(15, dtype=np.float32),
                    'ToF Left' : np.zeros(15, dtype=np.float32),
                    'ToF Right':np.zeros(15, dtype=np.float32)}
FPSIndex = {   'ToF Front' : 0,
                    'ToF Back' : 0,
                    'ToF Left' : 0,
                    'ToF Right':0 }
LastMeasurements = {   'ToF Front' : None,
                    'ToF Back' : None,
                    'ToF Left' : None,
                    'ToF Right': None }
init = {   'ToF Front' : False,
                    'ToF Back' : False,
                    'ToF Left' : False,
                    'ToF Right': False }
def FPSCounter(key):
    if init[key]:
        # calculate current FPS
        FPS = 1/(time.time()-LastMeasurements[key])
        # update last Measurement
        LastMeasurements[key] = time.time()
        # find index for rolling average
        idx = FPSIndex[key]
        # safe measurement in FPS array
        FPSMeasurments[key][idx]=FPS

        #check if 15 measurements have passed
        if idx == 14:
            # print FPS
            cache.FPS_mean = int(np.mean(FPSMeasurments[key]))
            if cache.FPS_mean <= 20:
                print(f"Error on ToF Sensor {key} with mean FPS of: {cache.FPS_mean}")
            FPSIndex[key] = 0
        else:
            # increment index for next measurement
            FPSIndex[key] +=1
    else:
        # if no measurements has been taken initialize with the first measurement
        LastMeasurements[key] = time.time()
        # set init to true
        init[key]=True

def simple_log(logger):
    for log_entry in logger:
        #timestamp = log_entry[0]
        data = log_entry[1]
        #logconf_name = log_entry[2]
        #cache.vx = data["stateEstimateZ.vx"]
        #cache.vy = data["stateEstimateZ.vy"]
        #cache.rateYaw = data["stateEstimateZ.rateYaw"]
        cache.yaw = data["stateEstimate.yaw"]
        #yaw = data["stateEstimate.yaw"]
        #print(f"Drone yaw from Logger {yaw}")
        cache.xPos = data["kalman.stateX"]
        #print(f"Drone x Pos from Logger {cache.xPos}")
        cache.yPos = data["kalman.stateY"]
        #print(f"Drone y Pos from Logger {cache.yPos}")
        break

def makeFrame():
    frame = None
    if cache.environment is not None:
        frame = cv2.cvtColor(cache.environment.copy(), cv2.COLOR_GRAY2RGB)
        for row in range(0, frame.shape[0]):
            for col in range(0, frame.shape[1]):
                if cache.environment[row, col] == 1:
                    frame[row, col, 0] = 0
                    frame[row, col, 1] = 0
                    frame[row, col, 2] = 255
                if cache.environment[row,col] == 10:
                    frame[row, col, 0] = 255
                    frame[row, col, 1] = 0
                    frame[row, col, 2] = 0
        frame = frame.astype(np.uint8)
        frame = cv2.resize(frame, dsize=[500, 500], interpolation=cv2.INTER_NEAREST)
    return frame


if __name__ == '__main__':

    time0 = datetime.now()

    # Set up drone radio connection
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # drones = cflib.crtp.scan_interfaces()
    # uri = drones[0][0]
    uri = 'radio://0/80/2M/E7E7E7E7E0'
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    # Start callback thread for receiving data from drone
    cf.add_port_callback(1, new_packet_received)

    lg_statez = LogConfig(name='stateEstimate', period_in_ms=10)
    #lg_statez.add_variable('stateEstimateZ.vx', 'float')
    #lg_statez.add_variable('stateEstimateZ.vy', 'float')
    #lg_statez.add_variable('stateEstimateZ.rateYaw', 'float')
    lg_statez.add_variable('stateEstimate.yaw', 'float')
    lg_statez.add_variable('kalman.stateX', 'float')
    lg_statez.add_variable('kalman.stateY', 'float')

    with SyncCrazyflie(uri, cf=cf) as scf:

        # Start ToF thread
        thread_1 = threading.Thread(target=controller, args=(time0,scf))
        thread_1.start()

        # Start Basic Avoidance thread
        thread_2 = threading.Thread(target=basicAvoidance)
        thread_2.start()

        thread_3 = threading.Thread(target=focusOnNearestObject)
        thread_3.start()

        thread_4 = threading.Thread(target=classifyMovingObject)
        thread_4.start()


        print("Logging activated.")
        frameCount = 200
        #with SyncLogger(scf, lg_statez) as logger:
        while True:
            #simple_log(logger)
            for key in init:
                if cache.update_frames[key]:
                    cache.update_frames[key] = False
                    #FPSCounter(key)
                    frameCount = 200
                else:
                    if frameCount == 0:
                        print("Error: No ToF Frames received.")
                    else:
                        frameCount -= 1
            time.sleep(0.005)

        # Clean up
    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()

