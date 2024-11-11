# this file stores the data as a global variable

import numpy as np

#####################       ToF Data        #######################
directions = np.array(['ToF Front','ToF Back','ToF Left','ToF Right'])

display_matrix = {'ToF Front': np.zeros((8, 8), dtype=np.float32),
                  'ToF Back': np.zeros((8, 8), dtype=np.float32),
                  'ToF Left': np.zeros((8, 8), dtype=np.float32),
                  'ToF Right': np.zeros((8, 8), dtype=np.float32)}

global_frames = {  'ToF Front' : np.zeros((168, 168,3), dtype=np.uint8),
                    'ToF Back' : np.zeros((168, 168,3), dtype=np.uint8),
                    'ToF Left' : np.zeros((168, 168,3), dtype=np.uint8),
                    'ToF Right': np.zeros((168, 168,3), dtype=np.uint8)}
update_frames = {
    'ToF Front' : False,
    'ToF Back' : False,
    'ToF Left' : False,
    'ToF Right': False
}

new_frame:bool = False

#####################       flight controller        #######################

controlState = "search"

movingObjectDirection = "ToF Front"

forward_vel = 0
sideways_vel = 0
yaw_rate = 0
height = 0

FPS_mean=0

yaw_k = -10
yaw_kd = -4
yaw_ki = -1

#####################       Logging        #######################

vx=0
vy=0
rateYaw=0
yaw=0
xPos=0
yPos=0

#####################       Environment        #######################

environment = None
