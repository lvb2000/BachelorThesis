import cflib.crtp  # noqa
import numpy as np
from cflib.crazyflie import Commander

from custom_logger import Logger
import time
from datetime import datetime
import cv2
import cache
import random
import math

from configs import *

FPSCounter = 0
start = None

tof_data = np.zeros(4 * 64, dtype=np.float32)
distances = np.zeros((8, 8), dtype=np.float32)
targets = np.zeros((8, 8), dtype=np.float32)
status = np.zeros((8, 8), dtype=np.float32)

tof_data_log = Logger(8, 8)
startup_time = datetime.now()

max_tof_distance_mm = 3.0 * 1000


# gets called from run function in __init__.py
def new_packet_received(packet):
    global startup_time
    # Delay needed for multiprocessing
    if (datetime.now() - startup_time).total_seconds() < 3.0:
        return

    data = packet._get_data_l()

    ToF = {'ToF Front': 97, 'ToF Back': 98, 'ToF Left': 99, 'ToF Right': 100}
    if data[0] == ToF['ToF Front'] or data[0] == ToF['ToF Back'] or data[0] == ToF['ToF Left'] or data[0] == ToF[
        'ToF Right']:
        tof_direction = list({i for i in ToF if ToF[i] == data[0]})[0]
        # the starting index of the current data packet in the whole massage (complete tof frame of one sensor)
        idx = data[1]
        # remove the two previous bytes (D and idx)
        for i in range(len(data) - 2):
            tof_data[idx * 28 + i] = data[i + 2]
        # sends 4*64/28 + 1 = 10 packets from 0 to 9
        # that means the if checks if the last packet is received
        if idx == 9:
            for i in range(8):
                for j in range(8):
                    # the distance is calculated by a pair of two consecutive bytes
                    # where the first is the distance in m and the second values after the dot
                    distances[i, j] = int(tof_data[2 * (j + 8 * i)] + 256 * tof_data[2 * (j + 8 * i) + 1])
                    targets[i, j] = int(tof_data[j + 8 * i + 128])
                    status[i, j] = int(tof_data[j + 8 * i + 128 + 64])

                    # check whether the distance is valid
                    if targets[i, j] < 1 or (status[i, j] != 5 and status[i, j] != 9):
                        cache.display_matrix[tof_direction][i, j] = -1
                    else:
                        cache.display_matrix[tof_direction][i, j] = distances[i, j] / 1000.0

            # tof_data_log.add(display_matrix)
            # file_name = (datetime.now() - time0).total_seconds()
            # np.save(cf_log_name + "/" + str(file_name) + ".npy", display_matrix)
            if not cache.update_frames[tof_direction]:
                #show_tof_frame(tof_direction)
                cache.update_frames[tof_direction] = True


def show_tof_frame(tof_direction):

    # Make invalid pixels red
    tof_show = cache.display_matrix[tof_direction].copy()
    tof_show = cv2.cvtColor(cache.display_matrix[tof_direction].copy(), cv2.COLOR_GRAY2RGB)
    for row in range(0, tof_show.shape[0]):
        for col in range(0, tof_show.shape[1]):
            if cache.display_matrix[tof_direction].copy()[row, col] == -1:
                tof_show[row, col, 0] = 0
                tof_show[row, col, 1] = 0
                tof_show[row, col, 2] = 255

    # Scale to uint8 range
    tof_show = (tof_show * 255 / 3.0).astype(np.uint8)

    # For visibility of output only
    tof_show = cv2.resize(tof_show, dsize=[168, 168], interpolation=cv2.INTER_NEAREST)
    cache.global_frames[tof_direction] = tof_show

    cache.update_frames[tof_direction] = True