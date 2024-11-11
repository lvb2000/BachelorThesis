import cflib.crtp  # noqa
from cflib.crazyflie import Commander

import time
import cache
import sys

########---------------- PARAMETER BEGIN ---------------############
default_forward_vel = 0
default_sideways_vel = 0.0
default_yaw_rate = 0.0
default_height = 0.7

timestamp = time.time()
flightTime = 120  # flightTime in seconds

focusCounter = 0

flying = True


########---------------- PARAMETER END ---------------############

def controller(time_ref, scf):
    global time0
    time0 = time_ref

    commander = Commander(scf.cf)

    cache.yaw_rate = default_yaw_rate
    cache.forward_vel = default_forward_vel
    cache.sideways_vel = default_sideways_vel
    cache.height = default_height

    first = True

    counter = 0

    while True:
        # land the drone if flight Time is over
        if flying:
            if first:
                waitTime = 2
                time.sleep(waitTime)
                print(f"Wait {waitTime} sec for Takeoff.")
                first = False
            if time.time() > (timestamp + flightTime):
                print("Landing.")
                commander.send_hover_setpoint(vx=0, vy=0, yawrate=0, zdistance=default_height / 2)
                time.sleep(1)
                commander.send_hover_setpoint(vx=0, vy=0, yawrate=0, zdistance=0.1)
                time.sleep(1)
                commander.send_hover_setpoint(vx=0, vy=0, yawrate=0, zdistance=0)
                time.sleep(1)
                sys.exit("Landed safely.")
            else:
                if counter > 1000:
                    counter = 0
                    print(f"Controller: {cache.controlState}")
                counter+= 1
                # send new hover-points to drone
                commander.send_hover_setpoint(vx=cache.forward_vel, vy=cache.sideways_vel, yawrate=cache.yaw_rate,
                                              zdistance=cache.height)
                #commander.send_hover_setpoint(vx=0, vy=0, yawrate=0,zdistance=0.5)
                #const = 0.1
                #factorX = math.cos(math.radians(cache.yaw))
                #factorY = -math.sin(math.radians(cache.yaw))
                #print(f"Yaw: {cache.yaw}; Faktor X: {factorX}; Faktor Y: {factorY}")
                #commander.send_hover_setpoint(vx=const*factorX, vy=const*factorY, yawrate=5,zdistance=cache.height)
                # sleep to give other tasks time to execute
                time.sleep(0.01)
                # time.sleep(3)
        else:
            if first:
                print("Flying is deactivated.")
                first = False
            time.sleep(1)