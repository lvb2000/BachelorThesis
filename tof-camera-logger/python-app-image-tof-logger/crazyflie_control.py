import keyboard
from cflib.crazyflie import Commander
from crazyflie_commander import *
import time


def crazyflie_keyboard_controller(scf):
    # Motion params
    forward_step = 1.0      # [m/s]
    sideways_step = 1.0     # [m/s]
    angular_step = 57.3     # [deg/s]  57.3 deg/s == 1 rad/s
    height_step = 0.01      # [m]

    height = 1.0
    # [m] Start height

    # Init
    commander = Commander(scf.cf)
    time.sleep(1)
    commander.send_setpoint(0.0, 0.0, 0.0, 0)
    print('Delaying takeoff by 10 seconds. Make sure to click onto terminal window. If OpenCv window is focused, a keypress will terminate programm.')
    time.sleep(10)

    # Slow takeoff to avoid high currents and camera feed cutoff
    current_height = 0.0
    for i in range(0, 100):
        current_height += height / 100
        commander.send_hover_setpoint(vx=0.0, vy=0.0, yawrate=0.0, zdistance=current_height)
        time.sleep(0.01)

    kill_flag = False
    keep_flying = True
    while keep_flying:
        forward_vel = 0.0
        sideways_vel = 0.0
        yaw_rate = 0.0

        # CMD TURN RIGHT
        if keyboard.is_pressed('e'):
            yaw_rate += angular_step
        # CMD TURN LEFT
        if keyboard.is_pressed('q'):
            yaw_rate -= angular_step
        # CMD FORWARD
        if keyboard.is_pressed('w'):
            forward_vel += forward_step
        # CMD BACKWARD
        if keyboard.is_pressed('s'):
            forward_vel -= forward_step
        # CMD RIGHT
        if keyboard.is_pressed('d'):
            sideways_vel -= sideways_step
        # CMD LEFT
        if keyboard.is_pressed('a'):
            sideways_vel += sideways_step
        # CMD DOWNWARD
        if keyboard.is_pressed('o'):
            height += height_step
        # CMD UPWARD
        if keyboard.is_pressed('p'):
            height -= height_step
        if keyboard.is_pressed('k'):
            kill_flag = True

        if kill_flag:
            print('KILL')
            commander.send_stop_setpoint()
            return



        commander.send_hover_setpoint(vx=forward_vel, vy=sideways_vel, yawrate=yaw_rate, zdistance=height)

        time.sleep(0.01)


