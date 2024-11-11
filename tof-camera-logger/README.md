# Crazyflie Matrix ToF + Camera streaming example including keyboard velocity controls 

---
# Build and flash procedure
- Move to `app-tof-logger-tof` folder

- In `Makefile` adapt path `CRAZYFLIE_BASE` to your crazyflie firmware

- Run `make clean` to clean exisiting previous builds

- Run `make` to build 

- Put crazyflie into bootloader mode and run `make cload`

# Run Procedure
- Move to `python-app-image-tof-logger`
- Start crazyflie and connect computer to crazyflie wifi
- Run `sudo ~/anaconda3/envs/tof_camera_logger/bin/python3 main.py`
  - Now three OpenCv windows will open. One displaying the camera feed, one the matrix ToF feed and one acts as a 
  support for the multithreading.
  - **You now have 10 seconds before the drone takes off**
  - **Make sure you click onto the terminal window again, so that this is focused** 
  - If not, keyboard commands will 
  be registered by OpenCv windows and abort process, making drone fall out of the sky.
- Keyboard commands:
  - W, A, S, D command velocities as usual
  - Q, E command yaw rates 
  - O, P command height changes
  - K kills the drone making it fall out of the sky
- Aborting the programm
  - After you landed the drone or killed the drone, make sure to click an OpenCv window and press `ESC`, afterwards 
  pressing `CTRL-C` on the terminal to completely stop programm. This hustle comes from the OpenCv `WaitKey(0)` 
  function.
  