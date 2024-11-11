
# enable app support
CXXFLAGS="-g -NDEBUG"

APP=1
APP_STACKSIZE=5000

VPATH += src/
PROJ_OBJ += test_tof.o
PROJ_OBJ += vl53l5cx_api.o
PROJ_OBJ += ToFplatform.o
PROJ_OBJ += I2C_expander.o
PROJ_OBJ += commander_interface.o
PROJ_OBJ += avoidance.o
PROJ_OBJ += helper.o
PROJ_OBJ += objects.o
PROJ_OBJ += mapping.o
PROJ_OBJ += focus.o
PROJ_OBJ += queue_drone.o

INCLUDES += -Isrc

CRAZYFLIE_BASE=../crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile