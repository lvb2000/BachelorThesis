//
// Created by Lukas von Briel on 20.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_APP_DEFINES_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_APP_DEFINES_H

#include "log.h"

// general
// 0 = mapping + focus ; 1 = only avoidance
#define MODE 0

#define NR_OF_SENSORS 4
#define NR_OF_PIXELS 64
#define NR_OF_ROWS 8
#define NR_OF_COLS 8
#define COLUMN_LENGTH 8

#define ToF_RIGHT 0
#define ToF_BACK 1
#define ToF_LEFT 2
#define ToF_FRONT 3

#define ToF_RIGHT_DEGREE 270
#define ToF_BACK_DEGREE 180
#define ToF_LEFT_DEGREE 90
#define ToF_FRONT_DEGREE 0

#define ToF_EMPTY 0
#define ToF_FULL 15
#define ToF_RIGHT_MASK 0b00001000
#define ToF_BACK_MASK 0b00000100
#define ToF_LEFT_MASK 0b00000010
#define ToF_FRONT_MASK 0b00000001


// controller
#define FLYING true
#define DEFAULT_HEIGHT 0.7        // in [m]
#define FLIGHT_TIME 120          // in [s]
#define START_TIME 5            // in [s]

// mapping
#define WALL_PUNISH 9
#define MOVING_OBJECT_DISTANCE_THRESHOLD 1.5f //1.4f

// avoidance
#define ABORT_COUNTER_LIMIT 200
#define NR_OF_AREAS 32
#define SECTOR_ANGLE (360/NR_OF_AREAS)
#define DEFAULT_POC_VALUE 0.4
#define SMOOTHING 4
#define DEFAULT_MAX_SPEED 0.8f
#define DEFAULT_SPEED 0.6f
#define DEFAULT_CLOSE_SPEED 0.3f

// objects
#define COLUMN_OFFSET 0.05f

logVarId_t logIdStateEstimateYaw;

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_APP_DEFINES_H
