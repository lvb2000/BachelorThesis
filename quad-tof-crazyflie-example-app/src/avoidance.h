//
// Created by Lukas von Briel on 16.05.23.
//
#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_AVOIDANCE_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_AVOIDANCE_H

#include <stdbool.h>

#include "App_parameter.h"
#include <stdbool.h>

typedef struct AvoidanceData_t{
    bool firstActive;
    bool firstInactive;

    bool areaInit;
    int lastWidestArea;
    int AbortCounter;

    struct ToF_Sensors_Data_t *ToF_Sensor_Data;
    enum Control_States *Control_State;
    enum Directions *MovingObjectDirection;

    float *forwardVel;
    float *sidewaysVel;
    float *yawRate;
}AvoidanceData_t;

void avoidance_main(AvoidanceData_t *AvoidanceData);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_AVOIDANCE_H
