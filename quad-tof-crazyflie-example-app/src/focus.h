//
// Created by Lukas von Briel on 21.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_FOCUS_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_FOCUS_H

#include "App_defines.h"
#include "App_parameter.h"
#include "stdbool.h"
#include "objects.h"

typedef struct FocusData_t{
    bool firstActive;
    bool firstInactive;
    enum Control_States *Control_State;
    struct ToF_Sensors_Data_t *ToF_Sensor_Data;
    enum Directions *MovingObjectDirection;

    // PID Controller
    unsigned int lastMeasurementTimestamp;
    float lastErr;
    float integralErr;
    float k;
    float kd;
    float ki;
    float *yawRate;
}FocusData_t;

void focus_main(FocusData_t *FocusData,ObjectsData_t *ObjectData);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_FOCUS_H
