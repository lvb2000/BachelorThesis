//
// Created by Lukas von Briel on 20.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_OBJECTS_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_OBJECTS_H

#include "App_defines.h"
#include "App_parameter.h"
#include <stdbool.h>

typedef struct ObjectsData_t{
    float *ToF_Sensor_Data;
    enum Control_States *Control_State;

    // Object Properties
    bool isObject;
    bool isWall;
    bool isCeilingOrGround;
    float ObjectDistance;
    int ObjectSize;
    int CenterCol;
    int CenterRow;
    int closestCol;

    int Object[NR_OF_PIXELS];
}ObjectsData_t;

void objects_main(ObjectsData_t *ObjectsData);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_OBJECTS_H