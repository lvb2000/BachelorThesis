//
// Created by Lukas von Briel on 21.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_MAPPING_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_MAPPING_H

#include "App_defines.h"
#include "App_parameter.h"
#include "stdbool.h"
#include "queue_drone.h"
#include "objects.h"

typedef struct MappingData_t{
    bool firstActive;
    bool firstInactive;
    enum Control_States *Control_State;
    struct ToF_Sensors_Data_t *ToF_Sensor_Data;
    enum Directions *MovingObjectDirection;

    // Mapping Data
    Queue_t distanceQueueFront;
    Queue_t distanceQueueLeft;
    Queue_t distanceQueueRight;
    Queue_t distanceQueueBack;

    float *yaw_rate;
}MappingData_t;

void mapping_main(MappingData_t *MappingData,ObjectsData_t *ObjectData);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_MAPPING_H
