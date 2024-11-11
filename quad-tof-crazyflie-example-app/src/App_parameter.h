//
// Created by Lukas von Briel on 20.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_APP_PARAMETER_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_APP_PARAMETER_H

#include <stdint.h>
#include "App_defines.h"

typedef struct ToF_Sensors_Data_t {
    float FrontDistance[NR_OF_PIXELS];
    float BackDistance[NR_OF_PIXELS];
    float LeftDistance[NR_OF_PIXELS];
    float RightDistance[NR_OF_PIXELS];
} ToF_Sensors_Data_t;

enum Control_States {
    focus,
    search
}Control_States;

enum Directions {
    Right,
    Back,
    Left,
    Front,
    None
}Directions;

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_APP_PARAMETER_H
