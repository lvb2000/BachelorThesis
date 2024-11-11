//
// Created by Lukas von Briel on 16.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_COMMANDER_INTERFACE_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_COMMANDER_INTERFACE_H

#include <stdbool.h>

typedef struct CommanderData_t{
    bool flying;

    float defaultHeight;

    float yawRate;
    float forwardVel;
    float sidewaysVel;
    float height;

    float flightTime;
    uint32_t startTick;
    float startTime;

    bool first;
}CommanderData_t;

void commander(CommanderData_t *commanderData);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_COMMANDER_INTERFACE_H
