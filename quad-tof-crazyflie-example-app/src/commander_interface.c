//
// Created by Lukas von Briel on 16.05.23.
//

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "commander_interface.h"
#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
    setpoint->mode.z = modeAbs;
    setpoint->position.z = z;


    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = yawrate;


    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = vx;
    setpoint->velocity.y = vy;

    setpoint->velocity_body = true;
}

void commander(CommanderData_t *commanderData){
    static setpoint_t setpoint;
    uint32_t delta = T2M(xTaskGetTickCount()-commanderData->startTick);

    if (commanderData->flying){
        if (commanderData->first){
            DEBUG_PRINT("Flying is activated.\n");
            commanderData->first = false;
        }
        if((float)delta>commanderData->flightTime){
            DEBUG_PRINT("Drone is landing.\n");
            setHoverSetpoint(&setpoint,0,0,commanderData->defaultHeight/2,0);
            commanderSetSetpoint(&setpoint, 3);
            vTaskDelay(M2T(1000));
            setHoverSetpoint(&setpoint,0,0,0.1f,0);
            commanderSetSetpoint(&setpoint, 3);
            vTaskDelay(M2T(500));
            setHoverSetpoint(&setpoint,0,0,0,0);
            commanderSetSetpoint(&setpoint, 3);
            vTaskDelay(M2T(500));
            DEBUG_PRINT("Landed safely\n");
            commanderData->flying = false;
        }else{
            setHoverSetpoint(&setpoint,commanderData->forwardVel,commanderData->sidewaysVel,commanderData->height,commanderData->yawRate);
            commanderSetSetpoint(&setpoint, 3);
        }
    }else{
        if (commanderData->first){
            DEBUG_PRINT("Flying is deactivated.\n");
            commanderData->first = false;
        }
    }
}



