//
// Created by Lukas von Briel on 21.05.23.
//

#include "focus.h"
#include "debug.h"
#include "string.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "helper.h"

static void yaw_controller(FocusData_t *FocusData,ObjectsData_t *ObjectData);
static void PID_Controller(FocusData_t *FocusData,ObjectsData_t *ObjectData);

void focus_main(FocusData_t *FocusData,ObjectsData_t *ObjectData){
    if (*(FocusData->Control_State) == focus){
        if(FocusData->firstActive){
            DEBUG_PRINT("Focus is active.\n");
            FocusData->firstActive = false;
            FocusData->firstInactive = true;
        }
        yaw_controller(FocusData,ObjectData);
    }else{
        if (FocusData->firstInactive){
            DEBUG_PRINT("Focus is inactive.\n");
            FocusData->firstActive = true;
            FocusData->firstInactive = false;
        }
    }
}

static void yaw_controller(FocusData_t *FocusData,ObjectsData_t *ObjectData){
    int i = (int)(*(FocusData->MovingObjectDirection));
    float matrix[NR_OF_PIXELS];
    bool err = false;
    switch (i) {
        case ToF_RIGHT:
            err = false;
            memcpy(matrix, &FocusData->ToF_Sensor_Data->RightDistance[0], sizeof(matrix));
            break;
        case ToF_BACK:
            err = false;
            memcpy(matrix, &FocusData->ToF_Sensor_Data->BackDistance[0], sizeof(matrix));
            break;
        case ToF_LEFT:
            err = false;
            memcpy(matrix, &FocusData->ToF_Sensor_Data->LeftDistance[0], sizeof(matrix));
            break;
        case ToF_FRONT:
            err = false;
            memcpy(matrix, &FocusData->ToF_Sensor_Data->FrontDistance[0], sizeof(matrix));
            break;
        default:
            err = true;
            DEBUG_PRINT("Focus has been started but no Direction has been set.\n");
            break;
    }
    if(!err){
        ObjectData->ToF_Sensor_Data = matrix;
        objects_main(ObjectData);
        //DEBUG_PRINT("Wall: %i; Ceiling: %i\n",(int)ObjectData->isWall,(int)ObjectData->isCeilingOrGround);
        if(ObjectData->isObject){
            PID_Controller(FocusData,ObjectData);
        }
    }
}

static void PID_Controller(FocusData_t *FocusData,ObjectsData_t *ObjectData){
    float err = 0;
    if(FocusData->lastMeasurementTimestamp!=0){
        //print_matrix(ObjectData->Object);
        //vTaskDelay(1000);
        //DEBUG_PRINT("Object Center Col: %i\n",ObjectData->CenterCol);
        switch(ObjectData->CenterCol){
            case 7:
                err=-4;
                break;
            case 6:
                err=-2.5f;
                break;
            case 5:
                err=-1;
                break;
            case 2:
                err=1;
                break;
            case 1:
                err=2.5f;
                break;
            case 0:
                err=4;
                break;
            default:
                err=0;
                break;
        }
        float dtErr = (err-FocusData->lastErr)/((float)(T2M(xTaskGetTickCount())-FocusData->lastMeasurementTimestamp)/1000);
        FocusData->integralErr += err*((float)(T2M(xTaskGetTickCount())-FocusData->lastMeasurementTimestamp)/1000);

        if (ObjectData->CenterCol == 3 || ObjectData->CenterCol == 4) FocusData->integralErr = 0;

        float yaw_rate = FocusData->k*err + FocusData->kd*dtErr +FocusData->ki*FocusData->integralErr;
        DEBUG_PRINT("Object Center: %i\n",ObjectData->CenterCol);
        DEBUG_PRINT("Focus Yaw Rate: %f\n",(double)yaw_rate);

        if (yaw_rate>80) yaw_rate=80;
        if(yaw_rate<-80) yaw_rate=-80;

        *FocusData->yawRate = yaw_rate;
        //DEBUG_PRINT("Focus Yaw Rate: %f\n",(double)yaw_rate);
    }
    FocusData->lastMeasurementTimestamp = T2M(xTaskGetTickCount());
}