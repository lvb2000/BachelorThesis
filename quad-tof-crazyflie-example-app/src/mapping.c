//
// Created by Lukas von Briel on 21.05.23.
//

#include "mapping.h"
#include "debug.h"
#include "App_defines.h"
#include "App_parameter.h"
#include "math.h"
#include "string.h"
#include "helper.h"

#include "FreeRTOS.h"
#include "task.h"

static float getYaw() { return logGetFloat(logIdStateEstimateYaw); }
static void mapObjectMovement(MappingData_t *MappingData,ObjectsData_t *ObjectData);
static bool findMovingObject(Queue_t queue);

void mapping_main(MappingData_t *MappingData,ObjectsData_t *ObjectData){
    if (*(MappingData->Control_State) == search){
        vTaskDelay(2);
        if(MappingData->firstActive){
            DEBUG_PRINT("Mapping is active.\n");
            *MappingData->yaw_rate = 25;
            MappingData->firstActive = false;
            MappingData->firstInactive = true;
        }
        mapObjectMovement(MappingData,ObjectData);
    }else{
        vTaskDelay(2);
        if (MappingData->firstInactive){
            DEBUG_PRINT("Mapping is inactive.\n");
            clear(&MappingData->distanceQueueFront);
            clear(&MappingData->distanceQueueLeft);
            clear(&MappingData->distanceQueueRight);
            clear(&MappingData->distanceQueueBack);
            MappingData->firstActive = true;
            MappingData->firstInactive = false;
        }
    }
}

static void mapObjectMovement(MappingData_t *MappingData,ObjectsData_t *ObjectData) {
    for (int i = 0; i < NR_OF_SENSORS; i++) {
        float matrix[NR_OF_PIXELS];
        int sensorAngle;
        switch (i) {
            case ToF_RIGHT:
                sensorAngle = ToF_RIGHT_DEGREE;
                memcpy(matrix, &MappingData->ToF_Sensor_Data->RightDistance[0], sizeof(matrix));
                break;
            case ToF_BACK:
                sensorAngle = ToF_BACK_DEGREE;
                memcpy(matrix, &MappingData->ToF_Sensor_Data->BackDistance[0], sizeof(matrix));
                break;
            case ToF_LEFT:
                sensorAngle = ToF_LEFT_DEGREE;
                memcpy(matrix, &MappingData->ToF_Sensor_Data->LeftDistance[0], sizeof(matrix));
                break;
            default:
                sensorAngle = ToF_FRONT_DEGREE;
                memcpy(matrix, &MappingData->ToF_Sensor_Data->FrontDistance[0], sizeof(matrix));
                break;
        }
        ObjectData->ToF_Sensor_Data = matrix;
        objects_main(ObjectData);
        // DEBUG_PRINT("Sensor: %i;Object: %i; Wall: %i, Center Column: %i,Center Row: %i,Ceiling or Ground: %i,Object Distance: %f,Closest Col: %i\n",i,(int)ObjectData->isObject,(int)ObjectData->isWall,ObjectData->CenterCol,ObjectData->CenterRow,(int)ObjectData->isCeilingOrGround,(double)ObjectData->ObjectDistance,ObjectData->closestCol);
        //print_matrixf(matrix);
        //print_matrix(ObjectData->Object);
        if (ObjectData->isObject) {
            float droneYaw = getYaw();
            //DEBUG_PRINT("Drone Yaw: %f",(double)droneYaw);
            int alpha = convert_column_in_degree(ObjectData->CenterCol);
            //DEBUG_PRINT("Alpha: %f",(double)alpha);
            float angle = Convertf(droneYaw + (float) alpha + (float) sensorAngle);
            //DEBUG_PRINT("Object Angle: %f",(double)angle);
            float x = cosf(angle) * ObjectData->ObjectDistance;
            float y = sinf(angle) * ObjectData->ObjectDistance;
            //DEBUG_PRINT("x: %f;y: %f\n",(double)x,(double)y);
            if (fabsf(x) < 4 && fabsf(y) < 4) {
                switch (i) {
                    case ToF_RIGHT:
                        if (isFull(MappingData->distanceQueueRight)) removeData(&MappingData->distanceQueueRight);
                        insert(&MappingData->distanceQueueRight, ObjectData->ObjectDistance);
                        if (isFull(MappingData->distanceQueueRight))
                            if(findMovingObject(MappingData->distanceQueueRight)){
                                DEBUG_PRINT("Right moving\n");
                                *MappingData->Control_State = focus;
                                *MappingData->MovingObjectDirection = Right;
                                *MappingData->yaw_rate = 0;
                            }
                        break;
                    case ToF_BACK:
                        if (isFull(MappingData->distanceQueueBack)) removeData(&MappingData->distanceQueueBack);
                        insert(&MappingData->distanceQueueBack, ObjectData->ObjectDistance);
                        if (isFull(MappingData->distanceQueueBack))
                            if(findMovingObject(MappingData->distanceQueueBack)){
                                DEBUG_PRINT("Back moving\n");
                                *MappingData->Control_State = focus;
                                *MappingData->MovingObjectDirection = Back;
                                *MappingData->yaw_rate = 0;
                            }
                        break;
                    case ToF_LEFT:
                        if (isFull(MappingData->distanceQueueLeft)) removeData(&MappingData->distanceQueueLeft);
                        insert(&MappingData->distanceQueueLeft, ObjectData->ObjectDistance);
                        if (isFull(MappingData->distanceQueueLeft))
                            if(findMovingObject(MappingData->distanceQueueLeft)){
                                DEBUG_PRINT("Left moving\n");
                                *MappingData->Control_State = focus;
                                *MappingData->MovingObjectDirection = Left;
                                *MappingData->yaw_rate = 0;
                            }
                        break;
                    default:
                        if (isFull(MappingData->distanceQueueFront)) removeData(&MappingData->distanceQueueFront);
                        insert(&MappingData->distanceQueueFront, ObjectData->ObjectDistance);
                        if (isFull(MappingData->distanceQueueFront))
                            if(findMovingObject(MappingData->distanceQueueFront)){
                                DEBUG_PRINT("Front moving\n");
                                *MappingData->Control_State = focus;
                                *MappingData->MovingObjectDirection = Front;
                                *MappingData->yaw_rate = 0;
                            }
                        break;
                }
            }
        } else {
            switch (i) {
                case ToF_RIGHT:
                    if (!isEmpty(MappingData->distanceQueueRight)) {
                        float lastEntry = peek_last(MappingData->distanceQueueRight);
                        for (int k=0;k<WALL_PUNISH;k++) {
                            if (isFull(MappingData->distanceQueueRight)) removeData(&MappingData->distanceQueueRight);
                            insert(&MappingData->distanceQueueRight, lastEntry);
                        }
                    }
                    break;
                case ToF_BACK:
                    if (!isEmpty(MappingData->distanceQueueBack)) {
                        float lastEntry = peek_last(MappingData->distanceQueueBack);
                        for (int k=0;k<WALL_PUNISH;k++) {
                            if (isFull(MappingData->distanceQueueBack)) removeData(&MappingData->distanceQueueBack);
                            insert(&MappingData->distanceQueueBack, lastEntry);
                        }
                    }
                    break;
                case ToF_LEFT:
                    if (!isEmpty(MappingData->distanceQueueLeft)) {
                        float lastEntry = peek_last(MappingData->distanceQueueLeft);
                        for (int k=0;k<WALL_PUNISH;k++) {
                            if (isFull(MappingData->distanceQueueLeft)) removeData(&MappingData->distanceQueueLeft);
                            insert(&MappingData->distanceQueueLeft, lastEntry);
                        }
                    }
                    break;
                default:
                    if (!isEmpty(MappingData->distanceQueueFront)) {
                        float lastEntry = peek_last(MappingData->distanceQueueFront);
                        for (int k=0;k<WALL_PUNISH;k++) {
                            if (isFull(MappingData->distanceQueueFront)) removeData(&MappingData->distanceQueueFront);
                            insert(&MappingData->distanceQueueFront, lastEntry);
                        }
                    }
                    break;
            }
        }
    }
}

static bool findMovingObject(Queue_t queue){
    float dis = mean_Speed_Between_Objects(queue);
    DEBUG_PRINT("dis: %f\n",(double)dis);
    if (dis>MOVING_OBJECT_DISTANCE_THRESHOLD) return true;
    else return false;
}
