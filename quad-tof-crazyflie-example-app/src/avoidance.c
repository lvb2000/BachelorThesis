//
// Created by Lukas von Briel on 16.05.23.
//

#include <string.h>
#include <stdint.h>

#include "avoidance.h"
#include "helper.h"
#include "App_defines.h"
#include "App_parameter.h"

#include "debug.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"

static void closest_area_avoidance(AvoidanceData_t *AvoidanceData);
static void PoC(AvoidanceData_t *AvoidanceData,double *highestPoC,double *smallestPoC,int *widestArea,double *lastWidestAreaPoC);
static void fill_sensor_data(AvoidanceData_t *AvoidanceData, double *PoC);
static void smooth_array(double *PoC);
static int regularization(int widestArea,double highestPoC,double smallestPoC,int lastWidestArea,double lastWidestAreaPoc);

void avoidance_main(AvoidanceData_t *AvoidanceData){
    if (*(AvoidanceData->Control_State) == focus){
        if(AvoidanceData->firstActive){
            DEBUG_PRINT("Avoidance in all directions is active.\n");
            AvoidanceData->firstActive = false;
            AvoidanceData->firstInactive = true;
        }
        closest_area_avoidance(AvoidanceData);
        if(!AvoidanceData->areaInit){
            AvoidanceData->areaInit = true;
        }
    }else{
        if (AvoidanceData->firstInactive){
            DEBUG_PRINT("Basic avoidance in all directions is inactive.\n");
            AvoidanceData->firstActive = true;
            AvoidanceData->firstInactive = false;
        }
    }
}

static void closest_area_avoidance(AvoidanceData_t *AvoidanceData){
    double highestPoC,smallestPoC,lastWidestAreaPoc;
    int widestArea;
    PoC(AvoidanceData,&highestPoC,&smallestPoC,&widestArea,&lastWidestAreaPoc);
    //vTaskDelay(10);
    //DEBUG_PRINT("Avoidance:\n");
    //DEBUG_PRINT("Highest PoC: %f, Smallest PoC: %f, last widest Area PoC: %f, widest Area: %i, last widest Area: %i\n",highestPoC,smallestPoC,lastWidestAreaPoc,widestArea,AvoidanceData->lastWidestArea);
    if (AvoidanceData->areaInit){
        widestArea = regularization(widestArea,highestPoC,smallestPoC,AvoidanceData->lastWidestArea,lastWidestAreaPoc);
        AvoidanceData->lastWidestArea = widestArea;
    }else{
        AvoidanceData->areaInit = true;
    }
    if (highestPoC > 0.8 && smallestPoC < 0.98){
        //if (MODE==1) *AvoidanceData->yawRate = 0;
        int direction = convert_area_in_degree(widestArea);
        if (direction <0){
            direction +=360;
        }
        float avoidanceSpeed = DEFAULT_MAX_SPEED;
        if(highestPoC <0.8){
            avoidanceSpeed = DEFAULT_SPEED;
        }
        if (smallestPoC>0.85){
            avoidanceSpeed = DEFAULT_CLOSE_SPEED;
        }
        double directionRad = Convert(direction);
        double factorX = cos(directionRad);
        double factorY = sin(directionRad);
        float SpeedX = avoidanceSpeed*(float)factorX;
        float SpeedY = avoidanceSpeed*(float)factorY;
        if (fabsf(SpeedX)<0.1f){SpeedX=0;}
        if (fabsf(SpeedY)<0.1f){SpeedY=0;}
        *AvoidanceData->forwardVel = SpeedX;
        *AvoidanceData->sidewaysVel = SpeedY;
        //DEBUG_PRINT("Forward Velocity: %f\n",(double)SpeedX);
        //DEBUG_PRINT("Sideways Velocity: %f\n",(double)SpeedY);
        //vTaskDelay(10);
    }else{
        if(AvoidanceData->AbortCounter>=ABORT_COUNTER_LIMIT){
            if (MODE==1) {
                //*AvoidanceData->yawRate = 20;
                AvoidanceData->AbortCounter = 0;
            }
            else {
                *AvoidanceData->Control_State = search;
                AvoidanceData->AbortCounter = 0;
            }
        }else{
            AvoidanceData->AbortCounter++;
        }
        *AvoidanceData->forwardVel = 0;
        *AvoidanceData->sidewaysVel = 0;
        AvoidanceData->areaInit = false;
    }
}

static void PoC(AvoidanceData_t *AvoidanceData,double *highestPoC,double *smallestPoC,int *widestArea,double *lastWidestAreaPoC){
    double PoC[NR_OF_AREAS] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    fill_sensor_data(AvoidanceData, PoC);
    //vTaskDelay(1000);
    //DEBUG_PRINT("Before Smoothing:\n");
    //print_array_double(PoC,NR_OF_AREAS);
    //vTaskDelay(1000);
    smooth_array(PoC);
    //vTaskDelay(1000);
    //DEBUG_PRINT("After Smoothing:\n");
    //print_array_double(PoC,NR_OF_AREAS);
    //vTaskDelay(1000);
    int argmax;
    min_max(PoC,NR_OF_AREAS,smallestPoC,widestArea,highestPoC,&argmax);
    int opposite = argmax-16;
    if (opposite<0) opposite = 32 + opposite;
    if(PoC[opposite]<0.7){
        *widestArea = opposite;
        *smallestPoC = 0.4;
    }
    *lastWidestAreaPoC = PoC[AvoidanceData->lastWidestArea];
}


static void fill_sensor_data(AvoidanceData_t *AvoidanceData, double *PoC){
    for(int i = 0;i<NR_OF_SENSORS;i++){
        float matrix[NR_OF_PIXELS];
        int sensorAngle;
        switch(i){
            case ToF_RIGHT:
                sensorAngle = ToF_RIGHT_DEGREE;
                memcpy(matrix,&AvoidanceData->ToF_Sensor_Data->RightDistance[0], sizeof(matrix));
                break;
            case ToF_BACK:
                sensorAngle = ToF_BACK_DEGREE;
                memcpy(matrix,&AvoidanceData->ToF_Sensor_Data->BackDistance[0], sizeof(matrix));
                break;
            case ToF_LEFT:
                sensorAngle = ToF_LEFT_DEGREE;
                memcpy(matrix,&AvoidanceData->ToF_Sensor_Data->LeftDistance[0], sizeof(matrix));
                break;
            default:
                sensorAngle = ToF_FRONT_DEGREE;
                memcpy(matrix,&AvoidanceData->ToF_Sensor_Data->FrontDistance[0], sizeof(matrix));
                break;
        }
        //DEBUG_PRINT("PoC Sensor: %i\n",sensorAngle);
        for (int j = 0; j<NR_OF_ROWS;j++){
            int alpha = convert_column_in_degree(j);
            //DEBUG_PRINT("\t alpha: %i\n",alpha);
            int colAngle = sensorAngle+alpha;
            if (colAngle<0){
                colAngle += 360;
            }
            colAngle %= 360;
            int PoCIndex = (int)(colAngle/SECTOR_ANGLE);
            //DEBUG_PRINT("\t PoC Index: %i\n",PoCIndex);
            if (PoCIndex>NR_OF_AREAS-1){
                PoCIndex = NR_OF_AREAS-1;
            }
            float array[COLUMN_LENGTH];
            memcpy(array,&matrix[j*COLUMN_LENGTH], sizeof(array));
            int numberOfInvalid = count_negatives(array, COLUMN_LENGTH);
            //DEBUG_PRINT("\t number of invalid: %i\n",numberOfInvalid);
            if (numberOfInvalid < 7){
                double mean,min;
                calculate_mean_and_min(array,COLUMN_LENGTH,numberOfInvalid,&mean,&min);
                //DEBUG_PRINT("\t mean: %f,min: %f\n",mean,min);
                double minPoC,meanPoC;
                if((int)*AvoidanceData->MovingObjectDirection==i){
                    minPoC = distance2PoC9(min);
                    meanPoC = distance2PoC9(mean);
                }else{
                    minPoC = distance2PoC4(min);
                    meanPoC= distance2PoC4(mean);
                }
                //DEBUG_PRINT("\t mean PoC: %f,min PoC: %f\n",meanPoC,minPoC);
                double PoCValue = 0.9*minPoC+0.1*meanPoC;
                double previousValue = PoC[PoCIndex];
                //DEBUG_PRINT("\t previous PoC Value: %f\n",previousValue);
                if (previousValue != -1){
                    PoCValue = (previousValue+PoCValue)/2;
                }
                PoC[PoCIndex]=PoCValue;
                //vTaskDelay(20);
            }else{
                double PoCValue = DEFAULT_POC_VALUE;
                double previousValue = PoC[PoCIndex];
                if (previousValue != -1){
                    PoCValue = (previousValue+DEFAULT_POC_VALUE)/2;
                }
                PoC[PoCIndex]=PoCValue;
            }
        }
        //vTaskDelay(10);
    }
}

static void smooth_array(double *PoC){
    int arraySize = (int)(NR_OF_AREAS+2*SMOOTHING);
    double extendedPoC[arraySize];
    for (int i=0;i<SMOOTHING;i++){
        extendedPoC[i]=PoC[NR_OF_AREAS-SMOOTHING+i];
        extendedPoC[arraySize-SMOOTHING+i]=PoC[i];
    }
    //vTaskDelay(1000);
    //DEBUG_PRINT("Extended PoC:\n");
    //print_array_double(extendedPoC,arraySize);
    //vTaskDelay(2000);
    memcpy(&extendedPoC[SMOOTHING],&PoC[0], 8*NR_OF_AREAS);
    //vTaskDelay(1000);
    //DEBUG_PRINT("Extended PoC Copy:\n");
    //print_array_double(extendedPoC,arraySize);
    //vTaskDelay(2000);

    //DEBUG_PRINT("Smoothing: \n");
    for (int i=SMOOTHING;i<(NR_OF_AREAS+SMOOTHING);i++){
        //DEBUG_PRINT("\t");
        //DEBUG_PRINT("%i = ",i);
        if (PoC[i-SMOOTHING]!=-1){
            double max = maximum(extendedPoC,i-SMOOTHING,i+SMOOTHING);
            //DEBUG_PRINT("%f, ",max);
            PoC[i-SMOOTHING]=(PoC[i-SMOOTHING]+2*max)/3;
        }
        //DEBUG_PRINT("\n");
    }
    //vTaskDelay(1000);
    //DEBUG_PRINT("Extended PoC Copy:\n");
    //print_array_double(extendedPoC,arraySize);
    //vTaskDelay(2000);
}

static int regularization(int widestArea,double highestPoC,double smallestPoC,int lastWidestArea,double lastWidestAreaPoc){
    double threshold = 0.6;
    if (smallestPoC<0.7){
        threshold = 0.35;
    }else if (smallestPoC<0.8) {
        threshold = 0.3;
    }else if (smallestPoC<0.95){
        threshold = 0.21;
    }else if (smallestPoC<0.98){
        threshold = 0.1;
    }else if (smallestPoC<=1){
        threshold =0.075;
    }
    if (highestPoC<0.87){
        threshold = 0.35;
    }
    if(fabs(lastWidestAreaPoc-smallestPoC)<threshold){
        return lastWidestArea;
    }else{
        return widestArea;
    }
}