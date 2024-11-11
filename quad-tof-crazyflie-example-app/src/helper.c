//
// Created by Lukas von Briel on 16.05.23.
//
#include <math.h>
#include <stdint.h>
#include "helper.h"
#include "stdbool.h"
#include <stdlib.h>
#include "App_defines.h"
#include "debug.h"

#include "string.h"

#include "FreeRTOS.h"
#include "task.h"

int columnInDegree[8]={22,16,10,3,-3,-10,-16,-22};

double Convert(double degree){
    double pi = 3.14159;
    return(degree * (pi/180));
}

float Convertf(float degree){
    float pi = 3.14159f;
    return(degree * (pi/180.0f));
}

int convert_column_in_degree(int col){
    return columnInDegree[col];
}

int count_negatives(float *array,int length){
    int result = 0;
    for(int i=0;i<length;i++){
        if (array[i]==-1){
            result += 1;
        }
    }
    return result;
}

void calculate_mean_and_min(float *array, int length,int numberOfInvalid,double* resultingMean,double* resultingMin){
    float sum = 0;
    float min = INT16_MAX;
    for(int i = 0;i<length;i++){
        float value = array[i];
        if(value==-1){
            continue;
        }else {
            if (value < min) {
                min = value;
            }
            sum += value;
        }
    }
    *resultingMean = (double)sum/(double)(length-numberOfInvalid);
    *resultingMin = (double)min;
}

double distance2PoC4(double distance){
    double PoC = 0;
    if (distance<=2){
        PoC = -0.25* pow(distance,2)+1;
    }
    return PoC;
}

double distance2PoC9(double distance){
    double PoC = 0;
    if (distance <= 2.5){
        PoC = -0.111111 * pow(distance, 2) + 1 + 0.05;
    }
    if (PoC>1) PoC=1;
    return PoC;
}

double maximum(double *array,int start,int end){
    double max = 0;
    for (int i=start;i<end;i++){
        double value = array[i];
        if (value>max){
            max=value;
        }
    }
    return max;
}

void min_max(const double *array,int length,double *input_min,int *input_argmin,double *input_max,int *input_argmax){
    double max = 0;
    double min = 1;
    int argmin = 0;
    int argmax = 0;
    for(int i=0;i<length;i++){
        double value = array[i];
        if (value!=-1){
            if (value>max){
                max=value;
                argmax = i;
            }
            if(value<min){
                min=value;
                argmin = i;
            }
        }
    }
    *input_min = min;
    *input_argmin = argmin;
    *input_max = max;
    *input_argmax = argmax;
}

float get_log_variable(logVarId_t logId) { return logGetFloat(logId); }

int convert_area_in_degree(int area){return (int)(11.25*area-5.625);}

void init_array_with_zero(int *array,int length){
    for (int i =0;i<length;i++){
        array[i]=0;
    }
}

void minimum(float *array,int length,float *input_min,int *input_argmin,bool remove){
    float min = 3;
    int argmin = 32;
    for(int i=0;i<length;i++){
        float value = array[i];
        if(value!=-1){
            if (value<min){
                min = value;
                argmin = i;
            }
        }else{
            if(remove) array[i] = 3;
        }
    }
    *input_min = min;
    *input_argmin = argmin;
}

float rectdist=1;

float computeAproachingAngle(float ref1,float ref2){
    float a = 0.0982f;
    float c = ref1/ref2;
    float droneAngle = -1;
    float firstStep = -2.0f*c*cosf(a)+powf(sinf(a),2.0f)+powf(cosf(a),2.0f)+c*c;
    if (firstStep>0){
        float secondStep=sinf(a)/ sqrtf(firstStep);
        if(fabsf(secondStep)<1){
            droneAngle = acosf(secondStep);
        }
    }
    if(droneAngle!=-1) rectdist=ref1*cosf(droneAngle);
    return droneAngle;
}

float getNextPixelDistance(float droneAngle){
    return (rectdist/cosf(droneAngle+0.1963f));
}

float pythagros(float x1,float x2,float y1,float y2){
    return sqrtf(powf(x1-x2,2.0f)+powf(y1-y2,2.0f));
}

void print_matrixf(float *array){
    for(int i=0;i<NR_OF_PIXELS;i++){
        if(i%8==0 && i!=0) DEBUG_PRINT("\n");
        DEBUG_PRINT("%f \t",(double)array[i]);
    }
    DEBUG_PRINT("\n");
}

void print_matrix(int *array){
    for(int i=0;i<NR_OF_PIXELS;i++){
        if(i%8==0 && i!=0) DEBUG_PRINT("\n");
        DEBUG_PRINT("%i \t",array[i]);
    }
    DEBUG_PRINT("\n");
}

void print_array_double(const double *array,int length){
    for(int i=0;i<length;i++){
        if(i%8==0 && i!=0) DEBUG_PRINT("\n");
        DEBUG_PRINT("%f \t",array[i]);
    }
    DEBUG_PRINT("\n");
}
