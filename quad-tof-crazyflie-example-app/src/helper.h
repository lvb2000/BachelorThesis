//
// Created by Lukas von Briel on 16.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_HELPER_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_HELPER_H

#include "log.h"

double Convert(double degree);
float Convertf(float degree);
int convert_column_in_degree(int col);
int count_negatives(float *array, int length);
void calculate_mean_and_min(float *array, int length,int numberOfInvalid,double* resultingMean,double* resultingMin);
double distance2PoC4(double distance);
double distance2PoC9(double distance);
double maximum(double *array,int start,int end);
void min_max(const double *array,int length,double *input_min,int *input_argmin,double *input_max,int *input_argmax);
float get_log_variable(logVarId_t logId);
int convert_area_in_degree(int area);
void init_array_with_zero(int *array,int length);
void minimum(float *array,int length,float *input_min,int *input_argmin,bool remove);
float computeAproachingAngle(float ref1,float ref2);
float getNextPixelDistance(float droneAngle);
float pythagros(float x1,float x2,float y1,float y2);
void print_matrixf(float *array);
void print_matrix(int *array);
void print_array_double(const double *array,int length);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_HELPER_H
