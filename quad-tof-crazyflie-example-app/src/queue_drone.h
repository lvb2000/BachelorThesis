//
// Created by Lukas von Briel on 21.05.23.
//

#ifndef QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_QUEUE_DRONE_H
#define QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_QUEUE_DRONE_H

// D stands for Distance and P for Position

#define D_QUEUE_SIZE 20
#define P_QUEUE_SIZE 40

#include "stdbool.h"

typedef struct Queue_t{

    float array[D_QUEUE_SIZE];
    int front;
    int rear;
    int itemCount;
}Queue_t;


float peek_last(Queue_t queue);
bool isEmpty(Queue_t queue);
bool isFull(Queue_t queue);
void insert(Queue_t *queue, float data);
void removeData(Queue_t *queue);
void clear(Queue_t *queue);
float mean_Speed_Between_Objects(Queue_t queue);
void print_queue(Queue_t queue);

#endif //QUAD_TOF_CRAZYFLIE_EXAMPLE_APP_QUEUE_DRONE_H
