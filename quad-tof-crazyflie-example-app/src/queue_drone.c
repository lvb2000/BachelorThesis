//
// Created by Lukas von Briel on 21.05.23.
//
#include "queue_drone.h"
#include "helper.h"
#include "math.h"
#include "debug.h"


float peek_last(Queue_t queue){
    return queue.array[queue.rear];
}

bool isEmpty(Queue_t queue){
    return queue.itemCount ==0;
}
bool isFull(Queue_t queue){
    return queue.itemCount == D_QUEUE_SIZE;
}
void insert(Queue_t *queue, float data){
    if(queue->rear == D_QUEUE_SIZE - 1){
        queue->rear = -1;
    }
    queue->array[++queue->rear]=data;
    queue->itemCount++;
}
void removeData(Queue_t *queue){
    queue->front++;
    if(queue->front == D_QUEUE_SIZE){
        queue->front=0;
    }
    queue->itemCount--;
}
void clear(Queue_t *queue){
    queue->front=0;
    queue->rear=-1;
    queue->itemCount=0;
}
float mean_Speed_Between_Objects(Queue_t queue){
    float mean=0;
    int i=queue.front;
    do{
        float x1 = queue.array[i];
        i++;
        if(i==D_QUEUE_SIZE) i=0;
        int next = i+1;
        if(next==D_QUEUE_SIZE) next=0;
        float x2 = queue.array[next];
        float dist=fabsf(x1-x2);
        if(dist<0.5f){
            mean += dist;
        }
    }while(i!=queue.front);
    mean/=((D_QUEUE_SIZE-1)*0.05f);
    return mean;
}


void print_queue(Queue_t queue){
    int i=queue.front;
    DEBUG_PRINT("Queue: ");
    do{
        float value = queue.array[i];
        i++;
        if(i==D_QUEUE_SIZE) i=0;
        DEBUG_PRINT("%f, ",(double)value);
    }while(i!=queue.front);
    DEBUG_PRINT("\n");
    DEBUG_PRINT("Array: ");
    for(int j=0;j<D_QUEUE_SIZE;j++){
        DEBUG_PRINT("%f, ",(double)queue.array[j]);
    }
    DEBUG_PRINT("\n");
}