//
// Created by Lukas von Briel on 20.05.23.
//

#include <string.h>
#include <stdbool.h>
#include "math.h"

#include "objects.h"
#include "debug.h"
#include "helper.h"
#include "stdlib.h"


static void label_nearest_object(ObjectsData_t *ObjectsData);
static void recursive_cluster(const float *array,int *object,int closestPixel,float upperBound,int *size);
static int getCenter(ObjectsData_t *ObjectsData,char direction);
static int getOuterCenter(int rightMostPixelIdx,int leftMostPixelIdx);
static void detect_ceiling_or_floor(ObjectsData_t *ObjectsData);
static void detect_wall(ObjectsData_t *ObjectsData);
static void check_edge(ObjectsData_t *ObjectsData,int *edge);
static bool check_first_Col(ObjectsData_t *ObjectsData,int col,int *height,float *ref);
static bool check_second_Col(ObjectsData_t *ObjectsData,int col,int height,float *ref);
static void check_third_Col(ObjectsData_t *ObjectsData,int col,int height,float ref);

void objects_main(ObjectsData_t *ObjectsData){
    init_array_with_zero(ObjectsData->Object,NR_OF_PIXELS);
    label_nearest_object(ObjectsData);
    if (ObjectsData->ObjectSize>4 && ObjectsData->ObjectSize<55){
        detect_ceiling_or_floor(ObjectsData);
        if(!ObjectsData->isCeilingOrGround){
            detect_wall(ObjectsData);
            if(!ObjectsData->isWall){
                ObjectsData->isObject = true;
            }else{
                ObjectsData->isObject = false;
            }
        }else{
            ObjectsData->isObject = false;
        }
    }else{
        ObjectsData->isObject = false;
    }
}

static void label_nearest_object(ObjectsData_t *ObjectsData){
    float matrix[NR_OF_PIXELS];
    memcpy(matrix,ObjectsData->ToF_Sensor_Data, sizeof(matrix));
    int closestPixel;
    float closestValue;
    minimum(matrix,NR_OF_PIXELS,&closestValue,&closestPixel,true);
    ObjectsData->ObjectDistance = closestValue;
    ObjectsData->closestCol = closestPixel%8;
    float upperBound = closestValue + 0.3f;
    ObjectsData->ObjectSize = 0;
    recursive_cluster(matrix,ObjectsData->Object,closestPixel,upperBound,&ObjectsData->ObjectSize);
    ObjectsData->CenterRow = getCenter(ObjectsData,'r');
    ObjectsData->CenterCol = getCenter(ObjectsData,'c');
}
static void recursive_cluster(const float *array,int *object,int closestPixel,float threshold,int *size){
    if (closestPixel<0 || closestPixel>=NR_OF_PIXELS){
        return;
    }
    if (object[closestPixel]==0 && array[closestPixel]<threshold){
        object[closestPixel] = 1;
        *size = *size+1;
        recursive_cluster(array,object,closestPixel-1,threshold,size);
        recursive_cluster(array,object,closestPixel+1,threshold,size);
        recursive_cluster(array,object,closestPixel-COLUMN_LENGTH,threshold,size);
        recursive_cluster(array,object,closestPixel+COLUMN_LENGTH,threshold,size);
    }
}
static int getCenter(ObjectsData_t *ObjectsData,char direction) {

    int leftMostPixelIdx = 0;
    int rightMostPixelIdx = 7;
    bool leftBoarderReached = false;

    for(int i=0;i<NR_OF_COLS;i++){
        int sum = 0;
        for(int j=0;j<COLUMN_LENGTH;j++){
            if(direction=='c'){
                sum += ObjectsData->Object[i+j*8];
            }else if(direction=='r'){
                sum += ObjectsData->Object[i*8+j];
            }
        }
        if(sum>2 && !leftBoarderReached){
            leftMostPixelIdx = i;
            leftBoarderReached = true;
        }
        if(sum<=2 && leftBoarderReached){
            rightMostPixelIdx = i-1;
            break;
        }
    }
    //if(direction=='c'){
    //    DEBUG_PRINT("Left most Pixel: %i; Right most Pixel: %i\n",leftMostPixelIdx,rightMostPixelIdx);
    //}
    return getOuterCenter(rightMostPixelIdx, leftMostPixelIdx);
}

static int getOuterCenter(int rightMostPixelIdx,int leftMostPixelIdx){
    int sum = rightMostPixelIdx + leftMostPixelIdx;
    if (sum >=8){
        if (sum%2 == 1){
            sum +=1;
        }
    }else{
        if (sum%2 == 1){
            sum -=1;
        }
        if(sum<0) sum=0;
    }
    return (int)(sum/2);
}
static void detect_ceiling_or_floor(ObjectsData_t *ObjectsData){
    if (ObjectsData->CenterRow == 0 || ObjectsData->CenterRow == 7){
        ObjectsData->isCeilingOrGround = true;
    }else{
        ObjectsData->isCeilingOrGround = false;
    }
}
static void detect_wall(ObjectsData_t *ObjectsData){
    ObjectsData->isWall = false;
    if(ObjectsData->closestCol == 0 || ObjectsData->closestCol==7){
        int leftEdge[3]={0,1,2};
        int rightEdge[3]={7,6,5};
        check_edge(ObjectsData,leftEdge);
        if(!ObjectsData->isWall){
            check_edge(ObjectsData,rightEdge);
        }
    }
}

static void check_edge(ObjectsData_t *ObjectsData,int *edge){
    int height = 8;
    float ref1 = 0;
    bool potentialWall = check_first_Col(ObjectsData,edge[0],&height,&ref1);
    //DEBUG_PRINT("First Potential Wall: %i;height: %i;ref %f \n",(int)potentialWall,height,(double)ref1);
    if (potentialWall){
        float ref2 = 0;
        potentialWall = check_second_Col(ObjectsData,edge[1],height,&ref2);
        //DEBUG_PRINT("Second Potential Wall: %i;ref %f \n",(int)potentialWall,(double)ref2);
        if (potentialWall){
            float approachingAngle = computeAproachingAngle(ref1,ref2);
            //DEBUG_PRINT("Approaching Angle: %f\n",(double)approachingAngle);
            if(approachingAngle!=-1){
                float ref3 = getNextPixelDistance(approachingAngle);
                //DEBUG_PRINT("ref3: %f\n",(double)ref3);
                check_third_Col(ObjectsData,edge[2],height,ref3);
            }
        }
    }
}

static bool check_first_Col(ObjectsData_t *ObjectsData,int col,int *height,float *ref){
    bool init = false;
    for (int i=COLUMN_LENGTH-1;i>=0;i--){
        float value = ObjectsData->ToF_Sensor_Data[col+i*8];
        if(value==-1){
            if(init){
                if (i == 1){
                    return false;
                }else{
                    *height = i;
                    return true;
                }
            }else{
                return false;
            }
        }else{
            if(!init){
                *ref = value;
                init = true;
            }else if(fabsf(value-*ref)>COLUMN_OFFSET){
                if (i == 1){
                    return false;
                }else{
                    *height = i;
                    return true;
                }
            }
        }
    }
    return true;
}

static bool check_second_Col(ObjectsData_t *ObjectsData,int col,int height,float *ref){
    bool init = false;
    for (int i=COLUMN_LENGTH-1;i>=0;i--){
        float value = ObjectsData->ToF_Sensor_Data[col+i*8];
        if(value==-1){
            if(init){
                if( i>=height-1 && i<=height+1){
                    return true;
                }else{
                    return false;
                }
            }else{
                return false;
            }
        }else{
            if(!init){
                *ref = value;
                init = true;
            }else if(fabsf(value-*ref)>COLUMN_OFFSET){
                if( i>=height-1 && i<=height+1){
                    return true;
                }else{
                    return false;
                }
            }
        }
    }
    return true;
}

static void check_third_Col(ObjectsData_t *ObjectsData,int col,int height,float ref){
    for (int i=COLUMN_LENGTH-1;i>=0;i--){
        float value = ObjectsData->ToF_Sensor_Data[col+i*8];
        if(value==-1){
            if( i>=height-1 && i<=height+1){
                ObjectsData->isWall = true;
            }else{
                ObjectsData->isWall = false;
            }
            break;
        }else{
            if(fabsf(value-ref)>COLUMN_OFFSET){
                if( i>=height-1 && i<=height+1){
                    ObjectsData->isWall = true;
                }else{
                    ObjectsData->isWall = false;
                }
                break;
            }
        }
    }
    ObjectsData->isWall = true;
}