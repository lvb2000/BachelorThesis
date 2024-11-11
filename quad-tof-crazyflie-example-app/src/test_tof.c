#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "log.h"
#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "crtp.h"
#include "vl53l5cx_api.h"
#include "I2C_expander.h"

#include "commander_interface.h"
#include "avoidance.h"
#include "App_defines.h"
#include "App_parameter.h"
#include "objects.h"
#include "mapping.h"
#include "queue_drone.h"
#include "focus.h"

#define DEBUG_MODULE "TOFMATRIX"

static VL53L5CX_Configuration tof_dev[NR_OF_SENSORS];
static VL53L5CX_ResultsData tof_data;

#define TOF_I2C_ADDR 0x56
uint8_t tof_i2c_addresses[NR_OF_SENSORS];

uint8_t config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address);

void set_default_CommanderData(CommanderData_t *CommanderData);

void set_default_AvoidanceData(AvoidanceData_t *AvoidanceData, ToF_Sensors_Data_t *ToF_SensorData,
                               enum Control_States *ControlState, enum Directions *MovingObjectDirection,
                               CommanderData_t *CommanderData);

void set_default_ObjectsData(ObjectsData_t *ObjectData, ToF_Sensors_Data_t *ToF_SensorData,
                             enum Control_States *ControlState);

void set_default_MappingData(MappingData_t *MappingData,ToF_Sensors_Data_t *ToF_SensorData,enum Directions *MovingObjectDirection,CommanderData_t *CommanderData,enum Control_States *ControlState);
void set_default_FocusData(FocusData_t *FocusData,ToF_Sensors_Data_t *ToF_SensorData,enum Directions *MovingObjectDirection,CommanderData_t *CommanderData,enum Control_States *ControlState);

void appMain() {

    logIdStateEstimateYaw = logGetVarId("stateEstimate", "yaw");

    DEBUG_PRINT("Size of configuration %d \n", sizeof(VL53L5CX_Configuration));

    DEBUG_PRINT("Configured for %d ToF sensor(s) \n", NR_OF_SENSORS);
    // Configure GPIO expander pins modes
    I2C_expander_initialize();

    // Define the address of each ToF matrix sensor
    for (uint8_t i = 0; i < NR_OF_SENSORS; i++)
        tof_i2c_addresses[i] = TOF_I2C_ADDR + 2 + 2 * i;

    // Configure the ToF sensor(s)
    for (uint8_t i = 0; i < NR_OF_SENSORS; i++) {
        I2C_expander_set_pin(i, 1);

        // Configure the current sensor
        uint8_t status = config_sensors(&tof_dev[i], tof_i2c_addresses[i]);
        DEBUG_PRINT("Sensor %d conf. status: %d  (0 means ok) \n", i, status);

        // Start ranging
        status = vl53l5cx_start_ranging(&tof_dev[i]);
        DEBUG_PRINT("Sensor %d ranging status: %d  (0 means ok) \n", i, status);

        if (status == 0)
            I2C_expander_set_pin(LED0, 1);
    }

    uint8_t reg_value;
    i2cdevReadByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS, OUTPUT_REG_ADDRESS, &reg_value);
    DEBUG_PRINT("Sensor %d \n", reg_value);

    unsigned int lastUpdate = 0;
    uint8_t ranging_ready = 255;
    uint8_t get_data_success = 255;
    //uint8_t to_send_buffer[4 * NR_OF_PIXELS];

    uint8_t ToF_Sensors_Update = 0;
    ToF_Sensors_Data_t ToF_Sensors_Data;

    enum Control_States ControlState;

    if(MODE==1) ControlState = focus;
    else ControlState = search;

    enum Directions MovingObjectDirection = None;

    CommanderData_t CommanderData;
    set_default_CommanderData(&CommanderData);

    AvoidanceData_t AvoidanceData;
    set_default_AvoidanceData(&AvoidanceData, &ToF_Sensors_Data, &ControlState, &MovingObjectDirection, &CommanderData);

    ObjectsData_t ObjectsData;
    set_default_ObjectsData(&ObjectsData, &ToF_Sensors_Data, &ControlState);

    MappingData_t MappingData;
    set_default_MappingData(&MappingData,&ToF_Sensors_Data,&MovingObjectDirection,&CommanderData,&ControlState);

    FocusData_t FocusData;
    set_default_FocusData(&FocusData,&ToF_Sensors_Data,&MovingObjectDirection,&CommanderData,&ControlState);

    DEBUG_PRINT("Initialization finished.\n");
    DEBUG_PRINT("Wait 5 sec for takeoff.\n");
    vTaskDelay(M2T(15000));
    CommanderData.startTick = xTaskGetTickCount();

    bool hovering = false;

    while (1) {
        //TODO put this back to 20!
        vTaskDelay(M2T(5));

        // loop over all 4 sensors
        for (uint8_t s = 0; s < 4; s++) {
            vl53l5cx_check_data_ready(&tof_dev[s], &ranging_ready);  // poll for data-ready
            if (ranging_ready == 1) {
                get_data_success = vl53l5cx_get_ranging_data(&tof_dev[s], &tof_data);
                if (get_data_success == VL53L5CX_STATUS_OK) {
                    float data[NR_OF_PIXELS];
                    for (int i = 0; i < NR_OF_PIXELS; i++) {
                        if (tof_data.nb_target_detected[i] < 1 ||
                            (tof_data.target_status[i] != 5 && tof_data.target_status[i] != 9))
                            data[i] = -1;
                        else {
                            data[i] = (float) tof_data.distance_mm[i] / 1000.0f;
                        }
                    }
                    switch (s) {
                        case ToF_RIGHT:
                            ToF_Sensors_Update |= ToF_RIGHT_MASK;
                            memcpy(ToF_Sensors_Data.FrontDistance, data, sizeof(data));
                            break;
                        case ToF_BACK:
                            ToF_Sensors_Update |= ToF_BACK_MASK;
                            memcpy(ToF_Sensors_Data.BackDistance, data, sizeof(data));
                            break;
                        case ToF_LEFT:
                            ToF_Sensors_Update |= ToF_LEFT_MASK;
                            memcpy(ToF_Sensors_Data.LeftDistance, data, sizeof(data));
                            break;
                        case ToF_FRONT:
                            ToF_Sensors_Update |= ToF_FRONT_MASK;
                            memcpy(ToF_Sensors_Data.RightDistance, data, sizeof(data));
                            break;
                        default:
                            break;
                    }
                }
            }
            ranging_ready = 2;
        }
        if (ToF_Sensors_Update == ToF_FULL) {
            ToF_Sensors_Update = ToF_EMPTY;
            unsigned int start = T2M(xTaskGetTickCount());
            if (lastUpdate!=0){
                //DEBUG_PRINT("The Update Rate is: %i [frame/s]\n",(int)(1000/(start-lastUpdate)));
            }
            lastUpdate = start;
            // execute Pipeline
            if(hovering){
                if (MODE == 0){
                    mapping_main(&MappingData,&ObjectsData);
                    focus_main(&FocusData,&ObjectsData);
                }
                avoidance_main(&AvoidanceData);
            }else{
                uint32_t delta = T2M(xTaskGetTickCount()-CommanderData.startTick);
                hovering = (float)delta>CommanderData.startTime;
            }
            commander(&CommanderData);
            //unsigned int end = T2M(xTaskGetTickCount());
            //double duration = (double)(end-start)/1000;
            //DEBUG_PRINT("The Algorithm takes: %f [s]\n",duration);
        }
    }
}

void set_default_CommanderData(CommanderData_t *CommanderData) {
    CommanderData->flying = FLYING;
    CommanderData->defaultHeight = DEFAULT_HEIGHT;
    CommanderData->yawRate = 0;
    CommanderData->forwardVel = 0;
    CommanderData->sidewaysVel = 0;
    CommanderData->height = DEFAULT_HEIGHT;
    CommanderData->flightTime = FLIGHT_TIME * 1000;
    CommanderData->startTime = START_TIME * 1000;
}

void set_default_AvoidanceData(AvoidanceData_t *AvoidanceData, ToF_Sensors_Data_t *ToF_SensorData,
                               enum Control_States *ControlState, enum Directions *MovingObjectDirection,
                               CommanderData_t *CommanderData) {
    AvoidanceData->firstActive = true;
    AvoidanceData->firstInactive = true;
    AvoidanceData->areaInit = false;
    AvoidanceData->lastWidestArea = 0;
    AvoidanceData->AbortCounter = 0;
    AvoidanceData->ToF_Sensor_Data = ToF_SensorData;
    AvoidanceData->Control_State = ControlState;
    AvoidanceData->MovingObjectDirection = MovingObjectDirection;
    AvoidanceData->forwardVel = &CommanderData->forwardVel;
    AvoidanceData->sidewaysVel = &CommanderData->sidewaysVel;
    AvoidanceData->yawRate = &CommanderData->yawRate;
}

void set_default_ObjectsData(ObjectsData_t *ObjectData, ToF_Sensors_Data_t *ToF_SensorData,
                             enum Control_States *ControlState) {
    ObjectData->ToF_Sensor_Data = ToF_SensorData->FrontDistance;
    ObjectData->Control_State = ControlState;
    ObjectData->isObject = false;
    ObjectData->isWall = false;
    ObjectData->isCeilingOrGround = false;
    ObjectData->ObjectSize = 0;
    ObjectData->CenterCol = 3;
    ObjectData->CenterRow = 3;
    ObjectData->closestCol = 3;
}

void set_default_MappingData(MappingData_t *MappingData,ToF_Sensors_Data_t *ToF_SensorData,enum Directions *MovingObjectDirection,CommanderData_t *CommanderData,enum Control_States *ControlState) {
    MappingData->firstActive = true;
    MappingData->firstInactive = true;
    MappingData->ToF_Sensor_Data = ToF_SensorData;
    MappingData->MovingObjectDirection = MovingObjectDirection;
    MappingData->Control_State = ControlState;
    MappingData->yaw_rate = &CommanderData->yawRate;
    // Init queues
    Queue_t distanceQueue;
    distanceQueue.front = 0;
    distanceQueue.rear = -1;
    distanceQueue.itemCount = 0;
    // Link queues
    MappingData->distanceQueueFront = distanceQueue;
    MappingData->distanceQueueLeft =distanceQueue;
    MappingData->distanceQueueRight = distanceQueue;
    MappingData->distanceQueueBack =distanceQueue;
}

void set_default_FocusData(FocusData_t *FocusData,ToF_Sensors_Data_t *ToF_SensorData,enum Directions *MovingObjectDirection,CommanderData_t *CommanderData,enum Control_States *ControlState){
    FocusData->firstActive = true;
    FocusData->firstInactive = true;
    FocusData->ToF_Sensor_Data = ToF_SensorData;
    FocusData->MovingObjectDirection = MovingObjectDirection;
    FocusData->Control_State = ControlState;
    //PID
    FocusData->lastMeasurementTimestamp = 0;
    FocusData->integralErr=0;
    FocusData->lastErr = 0;
    FocusData->k =5;
    FocusData->kd=1;
    FocusData->ki=0.3f;
    FocusData->yawRate = &CommanderData->yawRate;
}

// 
uint8_t config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address) {
    p_dev->platform = VL53L5CX_DEFAULT_I2C_ADDRESS; // use default adress for first use

    uint8_t status = 0;
    // Initialize the sensor
    status += vl53l5cx_init(p_dev);

    // Change I2C address
    status += vl53l5cx_set_i2c_address(p_dev, new_i2c_address);
    status += vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);

    // 15Hz frame rate
    status += vl53l5cx_set_ranging_frequency_hz(p_dev, 15);
    status += vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    status += vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

    return status;
}