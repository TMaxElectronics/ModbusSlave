#ifndef ModbusSlave_INC 
#define ModbusSlave_INC 
 
#include <stdint.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"

#include "UART.h"
#include "Timer.h"
#include "DMA.h"

//uncomment this line to generate runtime stats (like received/tranmitted packets etc.)
#define MBS_GENERATE_STATS
#define MODBUS_TX_TIMEOUT 1000

//MBS_EVT_INVALID must be 0 so a check for if(!{ModbusEvent_t}) can work
typedef enum {MBS_EVT_INVALID = 0, MBS_EVT_FRAME_RECEIVED, MBS_EVT_TIMING_ERROR, MBS_EVT_BUFFER_OVERFLOW, MBS_EVT_UART_ERROR, MBS_EVT_RX_OFF, MBS_EVT_DATA_ERROR, MBS_EVT_CRC_ERROR, MBS_EVT_READOUT_BUFFER_TOO_SMALL, MBS_EVT_TX_ERROR, MBS_EVT_TIMEOUT} ModbusEvent_t;

typedef struct{
    uint32_t timestamp;
    uint32_t address;
    uint32_t function;
    uint32_t dataSize;
    uint32_t crc;
} ModbusFrameDescriptor_t;

typedef struct{
    ModbusEvent_t evt;
    uint8_t * data;
    uint32_t dataLength;
} ModbusEventDescriptor_t;

typedef struct{
    uint32_t uartBaudrate;
    uint32_t uartParityMode;
    uint32_t uartStopBits;
    
    uint32_t characterTime_us;
    
    uint32_t use16BitAdress;
} ModbusConfig_t;

typedef struct{
    UartHandle_t * uartHandle;
    TimerHandle_t * timerHandle;
    DmaHandle_t * dataDmaHandle;
    DmaHandle_t * auxDmaHandle;
    
    uint32_t t1_5prValue;
    uint32_t t2prValue;
    uint32_t t3_5prValue;
    
    uint8_t ** buffers;
    uint32_t lastBuffer;
    uint32_t currentBuffer;
    uint32_t nextBuffer;
    
    uint32_t lastBufferSize;
    
    uint32_t use16BitAddress;
    
    QueueHandle_t    eventQueue;
    SemaphoreHandle_t semaphore;
    uint32_t rxEnabled;
} ModbusHandle_t;

//--------------------------- init and layer 1 setup functions ----------------------------------------------------------------------------------------------------- 

ModbusHandle_t * MBS_create(UartHandle_t * uartHandle, TimerHandle_t * timerHandle, ModbusConfig_t * config, uint32_t address);

uint32_t MBS_destroy(ModbusHandle_t * handle);

void MBS_handleConfigUpdate(ModbusHandle_t * handle, ModbusConfig_t * config);


//--------------------------- layer 2 functions ----------------------------------------------------------------------------------------------------- 

void MBS_startRx(ModbusHandle_t * handle);

//stop rx
void MBS_stopRx(ModbusHandle_t * handle);

//--------------------------- layer 3 functions ----------------------------------------------------------------------------------------------------- 

//send a request to a slave and wait for it to respond
ModbusEvent_t MBS_getSlaveResponse(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length, uint32_t timeout_ticks);

//try to send a frame out onto the bus
uint32_t MBS_sendFrame(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length, uint32_t timeout_ticks);

//try to read a frame from the current handle for the duration of {timeout}
ModbusEvent_t MBS_readFrame(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t maxLength, uint32_t timeout_ticks);


#endif 
