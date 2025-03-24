#ifndef ModbusSlave_INC 
#define ModbusSlave_INC 
 
#include <stdint.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"

#include "UART.h"
#include "Timer.h"
#include "DMA.h"

//TODO move these to the modbusconfig.h file -------------------------------------------------

#define MODBUS_EVENTBUFFER_SIZE 8
#define MODBUS_BUFFER_SIZE 256
#define MODBUS_MAX_DATASIZE 252
#define MODBUS_BUFFERCOUNT 4

#define MODBUS_CRC_POLYNOMIAL 0xA001
#define MBS_MAX_PACKET_SIZE 256
//-------------------------------------------------

//uncomment this line to generate runtime stats (like received/tranmitted packets etc.)
#define MBS_GENERATE_STATS
#define MODBUS_TX_TIMEOUT 1000



// Modbus Function Codes
#define MODBUS_FUNCTIONCODE_READ_COILS                 0x01
#define MODBUS_FUNCTIONCODE_READ_DISCRETE_INPUTS       0x02
#define MODBUS_FUNCTIONCODE_READ_HOLDING_REGISTERS     0x03
#define MODBUS_FUNCTIONCODE_READ_INPUT_REGISTERS       0x04
#define MODBUS_FUNCTIONCODE_WRITE_SINGLE_COIL          0x05
#define MODBUS_FUNCTIONCODE_WRITE_SINGLE_REGISTER      0x06
#define MODBUS_FUNCTIONCODE_READ_EXCEPTION_STATUS      0x07
#define MODBUS_FUNCTIONCODE_DIAGNOSTICS                0x08
#define MODBUS_FUNCTIONCODE_GET_COM_EVENT_COUNTER      0x0B
#define MODBUS_FUNCTIONCODE_GET_COM_EVENT_LOG          0x0C
#define MODBUS_FUNCTIONCODE_WRITE_MULTIPLE_COILS       0x0F
#define MODBUS_FUNCTIONCODE_WRITE_MULTIPLE_REGISTERS   0x10
#define MODBUS_FUNCTIONCODE_REPORT_SERVER_ID           0x11
#define MODBUS_FUNCTIONCODE_READ_FILE_RECORD           0x14
#define MODBUS_FUNCTIONCODE_WRITE_FILE_RECORD          0x15
#define MODBUS_FUNCTIONCODE_MASK_WRITE_REGISTER        0x16
#define MODBUS_FUNCTIONCODE_READ_WRITE_MULTIPLE_REGS   0x17
#define MODBUS_FUNCTIONCODE_READ_FIFO_QUEUE            0x18
#define MODBUS_FUNCTIONCODE_ENCAPSULATED_INTERFACE     0x2B

// Exception Codes (Function Code + 0x80)
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION           0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS       0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE         0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE       0x04
#define MODBUS_EXCEPTION_ACKNOWLEDGE                0x05
#define MODBUS_EXCEPTION_SLAVE_DEVICE_BUSY          0x06
#define MODBUS_EXCEPTION_MEMORY_PARITY_ERROR        0x07
#define MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAILABLE   0x08
#define MODBUS_EXCEPTION_GATEWAY_TARGET_FAILED      0x09

//MBS_EVT_INVALID must be 0 so a check for if(!{ModbusEvent_t}) can work
typedef enum {MBS_EVT_INVALID = 0, MBS_EVT_FRAME_RECEIVED, MBS_EVT_TIMING_ERROR, MBS_EVT_BUFFER_OVERFLOW, MBS_EVT_UART_ERROR, MBS_EVT_RX_OFF, MBS_EVT_DATA_ERROR, MBS_EVT_CRC_ERROR, MBS_EVT_READOUT_BUFFER_TOO_SMALL, MBS_EVT_TX_ERROR, MBS_EVT_TIMEOUT, MBS_EVT_COUNT} ModbusEvent_t;

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
    uint32_t address;
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

ModbusHandle_t * MBS_create(UartHandle_t * uartHandle, TimerHandle_t * timerHandle, ModbusConfig_t * config);

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
