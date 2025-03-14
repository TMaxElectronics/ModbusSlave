 
#include <stdint.h>

#include "ModbusSlave.h"

//TODO move these to the modbusconfig.h file -------------------------------------------------

#define MODBUS_EVENTBUFFER_SIZE 8
#define MODBUS_BUFFER_SIZE 256
#define MODBUS_BUFFERCOUNT 4

#define MODBUS_CRC_POLYNOMIAL 0xA001
#define MBS_MAX_PACKET_SIZE 256
//-------------------------------------------------

#define MBS_GET_DATA_PTR(handle, rawDataPtr) (uint8_t *) (&rawDataPtr[handle->use16BitAddress ? 2 : 1])
#define MBS_GET_PACKET_SIZE(handle, dataSize) (dataSize + (handle->use16BitAddress ? 5 : 4))

//zero variable to be copied into the timer counters
static uint32_t ZERO = 0;

//static eventbuffer. This is used across all libraries. Safe as it will only be written to and read from inside a single isr at a time.
static ModbusEventDescriptor_t eventBuffer;

#define MBS_sendEventFromISR(handle, evtId, dataPtr, length) {eventBuffer.evt = evtId; eventBuffer.data = dataPtr; eventBuffer.dataLength = length; xQueueSendFromISR(handle->eventQueue, &eventBuffer, NULL); }
static void MBS_updateCharacterTimes(ModbusHandle_t * handle, ModbusConfig_t * config);

//ISR prototypes
static uint32_t MBS_dmaISR(uint32_t evt, void * data);
static uint32_t MBS_timerISR(TimerHandle_t * timer, uint32_t flags, void* data);
static uint32_t MBS_uartISR(UartHandle_t * uart, uint32_t flags, void* data);


ModbusHandle_t * MBS_create(UartHandle_t * uartHandle, TimerHandle_t * timerHandle, ModbusConfig_t * config, uint32_t address){
    //is the config valid?
    if(config->uartBaudrate < 4800) return NULL;
    
    //get memory for the pointer
    ModbusHandle_t * ret = pvPortMalloc(sizeof(ModbusHandle_t));
    
    //did we get any?
    if(ret == NULL){
        return NULL;
    }
    
    ret->timerHandle = timerHandle;
    ret->uartHandle = uartHandle;
    
    ret->use16BitAddress = config->use16BitAdress;
    
    //get the dma channels
    ret->dataDmaHandle = DMA_allocateChannel();
    ret->auxDmaHandle = DMA_allocateChannel();
    
    //did we get the two dma channels we need?
    if(ret->dataDmaHandle == NULL || ret->auxDmaHandle == NULL){
        //no... can't do anything but kill ourselves :´(
        
        //free allocated dma channels (this really can only be the data dma as its gets allocated first)
        if(ret->dataDmaHandle != NULL) DMA_freeChannel(ret->dataDmaHandle);
        if(ret->auxDmaHandle != NULL) DMA_freeChannel(ret->auxDmaHandle);
        
        //free memory and return
        vPortFree(ret);
        return NULL;
    }
    
    //setup aux dma interrupts
    DMA_setInterruptConfig(ret->auxDmaHandle, 0, 0, 0, 0, 1, 0, 0, 0); //interrupt when block is done
    DMA_setIRQEnabled(ret->auxDmaHandle, 0);    //but make sure to disable the interrupt first
    
    //allocate buffer pointer TODO perhaps move this from the heap to the heap (but inside the descriptor :P)
    ret->buffers = pvPortMalloc(sizeof(uint8_t *) * MODBUS_BUFFERCOUNT);
    
    //did that work?
    if(ret->buffers == NULL){
        //no... :/
        
        //free allocated dma channels (this really can only be the data dma as its gets allocated first)
        DMA_freeChannel(ret->dataDmaHandle);
        DMA_freeChannel(ret->auxDmaHandle);
        
        //free memory and return
        vPortFree(ret);
        return NULL;
    }
    
    //allocate the buffers themselves
    for(int32_t i = 0; i < MODBUS_BUFFERCOUNT; i++){ 
        if((ret->buffers[i] = pvPortMalloc(sizeof(uint8_t) * MODBUS_BUFFER_SIZE)) == NULL){
            //failed, free all buffers before this one
            for(i--; i >= 0; i--){
                vPortFree(ret->buffers[i]);
            }
            vPortFree(ret->buffers);
            
            //free everything else
            DMA_freeChannel(ret->dataDmaHandle);
            DMA_freeChannel(ret->auxDmaHandle);
            vPortFree(ret);
            
            return NULL;
        }
    }
    
    //initialize buffer pointers. lastBuffer is init'ed to INT32_MAX to signal that its invalid right now as no last buffer is selected yet
    ret->lastBuffer = INT32_MAX;
    ret->currentBuffer = 0;
    ret->nextBuffer = 0;
    
    //create a semaphore
    ret->semaphore = xSemaphoreCreateBinary();
    
    //generate the event stream
    ret->eventQueue = xQueueCreate(MODBUS_EVENTBUFFER_SIZE, sizeof(ModbusEventDescriptor_t));
    
    //did the two work?
    if(ret->semaphore == NULL || ret->eventQueue == NULL){
        if(ret->semaphore != NULL) vSemaphoreDelete(ret->semaphore);
        if(ret->eventQueue != NULL) vQueueDelete(ret->eventQueue);
        
        //failed, free all buffers before this one
        for(int32_t i = MODBUS_BUFFERCOUNT - 1; i >= 0; i--){
            vPortFree(ret->buffers[i]);
        }
        vPortFree(ret->buffers);

        //free everything else
        DMA_freeChannel(ret->dataDmaHandle);
        DMA_freeChannel(ret->auxDmaHandle);
        vPortFree(ret);
        return NULL;
    }
    
    
    //assign isrs
    UART_setISR(ret->uartHandle, MBS_uartISR, ret);
    TMR_setISR(ret->timerHandle, MBS_timerISR, ret);
    DMA_setIRQHandler(ret->auxDmaHandle, MBS_dmaISR, ret);
    
    //setup the uart module
    
    //setup rts pin for simplex flow control mode
    UART_setFlowControl(ret->uartHandle, UART_RTSMODE_SIMPLEX, UART_FC_RTS);
    
    //no signal inversion
    UART_setOutputInvert(ret->uartHandle, 0, 0);
    
    //set configurable parameters
    UART_setParity(ret->uartHandle, config->uartParityMode);
    UART_setStopBits(ret->uartHandle, config->uartStopBits);
    UART_setBaud(ret->uartHandle, config->uartBaudrate);
    
    //update character times
    MBS_updateCharacterTimes(ret, config);
    
    //set irq modes
    UART_setTxIrqMode(ret->uartHandle, UART_TX_IRQ_WHEN_BUFFER_EMPTY);  //this is set to when_empty so the delay after dma block complete will start only once the second to last byte was sent
    UART_setRxIrqMode(ret->uartHandle, UART_RX_IRQ_WHEN_DATA_AVAILABLE);        
    
    //disable both rx and tx until we need it
    UART_setRxEnabled(ret->uartHandle, 0);
    UART_setTxEnabled(ret->uartHandle, 0);
    
    UART_setEnabled(ret->uartHandle, 1);
    
    //setup interrupt priorities
    UART_setInterruptPriority(ret->uartHandle, configMAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    TMR_setInterruptPriority(ret->timerHandle, configMAX_SYSCALL_INTERRUPT_PRIORITY, 1);
    //DMA_setInterruptPriority(ret->timerHandle, configMAX_SYSCALL_INTERRUPT_PRIORITY, 1);  //TODO: dma lib doesn't yet support interrupt priorities :(
    
    return ret;
}

uint32_t MBS_destroy(ModbusHandle_t * handle){
    //first make sure the handle isn't in use right now
    if(!xSemaphoreTake(handle->semaphore, portMAX_DELAY)){
        return pdFAIL;
    }
    
    //free the semaphore
    vSemaphoreDelete(handle->semaphore);
    
    //free the eventBuffer
    vQueueDelete(handle->eventQueue);
    
    //free buffers
    for(uint32_t i = 0; i < MODBUS_BUFFERCOUNT; i++) vPortFree(handle->buffers[i]);
    vPortFree(handle->buffers);
    
    //unset isrs
    UART_setISR(handle->uartHandle, NULL, NULL);
    DMA_setIRQHandler(handle->dataDmaHandle, NULL, NULL);
    DMA_setIRQHandler(handle->auxDmaHandle, NULL, NULL);
    TMR_setISR(handle->timerHandle, NULL, NULL);
    

    //free everything else
    DMA_freeChannel(handle->dataDmaHandle);
    DMA_freeChannel(handle->auxDmaHandle);
    
    vPortFree(handle);
    //return
    return pdPASS;
}

//--------------------------- config functions ----------------------------------------------------------------------------------------------------- 

static void MBS_updateCharacterTimes(ModbusHandle_t * handle, ModbusConfig_t * config){
    if(config->uartBaudrate < 4800) return;
    
    //get t1
    uint32_t charTime_ns = 11 * (1000000000 / config->uartBaudrate);
    
    //derive t1.5,t2 and t3.5
    uint32_t t1_5_us    = (charTime_ns * 15)    / 10000;
    uint32_t t2_us      = (charTime_ns * 2)     / 1000;
    uint32_t t3_5_us    = (charTime_ns * 35)    / 10000;
    
    //get timer pr values TODO: fix setPeriod function in the timer lib to be fast enough not to need this...
    handle->t1_5prValue = TMR_calculatePR(handle->timerHandle, t1_5_us, 1);
    handle->t2prValue = TMR_calculatePR(handle->timerHandle, t2_us, 1);
    handle->t3_5prValue = TMR_calculatePR(handle->timerHandle, t3_5_us, 1);
}

void MBS_handleConfigUpdate(ModbusHandle_t * handle, ModbusConfig_t * config){
    if(config->uartBaudrate < 4800) return;
    
    //wait until the module is free
    if(!xSemaphoreTake(handle->semaphore, MODBUS_TX_TIMEOUT)){
        //didn't get it... wtf xD This probably means the timeout was too short or the receive routine is stuck somehow
        return pdFAIL;
    }
    
    //disable rx
    UART_setIRQsEnabled(handle->uartHandle, 0);
    UART_setRxEnabled(handle->uartHandle, 0);
    
    //update the uart parameters
    UART_setParity(handle->uartHandle, config->uartParityMode);
    UART_setStopBits(handle->uartHandle, config->uartStopBits);
    UART_setBaud(handle->uartHandle, config->uartBaudrate);
    
    //update character times
    MBS_updateCharacterTimes(handle, config);
    
    //and finally return the semaphore
    xSemaphoreGive(handle->semaphore);
}

//--------------------------- layer 2 functions ----------------------------------------------------------------------------------------------------- 
static uint8_t * MBS_getNextBuffer(ModbusHandle_t * handle){
    handle->lastBuffer = handle->currentBuffer;
    handle->currentBuffer = handle->nextBuffer;
    if(++handle->nextBuffer >= MODBUS_BUFFERCOUNT) handle->nextBuffer = 0;
    return handle->buffers[handle->currentBuffer];
}

void MBS_startRx(ModbusHandle_t * handle){
    //enable rx

    /*
     * How the reception works:
     * 
     * State 0 (waiting for the first byte):
     *      -   all dma channels enabled
     *      -   timer not yet running
     *      -   only uart rx interrupt enabled
     *  In this state we are idle and waiting for reception of the first byte of a packet. Once this occurs the uart rx interrupt will trigger calling our isr.
     *  This state can go on indefinitely as time between packets can be very long.
     * 
     *  Once a byte is received we start the timer with the period of t1.5 and disable the uart rx interrupt as the dma deals with the reading of bytes from the buffer.
     * 
     * State 1 (receive data):
     *      -   all dma channels enabled
     *      -   timer running
     *      -   timer interrupt enabled
     *  In this state we are actively receiving data from the transmitting device and writing it into our internal buffer. Everytime a byte is received the auxDma (timer DMA) writes
     *  Zero (0) into the timers count register resetting it. 
     * 
     *  If the frame is over and no more data gets received then the timer won't be reset by the dma anymore and reach its set threshold triggering the timer isr. This
     *  then sets up the timer to wait the remainder of t3.5, reenables the uart rx interrupt (so we notice if a byte comes in between t1.5 and t3.5) and stops the data dma. 
     *  If the buffer fills up before the frame is over the dma will disable itself which can then be seen in the timer isr.
     * 
     *  When this interrupt occurs we also set the data dma to write to the next buffer on reception of another byte.
     * 
     * State 2 (wait for real packet end):
     *      -   all dma channels enabled
     *      -   timer running
     *      -   timer and uart interrupt enabled
     *  Here we are waiting for the remainder of t3.5 to elapse.
     * 
     *  If a byte is received (uart isr triggered) before the timer has switched off (t3.5 elapsed) the last frame was not ended properly and is invalid. 
     *  In this case we reset the dma to start over with the same buffer we had up until now. We want to notify the user of this by sending an event to the queue
     * 
     *  But if no byte is received until the timer elapses (timer isr triggers first) then the frame was valid and we can notify the user of the new data.
     *  The next buffer in this case will be the next valid one
     * 
     *  After this we need to reset everything to state 
     * 
     * State 3 (exeption):
     *      -   dma enabled
     *      -   timer running
     *      -   timer interrupt on
     *  Wait for the t3.5 to elapse after an error or after we just enabled rx
     */

//setup uart module
    UART_setIRQsEnabled(handle->uartHandle, UART_EVENTFLAG_ERROR_PARITY | UART_EVENTFLAG_ERROR_FRAMING);
    UART_setRxEnabled(handle->uartHandle, 1);

//enable timer interrupt and prepare it for state 0
    TMR_setIRQEnabled(handle->timerHandle, 1);
    TMR_setPR(handle->timerHandle, handle->t3_5prValue);

//prepare the dmas to read the bytes from the uart into the next buffer and reset the timer

    //disable dma irqs
    DMA_setIRQEnabled(handle->dataDmaHandle, 0);
    DMA_setIRQEnabled(handle->auxDmaHandle, 0);

    DMA_setChannelAttributes(handle->auxDmaHandle, 0, 0, 0, 1, 2);    //timer dma will transfer '0' into the timers counter every time a byte is received (this must loop)
    DMA_setChannelAttributes(handle->dataDmaHandle, 0, 0, 0, 0, 1);    //data dma will read out a byte if available (non-looping)

    DMA_setDestConfig(handle->auxDmaHandle, TMR_getTMRPointer(handle->timerHandle), sizeof(uint32_t));
    DMA_setDestConfig(handle->dataDmaHandle, MBS_getNextBuffer(handle), sizeof(uint8_t) * MODBUS_BUFFER_SIZE);

    DMA_setSrcConfig(handle->auxDmaHandle, &ZERO, sizeof(uint32_t));
    DMA_setSrcConfig(handle->dataDmaHandle, UART_getRXRegPtr(handle->uartHandle), sizeof(uint8_t));

    DMA_setTransferAttributes(handle->auxDmaHandle, sizeof(uint32_t), Uart_getRxIRQNum(handle->uartHandle), DMA_IRQ_DISABLED);
    DMA_setTransferAttributes(handle->dataDmaHandle, sizeof(uint8_t), Uart_getRxIRQNum(handle->uartHandle), DMA_IRQ_DISABLED);

    //then enable both channels
    DMA_setEnabled(handle->auxDmaHandle, 1);
    DMA_setEnabled(handle->dataDmaHandle, 1);
    
    //set the flag
    handle->rxEnabled = 1;

    //finally enable the timer to start state 3
    TMR_setEnabled(handle->timerHandle, 1);
}

//stop rx
void MBS_stopRx(ModbusHandle_t * handle){
    //first we try to take the modbus semaphore. This is automatically taken on upon starting of frame reception and returned once its complete TODO add custom timeout
    if(!xSemaphoreTake(handle->semaphore, MODBUS_TX_TIMEOUT)){
        //didn't get it... wtf xD This probably means the timeout was too short or the receive routine is stuck somehow
        return pdFAIL;
    }
    
    //now disable uart rx and its corresponding interrupts. This prevents the rx state machine from starting up again
    UART_setIRQsEnabled(handle->uartHandle, 0);
    UART_setRxEnabled(handle->uartHandle, 0);
    
    //set the flag
    handle->rxEnabled = 0;
    
    //and finally return the semaphore
    xSemaphoreGive(handle->semaphore);
}

//internal function to send raw data onto the bus
static uint32_t MBS_transmitFrame(ModbusHandle_t * handle, uint8_t * buffer, uint32_t size, uint32_t timeout_ticks, uint32_t monitorTransmission){
    //we can only monitor our transmission if the entirety of it fits into our buffers
    if(size > MODBUS_BUFFER_SIZE) monitorTransmission = 0;
    
    /*
     * How transmission works:
     * 
     * as a starting point we assume that rx mode is enabled (which should be the case pretty much all the time). 
     * 
     * In the first step we make sure that the receiver is idle by trying to take the semaphore. If it isn't idle this will block until the bus is clear again.
     * 
     * Once we got the semaphore we set up for transmission. This is done using the auxDma as our data out channel, transferring a byte of data out of the buffer into the uart module on every uart_tx interrupt.
     * If monitorTransmission is set we also enable the receiver and corresponding data dma to read back what we sent out onto the bus. NOTE: this is only sensible if the receiver decodes the data on the line even during a transmission
     * 
     * After all byts are transmitted the auxDma's block done interrupt fires. In the corresponding isr we return the semaphore to signal a complete transmission. After returning from that context we set up for reception again.
     * The startRx function will return all configs to the default for the reception and also cause a t3.5 delay until both reception starts and the semaphore is returned. 
     * This also helps enforce the t3.5 delay for consecutive transmissions as we don't return the semaphore ourselves after a transmission
     * 
     * And if monitorTransmission is set we then check to make sure that the received data is identical to what was transmitted, indicating a valid transfer without a collision
     */
    
    
    //first we try to take the modbus semaphore. This is automatically taken on upon starting of frame reception and returned once its complete
    if(!xSemaphoreTake(handle->semaphore, timeout_ticks)){
        //didn't get it... wtf xD This probably means the timeout was too short or the receive routine is stuck somehow
        return pdFAIL;
    }
    
    //got the semaphore, now set up everything for transmission
    
    //first disable all uart interrupts and rx in general to prevent a new frame reception from being triggered while we prepare for sending one
    //the error interrupts are not needed as an error would only be expected to be detected if monitorTransmission is 1. But if thats the case though we do a memcompare after we're done which would detect an error anyway
    UART_setIRQsEnabled(handle->uartHandle, 0);
    UART_setRxEnabled(handle->uartHandle, 0);
    UART_setTxEnabled(handle->uartHandle, 1);
    
    //disable the timer
    TMR_setEnabled(handle->timerHandle, 0);
    
    //then set up the dma for transmitting data. Data out will be handled by the aux dma
    
    //start off with all channels disabled
    DMA_setEnabled(handle->auxDmaHandle, 0);
    DMA_setEnabled(handle->dataDmaHandle, 0);

    //update channel attributes for the aux channel (disable loop)
    DMA_setChannelAttributes(handle->auxDmaHandle, 0, 0, 0, 0, 2);    //aux dma (data out dma) will transmit one byte every uart tx interrupt and end after one block is complete (non looping)
    DMA_setChannelAttributes(handle->dataDmaHandle, 0, 0, 0, 0, 1);    //data dma will read out a byte if available (non-looping)
    
    //update the cell start interrupt to uart tx irq
    DMA_setTransferAttributes(handle->auxDmaHandle, sizeof(uint8_t), Uart_getTxIRQNum(handle->uartHandle), DMA_IRQ_DISABLED);
    DMA_setTransferAttributes(handle->dataDmaHandle, sizeof(uint8_t), Uart_getRxIRQNum(handle->uartHandle), DMA_IRQ_DISABLED);

    //set the new destination for the aux dma (data dma destination can remain, but we set it anyway incase rx wasn't active before)
    DMA_setDestConfig(handle->auxDmaHandle, UART_getTXRegPtr(handle->uartHandle), sizeof(uint8_t));             //aux dma will write into the txReg of our uart module
    DMA_setDestConfig(handle->dataDmaHandle, MBS_getNextBuffer(handle), sizeof(uint8_t) * MODBUS_BUFFER_SIZE);
    
    //set the source of the aux dma to transfer out of the buffer given to us
    DMA_setSrcConfig(handle->auxDmaHandle, buffer, sizeof(uint8_t) * size);
    DMA_setSrcConfig(handle->dataDmaHandle, UART_getRXRegPtr(handle->uartHandle), sizeof(uint8_t));
    
    //enable block done interrupt for the aux channel (module internal enable bits are already set in our init function)
    DMA_setIRQEnabled(handle->auxDmaHandle, 1);
    
    //and reenable the rx channel and dma if we want to listen back to what we send out
    if(monitorTransmission){ 
        DMA_setEnabled(handle->dataDmaHandle, 1);
        UART_setRxEnabled(handle->uartHandle, 1);
    }
    //NOTE: if some other device on the bus started transmitting a packet while we were preparing to do so it will overwrite the data in the buffer
    //      this is however not a problem, as this has the desired effect of resulting in non-matching buffer content
    
    //at this point the hardware is set up for transmission, start it
    
    DMA_setEnabled(handle->auxDmaHandle, 1);
    DMA_forceTransfer(handle->auxDmaHandle);
    
    //now the transmission is running and we need to wait until it finished and t3.5 elapsed. This is signaled to us by returning the semaphore => try to take it again
    if(!xSemaphoreTake(handle->semaphore, MODBUS_TX_TIMEOUT)){
        //didn't get it, tx failed. We need to clean up and return to rx mode
        
        //abort dma
        DMA_abortTransfer(handle->dataDmaHandle);
        DMA_abortTransfer(handle->auxDmaHandle);
        
        //disable uart tx again
        UART_setTxEnabled(handle->uartHandle, 0);
        
        //setup reception again if needed
        if(handle->rxEnabled) MBS_startRx(handle);
        return pdFAIL;
    }
    
    //yay we got it again. we won't return it though as this is done by the timer isr during transition from state 3 to state 0
    //this will also enforce the t3.5 wait until we can send something again even if we wanted to, as getting the semaphore in the first step of this function would wait until the t3.5 timer has elapsed
    
    //disable uart tx again
    UART_setTxEnabled(handle->uartHandle, 0);

    //the transmission went out successfully, set up for reception again if it was enabled. This also enters state 3 which causes a delay of t3.5 until we get to do anything again
    //if it wasn't enabled we just wait at least the t3.5 with vTaskDelay() TODO!
    if(handle->rxEnabled) MBS_startRx(handle); else;
    
    
    //do we want to check that what was supposed to go out onto the bus actually did so?
    if(monitorTransmission){
        //TODO / NOTE: this does not evaluate presence of the t3.5 delay after the frame was transmitted!
        //yes! do a memcompare
        if(memcmp(buffer, handle->buffers[handle->lastBuffer], size * sizeof(uint8_t)) == 0){
            //contents match => transmission successful
            return pdPASS;
        }else{
            //content is different, something went wrong TODO: perhaps add some different return codes for different errors?
            return pdFAIL;
        }
    }
    
    return pdPASS;
}

static uint32_t MBS_uartISR(UartHandle_t * uart, uint32_t flags, void* data){
    //get our handle
    ModbusHandle_t * handle = (ModbusHandle_t *) data;
    
    //uart interrupt was just triggered, check what happened
    if(flags & (UART_EVENTFLAG_ERROR_FRAMING | UART_EVENTFLAG_ERROR_PARITY)){
        //some receiver error just occured. This can happen at any time during a reception. Check where we are and deal with it accordingly
        
        //no matter where we are we will always have to reset to state 3 and discard the current frame
        
        //disable the timer and set it to t1.5
        TMR_setEnabled(handle->timerHandle, 1);
        TMR_setPR(handle->timerHandle, handle->t3_5prValue);
        
        //disable the uart rx interrupt
        UART_setIRQsEnabled(handle->uartHandle, UART_EVENTFLAG_ERROR_PARITY | UART_EVENTFLAG_ERROR_FRAMING);
        
        //send event
        MBS_sendEventFromISR(handle, MBS_EVT_UART_ERROR, NULL, 0);
    }
    
    if(flags & UART_EVENTFLAG_RX_IRQ){
        //a byte was received! Make sure to disable the interrupt right away so we don't get called again when the next byte is received
        UART_setIRQsEnabled(uart, UART_EVENTFLAG_ERROR_PARITY | UART_EVENTFLAG_ERROR_FRAMING);
        
        //check if we are in state 0 or state 2 (state 2 would have the timer enabled)
        if(TMR_isEnabled(handle->timerHandle)){
            //we are in state 2:
            //  a new frame just started despite not enough time having elapsed since the last one
            
            //also clear the interrupt flag... maybe this might prevent a triggered interrupt from firing after this one completes? TODO evaluate
            TMR_clearIFS(handle->timerHandle);
            
            //set the timer to t1.5 again
            TMR_setPR(handle->timerHandle, handle->t1_5prValue);
            
            
            //at this point everything is setup for state 1 (reception of the data)
            
            //send event to the handler
            MBS_sendEventFromISR(handle, MBS_EVT_TIMING_ERROR, NULL, 0);
        }else{
            //we are in state 0
            //  A byte was received after the last frame had ended properly and was sent to the handler => start of the next frame
            
            //check if a transmission has just started by trying to get the semaphore. If this fails then the user started a transmission right before a new frame started, we'll have to discard this
            if(!xSemaphoreTakeFromISR(handle->semaphore, NULL)) return 0;
            
            //we got the semaphore => ready to receive the frame
            
            //Reenable the timer
            TMR_setEnabled(handle->timerHandle, 1);
            
            //at this point everything is setup for state 1 (reception of the data)
        }
    }
}

static uint32_t MBS_timerISR(TimerHandle_t * timer, uint32_t flags, void* data){
    //get our handle
    ModbusHandle_t * handle = (ModbusHandle_t *) data;
    
    //the timer has just elapsed. Check what state we are in
    if(*TMR_getPRPointer(timer) == handle->t1_5prValue){
        //timer was set to a period of t1.5 => we are in state 1
        
        //did the dma transfer succeed?
        if(DMA_isEnabled(handle->dataDmaHandle)){
            //yes, all good
            
            //set the timer for t2
            TMR_setPR(timer, handle->t2prValue);
        
            //get the size of the last buffer
            handle->lastBufferSize = DMA_getDestinationPointerValue(handle->dataDmaHandle);

            //update the data dma destination
            DMA_setDestConfig(handle->dataDmaHandle, MBS_getNextBuffer(handle), sizeof(uint8_t) * MODBUS_BUFFER_SIZE);
        }else{
            //no it failed, whatever data was received isn't valid anymore anyway and we can go straight to state 0
            
            //disable the timer
            TMR_setEnabled(timer, 0);
            
            //reenable the dma (it was reset once the transfer failed or was completed)
            DMA_setEnabled(handle->dataDmaHandle, 1);
            
            //send an event to the handler
            MBS_sendEventFromISR(handle, MBS_EVT_BUFFER_OVERFLOW, NULL, 0);
            
            //return the semaphore TODO add taskWoken return
            xSemaphoreGiveFromISR(handle->semaphore, NULL);
        }
        
        //enable uart rx irq (required regardless of state 0 or state 2)
        UART_setIRQsEnabled(handle->uartHandle, UART_EVENTFLAG_RX_IRQ | UART_EVENTFLAG_ERROR_PARITY | UART_EVENTFLAG_ERROR_FRAMING);
        
    }else if(*TMR_getPRPointer(timer) == handle->t3_5prValue){
        //we are in state 3 (waiting for final timeout after a frame with an error) and the timeout has occured => reset to state 0
        
        //disable the timer and set it to t1.5
        TMR_setEnabled(handle->timerHandle, 0);
        TMR_setPR(handle->timerHandle, handle->t1_5prValue);
        
        //reset the dma transfer
        DMA_resetTransfer(handle->dataDmaHandle);
        
        //return the semaphore TODO add taskWoken return
        xSemaphoreGiveFromISR(handle->semaphore, NULL);
        
        //disable the uart rx interrupt
        UART_setIRQsEnabled(handle->uartHandle, UART_EVENTFLAG_RX_IRQ | UART_EVENTFLAG_ERROR_PARITY | UART_EVENTFLAG_ERROR_FRAMING);
    }else{
        //we're in state 2 
        //  the t3.5 after the last frame has elapsed, frame was valid
        
        //return the semaphore TODO add taskWoken return
        xSemaphoreGiveFromISR(handle->semaphore, NULL);
        
        //reset the timer for t1.5
        TMR_setPR(timer, handle->t1_5prValue);
        
        //disable the timer
        TMR_setEnabled(timer, 0);
        
        //send the last frame to the handler
        //is the lastBuffer index valid?
        if(handle->lastBuffer == INT32_MAX){
            //lol no? wtf this shouldn't be possible unless this isr in this state gets called as the very first after the lib was initialized... Which should be impossible xD
            //best we can do is notify the user if we're in debug mode
            configASSERT(0);
        }else{
            //lastBuffer is valid => send it to the handler
            MBS_sendEventFromISR(handle, MBS_EVT_FRAME_RECEIVED, handle->buffers[handle->lastBuffer], handle->lastBufferSize);
        }
    }
}

static uint32_t MBS_dmaISR(uint32_t evt, void * data){
    //get our handle
    ModbusHandle_t * handle = (ModbusHandle_t *) data;
    
    //this interrupt is only enabled when we are transmitting, and triggers once the last byte was written into the uart. Just return the semaphore, everything else is handled in the transmit function
    xSemaphoreGiveFromISR(handle->semaphore, NULL);
}


//--------------------------- layer 3 functions ----------------------------------------------------------------------------------------------------- 

static uint32_t MBS_calculateCrc(uint8_t * data, uint32_t length){
    //TODO move this to the dma's crc module?
    
    //honestly crcs are black voodo magic, so i just used the implementation i also use in ConMan :3 Its actually even using the same polynomial
    uint32_t crc = 0xffff;
    
    for(uint32_t cb = 0; cb < length; cb++){
        crc ^= data[cb];
        for (uint32_t i = 0; i < 8; i++) {
            if (crc & 1){
                crc = (crc >> 1) ^ MODBUS_CRC_POLYNOMIAL;
            }else{
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

static uint32_t MBS_checkCrc(ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length){
    //first calculate the crc we'd expect from the data. This is calculated excluding the crc bytes themselves, hence the "-2"
    uint32_t expectedCrc = MBS_calculateCrc(data, length-2);
    
    //compare the two, and return pass if their identical
    if(expectedCrc == frameDescriptor->crc) return pdPASS;
    
    return pdFAIL;
}

static uint32_t MBS_generateFrame(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length, uint8_t * buffer, uint32_t finalFrameSize){
    //generate a frame from descriptor and data
    uint32_t currentByte = 0;
    
    //address, byte pattern depends on the use of 16bit addressing
    if(handle->use16BitAddress){
        buffer[0] = frameDescriptor->address;
        buffer[1] = (frameDescriptor->address >> 8);
        currentByte += 2;
    }else{
        buffer[0] = frameDescriptor->address;
        currentByte++;
    }
    
    //function code
    buffer[currentByte++] = frameDescriptor->function;
    
    //data
    memcpy(&buffer[currentByte++], data, length);
    
    //crc is the last two bytes
    frameDescriptor->crc = MBS_calculateCrc(buffer, finalFrameSize - 2);
    data[finalFrameSize-2] = frameDescriptor->crc;
    data[finalFrameSize-1] = frameDescriptor->crc >> 8;
    
    return pdPASS;
}

static uint32_t MBS_extractDescriptor(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length){
    //check if we even have enough data
    if(length < (handle->use16BitAddress ? 5 : 4)) return pdFAIL;
    
    //pull the fields out of a frame
    uint32_t currentByte = 0;
    
    //address, byte pattern depends on the use of 16bit addressing
    if(handle->use16BitAddress){
        frameDescriptor->address = (data[1] << 8) | data[0];
        currentByte += 2;
    }else{
        frameDescriptor->address = data[0];
        currentByte++;
    }
    
    //function code
    frameDescriptor->function = data[currentByte++];
    
    //crc is the last two bytes
    frameDescriptor->crc = (data[length-1] << 8) | data[length-2];
    
    //data size indicates the number of bytes available in the "data" section, this excludes address, function and the crc
    frameDescriptor->dataSize = length - (handle->use16BitAddress ? 5 : 4);
    
    return pdPASS;
}

//TODO add modbus semaphore to all of these functions:

//try to read a frame from the current handle for the duration of {timeout}
ModbusEvent_t MBS_readFrame(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t maxLength, uint32_t timeout_ticks){
    //check if rx is enabled, if not just return
    if(!handle->rxEnabled) return MBS_EVT_RX_OFF;
    
    //rx is on, try to receive an event from the queue
    ModbusEventDescriptor_t event;
    if(!xQueueReceive(handle->eventQueue, &event, timeout_ticks)){
        //didn't get any event, return
        return MBS_EVT_TIMEOUT;
    }
    
    //got something, is it a valid frame?
    if(event.evt == MBS_EVT_FRAME_RECEIVED){
        //yes! process it
        
        //write data to the header first
        if(!MBS_extractDescriptor(handle, frameDescriptor, event.data, event.dataLength)){
            //failed, not enough data or other issue => cancel processing and reset header
            memset(frameDescriptor, 0, sizeof(frameDescriptor));
            
            return MBS_EVT_DATA_ERROR;
        }
        
        //got the header, now check if the crc is correct
        if(!MBS_checkCrc(frameDescriptor, event.data, event.dataLength)){
            //crc is incorrect, return with an error
            memset(frameDescriptor, 0, sizeof(frameDescriptor));
            
            return MBS_EVT_CRC_ERROR;
        }
        
        //make sure we have a large enough buffer for the data
        if(frameDescriptor->dataSize > maxLength){
            //frame descriptor can remain set, but we return an error anyway
            return MBS_EVT_READOUT_BUFFER_TOO_SMALL;
        }
        
        //packet seems valid and the descriptor is populated, now copy the data, excluding the address, function and crc field
        memcpy(data, MBS_GET_DATA_PTR(handle, event.data), frameDescriptor->dataSize);
        
        //return succ
        return MBS_EVT_FRAME_RECEIVED;
        
    }else{
        //no, just return the code the the caller
        return event.evt;
    }
}

//try to send a frame out onto the bus
uint32_t MBS_sendFrame(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length, uint32_t timeout_ticks){
    uint32_t ret = pdFAIL;
    
    //first calculate the size of the packet that we will send
    uint32_t packetSize = MBS_GET_PACKET_SIZE(handle, length);
    
    //would that work?
    if(packetSize > MBS_MAX_PACKET_SIZE) return pdFAIL;
    
    //get some memory to assemble the frame in
    uint8_t * buffer = pvPortMalloc(sizeof(uint8_t) * packetSize);
    if(buffer == NULL) return pdFAIL;
    
    //assemble the data
    if(MBS_generateFrame(handle, frameDescriptor, data, length, buffer, packetSize)){
        //generation succeeded, send the frame
        if(MBS_transmitFrame(handle, buffer, packetSize, timeout_ticks, 1)){
            //transmission succeeded 
            ret = pdPASS;
        }
    }
    
    //cleanup and return
    vPortFree(buffer);
    return ret;
}

//send a request to a slave and wait for it to respond
ModbusEvent_t MBS_getSlaveResponse(ModbusHandle_t * handle, ModbusFrameDescriptor_t * frameDescriptor, uint8_t * data, uint32_t length, uint32_t timeout_ticks){
    //is rx enabled? If so we need to make sure to reset the eventQueue. Also remember this so we can restore the state after we're done
    uint32_t rxWasOn = handle->rxEnabled;
    if(rxWasOn){
        xQueueReset(handle->eventQueue);
    }
    
    //first we need to send the request to the slave. To make sure we are ready to receive a response asap we enable rx now (which will cause the tx function to reenable it after transmission)
    MBS_startRx(handle);
    if(!MBS_sendFrame(handle, frameDescriptor, data, length, timeout_ticks)){
        //transmission failed somehow :( cleanup and return
        if(!rxWasOn) MBS_stopRx(handle);
        return MBS_EVT_TX_ERROR;
    }
    
    ModbusFrameDescriptor_t responseDescriptor;
    
    //now wait until the slave responds, we get an error or a timeout occurs
    uint32_t timeStarted = xTaskGetTickCount();
    uint32_t timeRemaining = timeout_ticks;
    ModbusEvent_t evt = MBS_EVT_INVALID;
    
    while(1){
        //transmission succeeded, lib is now waiting for the response. Try to get it
        evt = MBS_readFrame(handle, &responseDescriptor, data, length, timeRemaining);
        
        //what happened?
        if(evt == MBS_EVT_FRAME_RECEIVED){
            //we got a frame back :)
            
            //check if its the response we're waiting for. This is indicated by the address field and the lower 7 bits of the function code (bit8 indicates an error)
            if(responseDescriptor.address == frameDescriptor->address && (responseDescriptor.function & 0x7f) == (frameDescriptor->function & 0x7f)){
                //yep that matches, we got our response
                break;
            }
            
            //no its a different response, just ignore it and make sure to change the evt so we remember what happened
            evt = MBS_EVT_INVALID;
            
        //was it a unrecoverable error?
        }else if(evt != MBS_EVT_DATA_ERROR || evt != MBS_EVT_CRC_ERROR || evt != MBS_EVT_READOUT_BUFFER_TOO_SMALL || evt != MBS_EVT_TIMING_ERROR || evt != MBS_EVT_BUFFER_OVERFLOW || evt != MBS_EVT_UART_ERROR){
            //yes, in all other cases something is so wrong that there is no chance we'll receive the frame from the slave anymore :( (such as a timeout or misconfigured rx for example)
            break;
        }
        
        //haven't found what we're looking for yet, check how much (if any) time we have left to wait
        timeRemaining = (timeStarted + timeout_ticks) - xTaskGetTickCount();

        if(timeRemaining > timeout_ticks || timeRemaining == 0){
            //arithmetic overflow occurred on the last calculation or timeRemaining is zero, timeRemaining must be <=0 => timeout has occurred
            evt = MBS_EVT_TIMEOUT;
            break;
        }else{
            //we still have some time left
            continue;
        }
    }
    
    //loop exited. did we find our response?
    if(evt == MBS_EVT_FRAME_RECEIVED){
        //yep we found our frame, nice :) make sure to copy the descriptor from the response so the caller can see it
        memcpy(frameDescriptor, &responseDescriptor, sizeof(ModbusFrameDescriptor_t));
    }
    
    //and finally return what happened
    return evt;
}