/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "Board.h"

#define TASKSTACKSIZE   1024
#define Board_initI2C() I2C_init();

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;
static PIN_State ledPinState;
static PIN_Handle CinHandle;
static PIN_State CinState;

/*UART DRIVER*/
#include <ti/drivers/UART.h>
#define CMD_CONFIG 0x00
#define CMD_ID     0x39
#define CMD_IDLE   0x58

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */

PIN_Config CinTable[] = {
    Board_BTN1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config buttonPinTable[] = {
    Board_DIO15  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

uint16_t temp = 0;
uint16_t humidity = 0;
int t = 0;
int count=0;
int h = 0;
uint8_t tx_buffer_config[1];
uint8_t rx_buffer_config[1];
uint8_t tx_buffer_data[4];
uint8_t rx_buffer_id[13];
UART_Handle uart;
//float t = 0;
//float h = 0;
/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{
    char input;
            unsigned int i=0;
            UART_Params uartParams;
            const char echoPrompt[] = "0x00";
            /* Create a UART with data processing off. */
            UART_Params_init(&uartParams);
            uartParams.writeDataMode = UART_DATA_BINARY;
            uartParams.readDataMode = UART_DATA_BINARY;
            uartParams.readReturnMode = UART_RETURN_NEWLINE;
            uartParams.readEcho = UART_ECHO_OFF;
            uartParams.baudRate =19200;// baud rate of the interface
            uart = UART_open(Board_UART0, &uartParams);
            if (uart == NULL) {
            System_abort("Error opening the UART");
            }
            tx_buffer_config[0]=CMD_CONFIG;
            UART_write(uart, &tx_buffer_config, 1);
            UART_read(uart, &rx_buffer_config, 1);
            System_printf("sending the request for config mode\n");
            System_printf("received ACK %d\n",rx_buffer_config[0]);
            System_flush();
            tx_buffer_config[0]=CMD_ID;
            UART_write(uart, &tx_buffer_config, 1);
            UART_read(uart, &rx_buffer_id, 12);
            System_printf("sending the request to send ID\n");
            System_flush();
            for(i=0;i<12;i++){
            if (i<4){
             System_printf("ID[%d] is %02x\n",i,rx_buffer_id[i]);
             System_flush();
            }
            else {
             System_printf("PAC[%d] is %02x\n",i,rx_buffer_id[i]);
            }
            }
            System_flush();
            tx_buffer_config[0]=CMD_IDLE;
            UART_write(uart, &tx_buffer_config, 1);
            System_printf("sending the IDLE mode Req\n");
            System_flush();/* after this sequence the system is ready to send data */

     uint8_t txBuffer[1];
     uint8_t rxBuffer[2];
     I2C_Handle i2c;
     I2C_Params i2cParams;
     I2C_Transaction i2cTransaction;
     /* Create I2C for usage */
     I2C_Params_init(&i2cParams);
     i2cParams.bitRate = I2C_400kHz;
     i2c = I2C_open(Board_I2C, &i2cParams);

     if (i2c == NULL) {
     System_abort("Error Initializing I2C\n");
     }
     else {
     System_printf("I2C Initialized!\n");
     }


      i2cTransaction.slaveAddress = 0x40;
      i2cTransaction.writeBuf = txBuffer;
      i2cTransaction.writeCount = 1;
      i2cTransaction.readBuf = rxBuffer;
      i2cTransaction.readCount = 2;
      /* Take 20 samples and print them out onto the console */
      /* Deinitialized I2C */

      while (1) {
        txBuffer[0] = 0xE3;
        if (I2C_transfer(i2c, &i2cTransaction)) {
                //System_printf("Temp : %d %d (C)\n", rxBuffer[0], rxBuffer[1]);
                uint16_t temp1 = rxBuffer[0];
                uint16_t temp2 = rxBuffer[1];
                temp = ((temp1 << 8) | (temp2 & 0xFF));
                t = temp * 175.72;
                t = t / 65536;
                t = t - 46.85;
                t = t*1.8;
                t = t + 32;
                System_printf("Temp : %d (F)\n", t);
                }
                else {
                System_printf("I2C Bus fault\n");
                }
        txBuffer[0] = 0xE5;
        if (I2C_transfer(i2c, &i2cTransaction)) {
                        //System_printf("Humidity: %d %d (C)\n", rxBuffer[0], rxBuffer[1]);
                        uint16_t hum1 = rxBuffer[0];
                        uint16_t hum2 = rxBuffer[1];
                        humidity = ((hum1 << 8 ) | (hum2 & 0xFF));
                        h = humidity * 125;
                        h = h / 65536;
                        h = h - 6;
                        System_printf("Percentage Relative Humidity: %d \n", h);
                        }
                        else {
                        System_printf("I2C Bus fault\n");
                        }
        System_flush();
        Task_sleep((UInt)arg0);
      }

      I2C_close(i2c);
            System_printf("I2C closed!\n");

}

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {

   if(pinId>0)
    {
       PIN_setOutputValue(ledPinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
        count++;
        System_printf("Count : %d \n",count);
    }
           // System_printf("Sound : %d \n",sound);
              //      System_flush();
}

void CallbackFxn(PIN_Handle handle, PIN_Id pinId) {

   if(pinId>0)
    {
               tx_buffer_data[0]=0x02; // Size of data to be sent in bytes
               tx_buffer_data[1]=t; // Data begins........ upto the size of size defined
               tx_buffer_data[2]=h;
               //tx_buffer_data[3]=0x03;
       //      UART_write(uart, &tx_buffer_data, 4);
       //            System_printf("sending Data \n");
       //            System_flush();
       //            /* Send 5 packets to the base station*/
               UART_write(uart, &tx_buffer_data, 2);
               System_printf("sending Data \n");
               System_flush();
               PIN_setOutputValue(ledPinHandle, Board_LED0,
                                              1);
    }
}
/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    // Board_initSPI();
    // Board_initUART();
    // Board_initWatchdog();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
        if(!buttonPinHandle) {
            System_abort("Error initializing button pins\n");
        }

    CinHandle = PIN_open(&CinState, CinTable);
        if(!buttonPinHandle) {
            System_abort("Error initializing button pins\n");
        }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
            System_abort("Error registering button callback function");
        }

    if (PIN_registerIntCb(CinHandle, &CallbackFxn) != 0) {
                System_abort("Error registering button callback function");
            }

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    //System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
