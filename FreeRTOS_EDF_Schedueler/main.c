/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used.
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 *
 * Main.c also creates a task called "Check".  This only executes every three
 * seconds but has the highest priority so is guaranteed to get processor time.
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is
 * incremented each time the task successfully completes its function.  Should
 * any error occur within such a task the count is permanently halted.  The
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL ((unsigned char)0x01)

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE ((unsigned long)115200)

#define Button_1_Monitor_Period (portTickType)50
#define Button_2_Monitor_Period (portTickType)50
#define Periodic_Transmitter_Period (portTickType)100
#define Uart_Receiver_Period (portTickType)20
#define Load_1_Simulation_Period (portTickType)10
#define Load_2_Simulation_Period (portTickType)100
#define MESSAGE_SIZE 50
#define Load_1_Calibration 37300
#define Load_2_Calibration 89552

TaskHandle_t Button_1_Monitor_Handle = NULL;
TaskHandle_t Button_2_Monitor_Handle = NULL;
TaskHandle_t Periodic_Transmitter_Handle = NULL;
TaskHandle_t Uart_Receiver_Handle = NULL;
TaskHandle_t Load_1_Simulation_Handle = NULL;
TaskHandle_t Load_2_Simulation_Handle = NULL;

static QueueHandle_t xQueue = NULL;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware(void);
/*-----------------------------------------------------------*/

/* Task to be created. */
void Button_1_Monitor(void *pvParameters)
{

    int8_t xBuffer1[MESSAGE_SIZE] = "Falling Edge detected on Button 1";
    int8_t xBuffer2[MESSAGE_SIZE] = "Rising Edge detected on Button 1";
    pinState_t PinState;
    pinState_t exPinState = PIN_IS_LOW;
    portTickType xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = Button_1_Monitor_Period;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
        GPIO_write(PORT_1, PIN1, PIN_IS_HIGH);
        PinState = GPIO_read(PORT_0, PIN0);
        if (PinState != exPinState)
        {
            if (exPinState == PIN_IS_LOW && PinState == PIN_IS_HIGH)
            {
                xQueueSend(xQueue, xBuffer2, 0);
            }
            else
            {
                xQueueSend(xQueue, xBuffer1, 0);
            }
        }
        exPinState = PinState;
        GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
    }
}

/* Task to be created. */
void Button_2_Monitor(void *pvParameters)
{

    int8_t xBuffer1[MESSAGE_SIZE] = "Falling Edge detected on Button 1";
    int8_t xBuffer2[MESSAGE_SIZE] = "Rising Edge detected on Button 1";
    pinState_t PinState;
    pinState_t exPinState = PIN_IS_LOW;
    portTickType xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = Button_2_Monitor_Period;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
        GPIO_write(PORT_1, PIN2, PIN_IS_HIGH);
        PinState = GPIO_read(PORT_0, PIN1);
        if (PinState != exPinState)
        {
            if (exPinState == PIN_IS_LOW && PinState == PIN_IS_HIGH)
            {
                xQueueSend(xQueue, xBuffer2, 0);
            }
            else
            {
                xQueueSend(xQueue, xBuffer1, 0);
            }
        }
        exPinState = PinState;
        GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
    }
}

/* Task to be created. */
void Periodic_Transmitter(void *pvParameters)
{
    int8_t xBuffer[MESSAGE_SIZE] = "I'm a periodic string. I'm Cool!";
    portTickType xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = Periodic_Transmitter_Period;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
        GPIO_write(PORT_1, PIN3, PIN_IS_HIGH);

        xQueueSend(xQueue, xBuffer, 0);
        GPIO_write(PORT_1, PIN3, PIN_IS_LOW);
    }
}

/* Task to be created. */
void Uart_Receiver(void *pvParameters)
{
    int8_t xRXBuffer[MESSAGE_SIZE];
    int8_t *pxRXBuffer = (int8_t *)&xRXBuffer;
    portTickType xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = Uart_Receiver_Period;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
        GPIO_write(PORT_1, PIN4, PIN_IS_HIGH);
        if (uxQueueMessagesWaiting(xQueue) != 0)
        {
            if (xQueueReceive(xQueue, pxRXBuffer, 0) == pdPASS)
            {
                vSerialPutString(xRXBuffer, MESSAGE_SIZE);
            }
        }
        GPIO_write(PORT_1, PIN4, PIN_IS_LOW);
    }
}

/* Task to be created. */
void Load_1_Simulation(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = Load_1_Simulation_Period;
    uint32_t u32_i = 0;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
        GPIO_write(PORT_1, PIN5, PIN_IS_HIGH);
        for (u32_i = 0; u32_i < Load_1_Calibration; u32_i++)
            ;
        GPIO_write(PORT_1, PIN5, PIN_IS_LOW);
    }
}

/* Task to be created. */
void Load_2_Simulation(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = Load_2_Simulation_Period;
    uint32_t u32_i = 0;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
        GPIO_write(PORT_1, PIN6, PIN_IS_HIGH);
        for (u32_i = 0; u32_i < Load_2_Calibration; u32_i++)
            ;
        GPIO_write(PORT_1, PIN6, PIN_IS_LOW);
    }
}

void vApplicationIdleHook(void)
{
    GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);
}
void vApplicationTickHook(void)
{
    GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);
    GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler.
 */
int main(void)
{
    /* Setup the hardware for use with the Keil demo board. */
    prvSetupHardware();
    xQueue = xQueueCreate(20, (sizeof(char)) * MESSAGE_SIZE);
    /* Create Tasks here */
    /* Create the task, storing the handle. */
    xTaskCreatePeriodic(
        Button_1_Monitor,         /* Function that implements the task. */
        "Button 1 Monitor",       /* Text name for the task. */
        100,                      /* Stack size in words, not bytes. */
        (void *)1,                /* Parameter passed into the task. */
        1,                        /* Priority at which the task is created. */
        &Button_1_Monitor_Handle, /* Used to pass out the created task's handle. */
        Button_1_Monitor_Period);

    xTaskCreatePeriodic(
        Button_2_Monitor,         /* Function that implements the task. */
        "Button 2 Monitor",       /* Text name for the task. */
        100,                      /* Stack size in words, not bytes. */
        (void *)1,                /* Parameter passed into the task. */
        1,                        /* Priority at which the task is created. */
        &Button_2_Monitor_Handle, /* Used to pass out the created task's handle. */
        Button_2_Monitor_Period);

    xTaskCreatePeriodic(
        Periodic_Transmitter,         /* Function that implements the task. */
        "Periodic Transmitter",       /* Text name for the task. */
        100,                          /* Stack size in words, not bytes. */
        (void *)1,                    /* Parameter passed into the task. */
        1,                            /* Priority at which the task is created. */
        &Periodic_Transmitter_Handle, /* Used to pass out the created task's handle. */
        Periodic_Transmitter_Period);

    xTaskCreatePeriodic(
        Uart_Receiver,         /* Function that implements the task. */
        "Uart Receiver",       /* Text name for the task. */
        100,                   /* Stack size in words, not bytes. */
        (void *)1,             /* Parameter passed into the task. */
        1,                     /* Priority at which the task is created. */
        &Uart_Receiver_Handle, /* Used to pass out the created task's handle. */
        Uart_Receiver_Period);

    xTaskCreatePeriodic(
        Load_1_Simulation,         /* Function that implements the task. */
        "Load 1 Simulation",       /* Text name for the task. */
        100,                       /* Stack size in words, not bytes. */
        (void *)1,                 /* Parameter passed into the task. */
        1,                         /* Priority at which the task is created. */
        &Load_1_Simulation_Handle, /* Used to pass out the created task's handle. */
        Load_1_Simulation_Period);

    xTaskCreatePeriodic(
        Load_2_Simulation,         /* Function that implements the task. */
        "Load 2 Simulation",       /* Text name for the task. */
        100,                       /* Stack size in words, not bytes. */
        (void *)1,                 /* Parameter passed into the task. */
        1,                         /* Priority at which the task is created. */
        &Load_2_Simulation_Handle, /* Used to pass out the created task's handle. */
        Load_2_Simulation_Period);

    /* Now all the tasks have been started - start the scheduler.

    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used here. */
    vTaskStartScheduler();

    /* Should never reach here!  If you do then there was not enough heap
    available for the idle task to be created. */
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
    T1TCR |= 0x2;
    T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
    T1PR = 1000;
    T1TCR |= 0x1;
}

static void prvSetupHardware(void)
{
    /* Perform the hardware setup required.  This is minimal as most of the
    setup is managed by the settings in the project file. */

    /* Configure UART */
    xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

    /* Configure GPIO */
    GPIO_init();

    /* Config trace timer 1 and read T1TC to get current tick */
    configTimer1();

    /* Setup the peripheral bus to be the same as the PLL output. */
    VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/
