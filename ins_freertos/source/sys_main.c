/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "system.h"
#include "het.h"
#include "gio.h"
#include "spi.h"

/*freertos*/
#include "FreeRTOS.h" /* Must come first. */
#include "os_task.h"    /* RTOS task related API prototypes. */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
xTaskHandle xTask1Handle;
xTaskHandle xTask2Handle;

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
#define SENSOR_READ     0x01
#define SENSOR_WRITE    0x00


#define DEVID_AD        0x00
#define DEVID_MST       0x01
#define PART_ID         0x02

#define RANGE_CTL       0x2C
#define POWER_CTL       0x2D

#define RANGE           0x03
#define POWER           0x06

uint16 TX_Data_Master[16];
uint16 RX_Data_Master[16];

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    spiInit();
    gioSetDirection(hetPORT1, 0xFFFFFFFF);


    /* Create Task 1 */
    if (xTaskCreate(vTask1,"Task1", configMINIMAL_STACK_SIZE, NULL, 1, &xTask1Handle) != pdTRUE)
    {
       /* Task could not be created */
       while(1);
    }
    /* Create Task 2 */
    if (xTaskCreate(vTask2,"Task2", configMINIMAL_STACK_SIZE, NULL, 1, &xTask2Handle) != pdTRUE)
    {
       /* Task could not be created */
       while(1);
    }
    /* Start Scheduler */
    vTaskStartScheduler();
    /* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */

void vTask1(void *pvParameters) {

    while(1) {
        gioToggleBit(hetPORT1, 0);
        vTaskDelay(500);

    }
}

void vTask2(void *pvParameters) {
    spiDAT1_t dataconfig1_t;
    dataconfig1_t.CS_HOLD = TRUE;
    dataconfig1_t.WDEL    = TRUE;
    dataconfig1_t.DFSEL   = SPI_FMT_0;
    dataconfig1_t.CSNR    = 0xFE;


    TX_Data_Master[0] = (RANGE_CTL << 1) | SENSOR_WRITE;
    spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);
    TX_Data_Master[0] = RANGE;
    spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);

    TX_Data_Master[0] = (POWER_CTL << 1) | SENSOR_WRITE;
    spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);
    TX_Data_Master[0] = POWER;
    spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);

    vTaskDelay(100);




    while(1) {

        TX_Data_Master[0] = (DEVID_AD << 1) | SENSOR_READ;

        spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);
        spiReceiveData(spiREG1, &dataconfig1_t, 1, &RX_Data_Master[3]);
        spiReceiveData(spiREG1, &dataconfig1_t, 1, RX_Data_Master);

        TX_Data_Master[0] = (DEVID_MST << 1) | SENSOR_READ;
        spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);
        spiReceiveData(spiREG1, &dataconfig1_t, 1, RX_Data_Master);
        spiReceiveData(spiREG1, &dataconfig1_t, 1, &RX_Data_Master[1]);

        TX_Data_Master[0] = (PART_ID << 1) | SENSOR_READ;
        spiTransmitData(spiREG1, &dataconfig1_t, 1, TX_Data_Master);
        spiReceiveData(spiREG1, &dataconfig1_t, 1, RX_Data_Master);
        spiReceiveData(spiREG1, &dataconfig1_t, 1, &RX_Data_Master[2]);

        vTaskDelay(1000);
    }
}
/* USER CODE END */
