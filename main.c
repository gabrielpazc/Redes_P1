/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Practica1_CAN.c
 * @brief   Application entry point.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Freescale includes. */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
/*
 * ******************************************************************************
 * Definitions
 * ******************************************************************************
 */
/* ID's*/
#define rx_IDENTIFIER 0x123
#define tx_IDENTIFIER 0x123
/* Defines from frdm_flexcan_loopback*/
#define EXAMPLE_CAN CAN0
#define EXAMPLE_CAN_CLK_SOURCE (kFLEXCAN_ClkSrc1)
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define EXAMPLE_FLEXCAN_IRQn CAN0_ORed_Message_buffer_IRQn
#define EXAMPLE_FLEXCAN_IRQHandler CAN0_ORed_Message_buffer_IRQHandler
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define DLC (8)
/* */
#define SET_CAN_QUANTUM 0
#define PSEG1 3
#define PSEG2 2
#define PROPSEG 1
/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF
/* RTOS priority*/
#define RTOS_TASK_PRIORITY 5
/* Period */
#define OS_TICK_PERIOD_100MS 100
#define OS_TICK_PERIOD_50MS 50

/*
 * ******************************************************************************
 * Prototypes
 * ******************************************************************************
*/

/*
 * ******************************************************************************
 * Variables
 * ******************************************************************************
*/

volatile bool rxComplete = false;

/* flexcan FRAME*/
flexcan_frame_t txFrame, rxFrame;
void CAN_init(void);
/*
 * @brief   Application entry point.
 */

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

	/*--------------------------------------------------*/
	/*---------------------- Tasks ---------------------*/
	/*--------------------------------------------------*/
	/* Create task*/
	xTaskCreate(CAN_init, "CAN_init", 128, NULL,
			RTOS_TASK_PRIORITY - 1, NULL);

	/* Start scheduler*/
	vTaskStartScheduler();
    for(;;)
    {

    }


}

/* */
void CAN_init(void)
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	uint32_t flag = 1U;

	LOG_INFO("\r\n==FlexCAN loopback functional example -- Start.==\r\n\r\n");


    /* Init FlexCAN module. */
    /*
     * flexcanConfig.clkSrc                 = kFLEXCAN_ClkSrc0;
     * flexcanConfig.baudRate               = 1000000U;
     * flexcanConfig.baudRateFD             = 2000000U;
     * flexcanConfig.maxMbNum               = 16;
     * flexcanConfig.enableLoopBack         = false;
     * flexcanConfig.enableSelfWakeup       = false;
     * flexcanConfig.enableIndividMask      = false;
     * flexcanConfig.disableSelfReception   = false;
     * flexcanConfig.enableListenOnlyMode   = false;
     * flexcanConfig.enableDoze             = false;
     */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);

    /*
     *
     */
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
    flexcanConfig.enableLoopBack = true;

    /*
     *
     */
    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;

    mbConfig.id     = FLEXCAN_ID_STD(rx_IDENTIFIER);
    /* Setup Rx Message Buffer. */
    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

    mbConfig.id     = FLEXCAN_ID_STD(rx_IDENTIFIER);
    /* Setup Rx Message Buffer. */
    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);


    /* Setup Tx Message Buffer. */
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
}

void task_tx_100ms()
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = OS_TICK_PERIOD_100MS;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	/*
	 * Probablemente funcione mejor con semaforos
	 */

	while(1)
	{
	    /* Prepare Tx Frame for sending. */
		txFrame.id     = FLEXCAN_ID_STD(tx_IDENTIFIER);
	    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
	    txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
	    txFrame.length = (uint8_t)DLC;

	    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
	                        CAN_WORD0_DATA_BYTE_3(0x44);
	    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
	                        CAN_WORD1_DATA_BYTE_7(0x88);

	    LOG_INFO("Send message from MB%d to MB%d\r\n", TX_MESSAGE_BUFFER_NUM, RX_MESSAGE_BUFFER_NUM);
	    LOG_INFO("tx word0 = 0x%x\r\n", txFrame.dataWord0);
	    LOG_INFO("tx word1 = 0x%x\r\n", txFrame.dataWord1);

	    (void)FLEXCAN_TransferSendBlocking(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, &txFrame);

	    /* Waiting for Message receive finish. */
	    while (!rxComplete)
	    {
	    }

	    LOG_INFO("\r\nReceived message from MB%d\r\n", RX_MESSAGE_BUFFER_NUM);
	    LOG_INFO("rx word0 = 0x%x\r\n", rxFrame.dataWord0);
	    LOG_INFO("rx word1 = 0x%x\r\n", rxFrame.dataWord1);
	    /* Stop FlexCAN Send & Receive. */
	    FLEXCAN_DisableMbInterrupts(EXAMPLE_CAN, flag << RX_MESSAGE_BUFFER_NUM);

	    LOG_INFO("\r\n==FlexCAN loopback functional example -- Finish.==\r\n");

        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}
