/*
 * Copyright 2016-2018 NXP
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
 * @file    LPC824_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "clock_config.h"
#include "LPC824.h"
#include "fsl_debug_console.h"
#include "fsl_usart.h"
#include "fsl_dma.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include <stdbool.h>

#define USART USART1
//#define EXAMPLE_USART_CLK_SRC kCLOCK_MainClk
//#define EXAMPLE_USART_CLK_FREQ CLOCK_GetFreq(EXAMPLE_USART_CLK_SRC)

#define ECHO_BUFFER_LENGTH 5
#define BUFFER_SIZE 5
#define SPI_MASTER SPI0
#define CLK_SRC kCLOCK_MainClk
#define SPI_MASTER_CLK_FREQ CLOCK_GetFreq(CLK_SRC)
#define SPI_MASTER_BAUDRATE 6000000//500000U
#define SPI_MASTER_SSEL kSPI_Ssel0Assert

#define SPI_MASTER_DMA_BASEADDR DMA0
#define SPI_MASTER_TX_CHANNEL 7
#define SPI_MASTER_RX_CHANNEL 6


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void USARTUserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData);
static void USARTInit(void);
static void USARTPrepareTransfer(void);
static void SPI_DmaTxCallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);
static void SPI_DmaRxCallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);
static void SPIMasterInit(void);
static void MasterDMASetup(void);
static void MasterStartDMATransfer(void);


usart_handle_t g_usartHandle;
uint8_t g_tipString[] = "Usart interrupt transfer example.\r\nBoard receives 8 characters then sends them out.\r\nNow please input:\r\n";

uint16_t Uart_TxBuffer[ECHO_BUFFER_LENGTH] = {0};
uint16_t Uart_RxBuffer[ECHO_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

usart_transfer_t xfer;
usart_transfer_t sendXfer;
usart_transfer_t receiveXfer;
dma_handle_t masterTxHandle;
dma_handle_t masterRxHandle;

static volatile bool masterTxFinished = false;
static volatile bool masterRxFinished = false;

SDK_ALIGN(dma_descriptor_t txDescriptor, 16) = {0};


static uint16_t MotorUpdate[BUFFER_SIZE];
static uint16_t MotorData[BUFFER_SIZE];


uint16_t devID = 0;


static void SPI_DmaTxCallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	masterTxFinished = true;
}

static void SPI_DmaRxCallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	masterRxFinished = true;
}


static void USARTUserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{

	if (kStatus_USART_TxIdle == status)
	{
		GPIO_PinWrite(GPIO, 0, 22, 0); //set dir pin low
		USART_TransferReceiveNonBlocking(USART, &g_usartHandle, &receiveXfer, NULL);



	}

	if (kStatus_USART_RxIdle == status)
	{
		if(Uart_RxBuffer[0] == devID){
			memcpy(MotorUpdate, Uart_RxBuffer, ECHO_BUFFER_LENGTH);//copy
			memcpy(Uart_TxBuffer, MotorData, ECHO_BUFFER_LENGTH);
			MasterStartDMATransfer();
			GPIO_PinWrite(GPIO, 0, 22, 1); //set dir pin high
			USART_TransferSendNonBlocking(USART, &g_usartHandle, &sendXfer);
		}
		else
			return;

	}
}

int main(void) {

	/* Init board hardware. */
	CLOCK_EnableClock(kCLOCK_Spi0);

	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
	USARTInit();
	USARTPrepareTransfer();

	SPIMasterInit();

	/* Set up DMA configuration. */
	MasterDMASetup();
	USART_TransferReceiveNonBlocking(USART, &g_usartHandle, &receiveXfer, NULL);


	while (1)
	{

	}
}


static void USARTInit(void)
{
	usart_config_t config;

	/* Default config by using USART_GetDefaultConfig():
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUSART_ParityDisabled;
	 * config.stopBitCount = kUSART_OneStopBit;
	 * config.bitCountPerChar = kUSART_8BitsPerChar;
	 * config.loopback = false;
	 * config.enableRx = false;
	 * config.enableTx = false;
	 * config.syncMode = kUSART_SyncModeDisabled;
	 */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableRx = true;
	config.enableTx = true;

	/* Initialize the USART with configuration. */
	USART_Init(USART, &config, 0);
}

static void USARTPrepareTransfer(void)
{
	/* Create USART handle, this API will initialize the g_usartHandle and install the callback function. */
	USART_TransferCreateHandle(USART, &g_usartHandle, USARTUserCallback, NULL);

	/* Set the xfer parameter to send tip info to the terminal. */
	xfer.data = g_tipString;
	xfer.dataSize = sizeof(g_tipString) - 1;

	/* Set xfer parameters for sending data. */
	sendXfer.data = Uart_TxBuffer;
	sendXfer.dataSize = sizeof(Uart_TxBuffer);

	/* Set xfers parameters for receiving data. */
	receiveXfer.data = Uart_RxBuffer;
	receiveXfer.dataSize = sizeof(Uart_RxBuffer);
}

static void  SPIMasterInit(void)
{
	uint32_t srcFreq = 0U;
	spi_master_config_t masterConfig;
	/* configuration from using SPI_MasterGetDefaultConfig():
	 * userConfig->enableLoopback = false;
	 * userConfig->enableMaster = true;
	 * userConfig->polarity = kSPI_ClockPolarityActiveHigh;
	 * userConfig->phase = kSPI_ClockPhaseFirstEdge;
	 * userConfig->direction = kSPI_MsbFirst;
	 * userConfig->baudRate_Bps = 500000U;
	 * userConfig->dataWidth = kSPI_Data8Bits;
	 * userConfig->sselNum = kSPI_Ssel0Assert;
	 * userConfig->sselPol = kSPI_SpolActiveAllLow;
	 */
	SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = SPI_MASTER_BAUDRATE;
	masterConfig.sselNumber = SPI_MASTER_SSEL;
	srcFreq = SPI_MASTER_CLK_FREQ;
	SPI_MasterInit(SPI_MASTER, &masterConfig, srcFreq);
}


static void MasterDMASetup(void)
{
	/* DMA init */
	DMA_Init(SPI_MASTER_DMA_BASEADDR);

	/* Enable channel and Create handle for RX channel. */
	DMA_EnableChannel(SPI_MASTER_DMA_BASEADDR, SPI_MASTER_RX_CHANNEL);
	DMA_CreateHandle(&masterRxHandle, SPI_MASTER_DMA_BASEADDR, SPI_MASTER_RX_CHANNEL);
	DMA_SetCallback(&masterRxHandle, SPI_DmaRxCallback, NULL);

	/* Enable channel and Create handle for TX channel. */
	DMA_EnableChannel(SPI_MASTER_DMA_BASEADDR, SPI_MASTER_TX_CHANNEL);
	DMA_CreateHandle(&masterTxHandle, SPI_MASTER_DMA_BASEADDR, SPI_MASTER_TX_CHANNEL);
	DMA_SetCallback(&masterTxHandle, SPI_DmaTxCallback, NULL);

	/* Set the channel priority. */
	DMA_SetChannelPriority(SPI_MASTER_DMA_BASEADDR, SPI_MASTER_TX_CHANNEL, kDMA_ChannelPriority3);
	DMA_SetChannelPriority( SPI_MASTER_DMA_BASEADDR, SPI_MASTER_RX_CHANNEL, kDMA_ChannelPriority2);
}

static void MasterStartDMATransfer(void)
{
	uint32_t i = 0U;
	dma_transfer_config_t masterTxDmaConfig, masterRxDmaConfig;

	/* Prepare buffer to send and receive data. */
	//May remove this
	for (i = 0U; i < BUFFER_SIZE; i++)
	{
		MotorData[i] = 0U;
	}

	/* Prepare and start DMA RX transfer. */
	DMA_PrepareTransfer(&masterRxDmaConfig, (void *)&SPI_MASTER->RXDAT, MotorData, sizeof(uint8_t), BUFFER_SIZE,
			kDMA_PeripheralToMemory, NULL);
	DMA_SubmitTransfer(&masterRxHandle, &masterRxDmaConfig);

	/* Start DMA TX transfer. */
	DMA_StartTransfer(&masterRxHandle);

	/* DMA transfer configuration setting. */
	dma_xfercfg_t tmp_xfercfg = {0};
	tmp_xfercfg.valid = true;
	tmp_xfercfg.swtrig = true;
	tmp_xfercfg.intA = true;
	tmp_xfercfg.byteWidth = sizeof(uint32_t);
	tmp_xfercfg.srcInc = 0;
	tmp_xfercfg.dstInc = 0;
	tmp_xfercfg.transferCount = 1;

	/* Add confifuration parameter to descriptor. */
	DMA_PrepareTransfer(&masterTxDmaConfig, MotorUpdate, (void *)&SPI_MASTER->TXDAT, sizeof(uint8_t),
			BUFFER_SIZE - 1, kDMA_MemoryToPeripheral, &txDescriptor);//TODO: set tx buffer to be the received motor data

	/* Disable interrupts for first descriptor to avoid calling callback twice. */
	masterTxDmaConfig.xfercfg.intA = false;
	masterTxDmaConfig.xfercfg.intB = false;

	DMA_SubmitTransfer(&masterTxHandle, &masterTxDmaConfig);

	/* Start DMA TX transfer. */
	DMA_StartTransfer(&masterTxHandle);
}


