/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 4, 2019
 *      Author: prudh
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;  //@SPI_device mode
	uint8_t SPI_BusConfig;	//@bus_config
	uint8_t SPI_SclkSpeed;	//@clock speed
	uint8_t SPI_DFF;		//@DFF
	uint8_t SPI_CPOL;		//@CPOL
	uint8_t SPI_CPHA;		//@CPHA
	uint8_t SPI_SSM;		//@SSM

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxState;	/* !< To store Tx state > */
	uint8_t 		RxState;	/* !< To store Rx state > */

}SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/********************API Prototypes *********************************
********************Function Definitions****************************/

//GPIO peripheral clock control
void SPI_clkcontrol(SPI_RegDef_t *pSPIx , uint8_t EnorDi);

//GPIO Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//SPI peripheral enable
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//SSI CONFIGURATION
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//SSOE Configuration
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
//get flag status
uint8_t SPI_Flag_Status(SPI_RegDef_t *pSPIx,uint32_t Flag_name);
//Data send and receive

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length);

//IRQ Config and ISR handling

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriority(uint8_t IRQNumber, uint32_t Priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent);

//@SPI_device mode
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

//@bus_config
#define SPI_BUSCONFIG_FD			1
#define SPI_BUSCONFIG_HD			2
#define SPI_BUSCONFIG_SIMPLEX_RX	3

//@clock speed
#define	SPI_SCLK_SPEED_DIV2			0
#define	SPI_SCLK_SPEED_DIV4			1
#define	SPI_SCLK_SPEED_DIV8			2
#define	SPI_SCLK_SPEED_DIV16		3
#define	SPI_SCLK_SPEED_DIV32		4
#define	SPI_SCLK_SPEED_DIV64		5
#define	SPI_SCLK_SPEED_DIV128		6
#define	SPI_SCLK_SPEED_DIV256		7

//@DFF
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

//@CPOL
#define	SPI_CPOL_HIGH	1
#define	SPI_CPOL_LOW	0

//@CPHA
#define	SPI_CPHA_HIGH	1
#define	SPI_CPHA_LOW	0

//@SSM
#define	SPI_SSM_EN		1
#define	SPI_SSM_DI		0


/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */




