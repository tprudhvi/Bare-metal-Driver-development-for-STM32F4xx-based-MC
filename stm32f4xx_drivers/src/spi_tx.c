/*
 * spi_tx.c
 *
 *  Created on: Jun 4, 2019
 *      Author: prudh
 */
#include <string.h>
#include "stm32f407xx.h"
//PB15 -- SPI2_MOSI
//PB14 -- SPI2_MISO
//PB13 -- SPI2_SCLK
//PB12 -- SPI2_NSS
//ALTERNATE FUN MODE -- 5

void SPI2_GPIO_Init(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx= GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_pinMode=GPIO_MODE_ALT;
	SPIPins.GPIO_PinConfig.GPIO_pinAltFunc= 5;
	SPIPins.GPIO_PinConfig.GPIO_pinOPType=GPIO_OUT_PUP;
	SPIPins.GPIO_PinConfig.GPIO_pinPuPd= GPIO_NOPUPD;
	SPIPins.GPIO_PinConfig.GPIO_pinSpeed=GPIO_SPEED_LOW;

	SPIPins.GPIO_PinConfig.GPIO_pinNumber=GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//SPIPins->GPIO_PinConfig.GPIO_pinNumber=GPIO_PIN_12;
	//GPIO_Init(&SPIPins);

	//SPIPins->GPIO_PinConfig.GPIO_pinNumber=GPIO_PIN_14;
	//GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_pinNumber=GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}
void SPI2_Init(){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig=SPI_BUSCONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM=SPI_SSM_EN;
	SPI_Init(&SPI2handle);

}


int main(void){
	char data[]= "Hello World";
	SPI2_GPIO_Init();
	SPI2_Init();
	SPI_SSIConfig(SPI2,ENABLE);
	SPI_Peripheral_Control(SPI2,ENABLE);
	SPI_SendData(SPI2,(uint8_t*)data,strlen(data));
	SPI_Peripheral_Control(SPI2,DISABLE);
	while(1);
	return 0;
}
