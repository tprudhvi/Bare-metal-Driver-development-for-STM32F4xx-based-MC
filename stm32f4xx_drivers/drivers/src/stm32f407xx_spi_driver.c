/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 4, 2019
 *      Author: prudh
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include <stddef.h>

static void SPI_TXE_INT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_INT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_INT_HANDLE(SPI_Handle_t *pSPIHandle);

void SPI_clkcontrol(SPI_RegDef_t *pSPIx , uint8_t EnorDi){
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1){
				SPI1_CLK_EN;
			}
			else if(pSPIx == SPI2){
				SPI2_CLK_EN;
			}
			else if(pSPIx == SPI3){
				SPI3_CLK_EN;
			}
		}

		else{
			if(pSPIx == SPI1){
				SPI1_CLK_DI;
			}
			else if(pSPIx == SPI2){
				SPI2_CLK_DI;
			}
			else if(pSPIx == SPI3){
				SPI3_CLK_DI;
			}
		}

}

void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_clkcontrol(pSPIHandle->pSPIx,ENABLE);

	uint32_t temp=0;
	//configure device mode
	temp|= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//SPI bus configuration

	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUSCONFIG_FD){
		//BIDI mode cleared
		temp&= ~(1<< SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUSCONFIG_HD){
		//BIDI mode set
		temp|= (1<< SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUSCONFIG_SIMPLEX_RX){
		//BIDI mode cleared and RX only mode set
		temp&= ~(1 << SPI_CR1_BIDIMODE);
		temp|= (1<< SPI_CR1_RXONLY);
	}

	//configuring clock speed
	temp|=(pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//configuring DFF bits
	temp|=(pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	//configure CPOL
	temp|=(pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	//configure CPHA
	temp|=(pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	//configure SSM
	temp|=(pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 =temp;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx==SPI1){
		SPI1_REG_RST();
	}
	else if(pSPIx==SPI2){
		SPI2_REG_RST();
	}
	else if(pSPIx==SPI3){
			SPI3_REG_RST();
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		pSPIx->CR1|=(1<<SPI_CR1_SSI);
	}
	else{
		pSPIx->CR1&=~(1<<SPI_CR1_SSI);


}
}

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		pSPIx->CR1|=(1<<SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1&=~(1<<SPI_CR1_SPE);

	}
}

uint8_t SPI_Flag_Status(SPI_RegDef_t *pSPIx,uint32_t Flag_name){

	if(pSPIx->SR & Flag_name){
	return SPI_FLAG_SET;
	}
	return SPI_FLAG_RESET;

}
//blocking call

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length){
	while(Length>0){
		//wait until TXE is set
		while(SPI_Flag_Status(pSPIx,SPI_TXE_FLAG) == SPI_FLAG_RESET);// polling type code

		//check DFF BIT IN CR1
		if((pSPIx->CR1 & (1<<SPI_CR1_DFF))){
			//16 bit DFF
			//load data to DR
			pSPIx->DR=*((uint16_t*)pTxBuffer);
			Length--;
			Length--;
			(uint16_t*)pTxBuffer++;
		}
		else{
			//8bit DFF
			//load data to DR
			pSPIx->DR=*pTxBuffer;
			Length--;
			pTxBuffer++;
		}



	}
}

void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}


}

void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length){
	while(Length>0){
		//wait until TXE is set
		while(SPI_Flag_Status(pSPIx,SPI_RXNE_FLAG) == SPI_FLAG_RESET);// polling type code

		//check DFF BIT IN CR1
		if((pSPIx->CR1 & (1<<SPI_CR1_DFF))){
			//16 bit DFF
			//load data from DR to RXbuffer
			*((uint16_t*)pRxBuffer)=pSPIx->DR;
			Length--;
			Length--;
			(uint16_t*)pRxBuffer++;
		}
		else{
			//8bit DFF
			//load data from DR to RXbuffer
			*pRxBuffer=pSPIx->DR;
			Length--;
			pRxBuffer++;
		}

	}
}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi==ENABLE){
				if(IRQNumber <= 31){
					//ISER0 REG
					*NVIC_ISER0|= (1<<IRQNumber);
				}
				else if(IRQNumber>31 && IRQNumber <64){
					//ISER1 REG

					*NVIC_ISER1|= (1<<(IRQNumber%32));
				}
				else if(IRQNumber>=64 && IRQNumber< 96){
					//ISER2 REG

					*NVIC_ISER2|= (1<<(IRQNumber%64));
				}
			}
			else{
				if(IRQNumber <= 31){
						//ICER0
					*NVIC_ICER0|= (1<<IRQNumber);
						}
						else if(IRQNumber>31 && IRQNumber <64){
							//ICER1

							*NVIC_ICER1|= (1<<(IRQNumber%32));
						}
						else if(IRQNumber>=64 && IRQNumber< 96){
							//ICER2

							*NVIC_ICER2|= (1<<(IRQNumber%64));
						}

			}

}

void SPI_IRQPriority(uint8_t IRQNumber, uint32_t Priority){

	uint8_t tempa,tempb;
	tempa=IRQNumber/4;
	tempb=IRQNumber%4;
	uint8_t shift_amount=(8*tempb)+(8-no_ofpr_bits_implemented);

	*(NVIC_PR_BASE + (tempa)) |= (Priority<<shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length){

	uint8_t status=pSPIHandle->TxState;
	if(status!=SPI_BUSY_IN_TX){

	//Save TX buffer address and len in global vaiables

	pSPIHandle->pTxBuffer=pTxBuffer;
	pSPIHandle->TxLen=Length;

	//mark spi state as busy
	pSPIHandle->TxState=SPI_BUSY_IN_TX;
	//enable TXEIE
	pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);

	}

	return status;
}
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length){
	uint8_t status=pSPIHandle->RxState;
	if(status!=SPI_BUSY_IN_RX){

	//Save TX buffer address and len in global vaiables

	pSPIHandle->pRxBuffer=pRxBuffer;
	pSPIHandle->RxLen=Length;

	//mark spi state as busy
	pSPIHandle->RxState=SPI_BUSY_IN_RX;
	//enable TXEIE
	pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_RXNEIE);

	}

	return status;
}


static void SPI_TXE_INT_HANDLE(SPI_Handle_t *pSPIHandle){

	//check DFF BIT IN CR1
			if((pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))){
				//16 bit DFF
				//load data to DR
				pSPIHandle->pSPIx->DR=*((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else{
				//8bit DFF
				//load data to DR
				pSPIHandle->pSPIx->DR=*pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			if(!pSPIHandle->TxLen){
				//deactivate transmission
				SPI_CloseTransmisson(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
			}

}
static void SPI_RXNE_INT_HANDLE(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;

		}else
		{
			//8 bit
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}

		if(! pSPIHandle->RxLen)
		{
			//reception is complete
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}

}
static void SPI_OVR_INT_HANDLE(SPI_Handle_t *pSPIHandle){
		//CLEAR OVR FLAG
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2&= ~(1<<SPI_CR2_TXEIE);
					pSPIHandle->pTxBuffer=NULL;
					pSPIHandle->TxLen=0;
					pSPIHandle->TxState=SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2&= ~(1<<SPI_CR2_RXNEIE);
				pSPIHandle->pRxBuffer=NULL;
				pSPIHandle->RxLen=0;
				pSPIHandle->RxState=SPI_READY;
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	uint8_t temp1,temp2;
	//check for txe interrupt
	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_TXE);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//Handle the TX
		SPI_TXE_INT_HANDLE(pSPIHandle);
	}
		//check for RXNE interrupt
		temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_RXNE);
		temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		//Handle the RX
		SPI_RXNE_INT_HANDLE(pSPIHandle);
	}

	//check for ovr flag
	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_OVR);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_ERRIE);
	if(temp1 && temp2){
			//Handle the ERROR
			SPI_OVR_INT_HANDLE(pSPIHandle);
		}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent){

}








