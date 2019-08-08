/*
 * stmf407xx_i2c_driver.c
 *
 *  Created on: Jun 6, 2019
 *      Author: prudh
 */

#include "stm32f407xx.h"
#include <string.h>
//prototypes
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t ReadorWrite);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStop(I2C_RegDef_t *pI2Cx);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);



//Get APB1 bus clock value
uint32_t RCC_GetPCLK1Value(void){
	uint16_t AHB1P[8]={2,4,8,16,64,128,256,512};
	uint16_t APB1P[4]={2,4,8,16};
	uint32_t pclk1,systemclk,temp;
	uint8_t clksrc,ahbp,apb1p;
	clksrc= ((RCC->CFGR>>2) & (0x3));
	if(clksrc==0){
		systemclk=16000000;
	}
	else if(clksrc==1){
		systemclk=8000000;
	}

	temp=((RCC->CFGR>>4)& (0xF));
	if(temp<8){
		ahbp=1;
	}else{
		ahbp=AHB1P[temp-8];
	}

	temp=((RCC->CFGR>>10)& (0x7));
		if(temp<4){
			apb1p=1;
		}else{
			apb1p=APB1P[temp-4];
		}
		pclk1=(systemclk/ahbp)/apb1p;
	return pclk1;
}
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
				if(pI2Cx == I2C1){
					I2C1_CLK_EN;
				}
				else if(pI2Cx == I2C2){
					I2C2_CLK_EN;
				}
				else if(pI2Cx == I2C3){
					I2C3_CLK_EN;
				}
			}

			else{
				if(pI2Cx == I2C1){
					I2C1_CLK_DI;
				}
				else if(pI2Cx == I2C2){
					I2C2_CLK_DI;
				}
				else if(pI2Cx == I2C3){
					I2C3_CLK_DI;
				}
			}
}

void I2C_Init(I2C_Handle_t *pI2CHandle){
	//ACK control
	uint32_t temp=0;
	temp|= (pI2CHandle->I2C_Config.I2C_AckControl<<I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1=temp;
	//FREQ field
	temp=0;
	temp|=RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2=(temp &(0x3F));
	//store device address
	temp|=pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
	temp|=(1<<14);
	pI2CHandle->pI2Cx->OAR1=temp;

	//CCR calculations
		uint16_t ccr_value = 0;
		temp = 0;
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
			ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			temp |= (ccr_value & 0xFFF);
		}else
		{
			//mode is fast mode
			temp |= ( 1 << 15);
			temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
			if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}else
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			temp |= (ccr_value & 0xFFF);
		}
		pI2CHandle->pI2Cx->CCR = temp;

		//Trise Configuration
		uint32_t trise;

		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){

			trise=(RCC_GetPCLK1Value() / 1000000U) +1;
		}else{
			trise=((RCC_GetPCLK1Value()*300) / 1000000000U) +1;
		}
		(pI2CHandle->pI2Cx->TRISE )= (trise & 0x3F);
}



void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx==I2C1){
			I2C1_REG_RST();
		}
		else if(pI2Cx==I2C2){
			I2C2_REG_RST();
		}
		else if(pI2Cx==I2C3){
				I2C3_REG_RST();
		}
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t tempa,tempb;
		tempa=IRQNumber/4;
		tempb=IRQNumber%4;
		uint8_t shift_amount=(8*tempb)+(8-no_ofpr_bits_implemented);

		*(NVIC_PR_BASE + (tempa)) |= (IRQPriority<<shift_amount);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
		{
			pI2Cx->CR1 |= (1 << I2C_CR1_PE);
			//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
		}else
		{
			pI2Cx->CR1 &= ~(1 << 0);
		}
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length,uint8_t SlaveAddr,uint8_t SR){
	//generate start condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);
	//confirm start generation by checking SB in SR register
	//until SB is cleared SCL is stretched
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));
	//send address to slave and r/w bit 0
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr,WRITE);
	//confirm address phase completion by checking ADDR in SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)));
	//clear ADDR flag
	I2C_ClearAddrFlag(pI2CHandle);
	//send the data
	while(Length>0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR=*pTxBuffer;
		pTxBuffer++;
		Length--;
	}
	// when Length becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
		//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
		//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)));



	//Generate STOP condition and master need not to wait for the completion of stop condition.
		//   Note: generating STOP, automatically clears the BTF
	if(SR==I2C_NO_SR){		//no repeated start
		I2C_GenerateStop(pI2CHandle->pI2Cx);
	}



}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length,uint8_t SlaveAddr,uint8_t SR){
	//generate start condition
	I2C_GenerateStart(pI2CHandle->pI2Cx);
	//confirm start generation by checking SB in SR register
	//until SB is cleared SCL is stretched
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));
	//send address to slave and r/w bit 0
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr,READ);
	//confirm address phase completion by checking ADDR in SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)));
	//if read only 1 byte
	if(Length==1){
		//disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		//generate stop condition
		if(SR==I2C_NO_SR){		//no repeated start
				I2C_GenerateStop(pI2CHandle->pI2Cx);
			}
		//clear addr flag
		I2C_ClearAddrFlag(pI2CHandle);
		//wait until rxne becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE)));
		//generate stop condition
	   I2C_GenerateStop(pI2CHandle->pI2Cx);

		//read data into buffer
		*pTxBuffer=pI2CHandle->pI2Cx->DR;

	}
	//if length>1
	if(Length>1){
		//clear addr flag
			I2C_ClearAddrFlag(pI2CHandle);
			//read data until length becomes 0
			for(uint32_t i=Length;i>0;i++){
				//wait until rxne becomes 1
				while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE)));
				//if length=2
				if(i==2){
					//clear ack
					I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

					//generate stop
					if(SR==I2C_NO_SR){		//no repeated start
					I2C_GenerateStop(pI2CHandle->pI2Cx);
				}

				}
				//read data to buffer
				*pTxBuffer=pI2CHandle->pI2Cx->DR;

				//increment buffer address
				pTxBuffer++;
			}

	}
	//re-enable acking
	if(pI2CHandle->I2C_Config.I2C_AckControl==I2C_ACK_ENABLE){

	I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}
void I2C_GenerateStart(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1|=(1<<I2C_CR1_START);
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
	return I2C_FLAG_SET;
	}
	return I2C_FLAG_RESET;

}

void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t ReadorWrite){
	if(ReadorWrite==WRITE){

	SlaveAddr=(SlaveAddr<<1);
	SlaveAddr&=~1;
	pI2Cx->DR=SlaveAddr;
	}
	else if(ReadorWrite==READ){

		SlaveAddr=(SlaveAddr<<1);
		SlaveAddr|=1;
		pI2Cx->DR=SlaveAddr;
	}
}

void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;
if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){
	//MASTER MODE
	if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
		if(pI2CHandle->RxSize==1){
			//disable ack
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
			//clear addr flag
			dummy_read=pI2CHandle->pI2Cx->SR1;
			dummy_read=pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}
		else{
			//clear addr flag
			dummy_read=pI2CHandle->pI2Cx->SR1;
			dummy_read=pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}
	}
}
else{
	//slave mode
	//clear addr flag
	dummy_read=pI2CHandle->pI2Cx->SR1;
	dummy_read=pI2CHandle->pI2Cx->SR2;
	(void)dummy_read;

}


}

void I2C_GenerateStop(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1|=(1<<I2C_CR1_STOP);

}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi==ENABLE){
		pI2Cx->CR1|=(1<<I2C_CR1_ACK);
	}
	else{
		pI2Cx->CR1&=~(1<<I2C_CR1_ACK);

	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length,uint8_t SlaveAddr,uint8_t SR){
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = SR;

		//Implement code to Generate START Condition
		I2C_GenerateStart(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length,uint8_t SlaveAddr,uint8_t SR){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Length;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Length;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = SR;

			//Implement code to Generate START Condition
			I2C_GenerateStart(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		}

		return busystate;
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_EV_IRQ_Handling(I2C_Handle_t *pI2CHandle){
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1,temp2,temp3;

	temp1=pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2=pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);
		//1. Handle For interrupt generated by SB event
		//	Note : SB flag is only applicable in Master mode
	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_SB);
	if(temp1 && temp3){
		//execute address phase
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
				I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr,WRITE);
			}
			else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
				I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr,READ);

			}
	}
	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);

		//2. Handle For interrupt generated by ADDR event
		//Note : When master mode : Address is sent
		//		 When Slave mode   : Address matched with own address
		if(temp1&&temp3){
			I2C_ClearAddrFlag(pI2CHandle);

		}
		temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_BTF);
		//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
		if(temp1&&temp3){
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
				//check if txe is set
				if(pI2CHandle->TxLen==0){

				if(pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE)){
					//generate stop condition
					if(pI2CHandle->Sr==I2C_NO_SR)
						I2C_GenerateStop(pI2CHandle->pI2Cx);
					//reset member elements of handle structure
					I2C_CloseSendData(pI2CHandle);
					//notify application of transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
				}
			}
			else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){

			}
		}
		temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
				//4. Handle For interrupt generated by STOPF event
		// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
		if(temp1&&temp3){
			//clear stopf flag by reading sr1 and writing to cr1
			pI2CHandle->pI2Cx->CR1|=0x0;
			//notify application stop is detected
			I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);

		}
		temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE);
		//5. Handle For interrupt generated by TXE event
		if(temp1&&temp2&&temp3){
			//check if device mode is master
			if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
				if(pI2CHandle->TxLen>0){
					//load data to DR
					pI2CHandle->pI2Cx->DR=*(pI2CHandle->pTxBuffer);
					//decrement len
					pI2CHandle->TxLen--;
					//incremebt buffer
					pI2CHandle->pTxBuffer++;
				}
			}



			}
		}
		temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_RXNE);
		//6. Handle For interrupt generated by RXNE event
		if(temp1&&temp2&&temp3){
			if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
				if(pI2CHandle->RxSize==1){

					*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;

				}
				if(pI2CHandle->RxSize>1){
					if(pI2CHandle->RxSize==2){
						I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
					}

					*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;


				}
				if(pI2CHandle->RxSize==0){
					//stop generation and close reception and notify application
					I2C_GenerateStop(pI2CHandle->pI2Cx);
					I2C_CloseReceiveData(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
				}
			}

		}
		}
}
void I2C_ER_IRQ_Handling(I2C_Handle_t *pI2CHandle){

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data){
	pI2C->DR = data;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){
	uint8_t data= pI2C->DR ;
	return data;
}



