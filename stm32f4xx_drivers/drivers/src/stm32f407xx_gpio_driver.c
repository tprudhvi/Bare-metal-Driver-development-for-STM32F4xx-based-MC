/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 31, 2019
 *      Author: prudh
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

//GPIO peripheral clock control
/************************************
 * @fn			-	GPIO_clkcontrol
 * @brief		-	This function enables or disables the clock for GPIO port
 *
 * @param[in]	-	base address of gpio peripheral
 * @param[in]	- 	ENABLE or DISABLE MACROS
 *
 *
 * @return		- none
 *
 * @note		- none
 */
void GPIO_clkcontrol(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_CLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_CLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_CLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_CLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_CLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_CLK_EN();
		}
		else if(pGPIOx == GPIOJ){
			GPIOJ_CLK_EN();
		}
		else if(pGPIOx == GPIOK){
			GPIOK_CLK_EN();
		}

	}

	else{
		if(pGPIOx == GPIOA){
					GPIOA_CLK_DI();
				}
				else if(pGPIOx == GPIOB){
					GPIOB_CLK_DI();
				}
				else if(pGPIOx == GPIOC){
					GPIOC_CLK_DI();
				}
				else if(pGPIOx == GPIOD){
					GPIOD_CLK_DI();
				}
				else if(pGPIOx == GPIOE){
					GPIOE_CLK_DI();
				}
				else if(pGPIOx == GPIOF){
					GPIOF_CLK_DI();
				}
				else if(pGPIOx == GPIOG){
					GPIOG_CLK_DI();
				}
				else if(pGPIOx == GPIOH){
					GPIOH_CLK_EN();
				}
				else if(pGPIOx == GPIOI){
					GPIOI_CLK_DI();
				}
				else if(pGPIOx == GPIOJ){
					GPIOJ_CLK_DI();
				}
				else if(pGPIOx == GPIOK){
						GPIOK_CLK_DI();
				}
	}
}
//GPIO Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	GPIO_clkcontrol(pGPIOHandle->pGPIOx,ENABLE);

	//configure gpio mode
	uint32_t temp=0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode<=GPIO_MODE_ANALOG){

		temp =  pGPIOHandle->GPIO_PinConfig.GPIO_pinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{
			//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode<=GPIO_MODE_INT_FT){

			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
			EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode<=GPIO_MODE_INT_RT){
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode<=GPIO_MODE_INT_FRT){
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		}

		//enable interrupt delivery in IMR
		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);

		//configure sysscfg exti registers

	uint8_t temp1=  pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber /4;
	uint8_t temp2=  pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber %4;
	uint8_t portcode= GPIO_PORT_CODE(pGPIOHandle->pGPIOx);
	SYSCFG_CLK_EN;
	SYSCFG->EXTICR[temp1]|=( portcode<< 4* temp2);

	}
	temp = 0;

	//configure gpio speed
	temp =  pGPIOHandle->GPIO_PinConfig.GPIO_pinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;

	//configure pupd settings

	temp =  pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPd << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	//configure output type
	temp =  pGPIOHandle->GPIO_PinConfig.GPIO_pinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	//configure alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode== GPIO_MODE_ALT){

		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<< (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_pinAltFunc << (4 * temp2);
	}







}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RST();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RST();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RST();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RST();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RST();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RST();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RST();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RST();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RST();
	}
}



//Data read and write
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber){

	uint8_t value;
	value= pGPIOx->IDR >> (uint8_t)(PinNumber) & (0x00000001);
	return value;
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx ){

	uint16_t value;
	value= (uint16_t) pGPIOx->IDR;
	return value;
}
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber, uint8_t value){

	pGPIOx->ODR |= (value << PinNumber);
}
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx , uint16_t value){
	pGPIOx->ODR = (value);
}
void GPIO_Togglepin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);

}

//GPIO interrupt config and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){

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

void GPIO_IRQPriority(uint8_t IRQNumber, uint32_t Priority){

	uint8_t tempa,tempb;
	tempa=IRQNumber/4;
	tempb=IRQNumber%4;
	uint8_t shift_amount=(8*tempb)+(8-no_ofpr_bits_implemented);

	*(NVIC_PR_BASE + (tempa)) |= (Priority<<shift_amount);
}
void GPIO_IRQHandling( uint8_t PinNumber){

	if(EXTI->PR&(1<<PinNumber)){
		EXTI->PR|=(1<<PinNumber);
	}
}
