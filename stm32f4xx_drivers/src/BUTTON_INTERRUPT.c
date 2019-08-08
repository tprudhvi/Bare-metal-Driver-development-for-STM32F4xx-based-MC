/*
 * main.c
 *
 *  Created on: Jun 2, 2019
 *      Author: prudh
 */
#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

void delay(void){
	uint32_t i;
	for(i=0;i<=500000;i++);
}



int main(void){



	GPIO_Handle_t pGpio,pGpioBtn;

	//set the member elements of structure to 0
	memset(&pGpio,0,sizeof(pGpio));
	memset(&pGpioBtn,0,sizeof(pGpioBtn));

		pGpio.pGPIOx=GPIOD;
		pGpio.GPIO_PinConfig.GPIO_pinNumber=GPIO_PIN_12;
		pGpio.GPIO_PinConfig.GPIO_pinMode= GPIO_MODE_OUT;
		pGpio.GPIO_PinConfig.GPIO_pinOPType=GPIO_OUT_PUP;
		pGpio.GPIO_PinConfig.GPIO_pinPuPd=GPIO_NOPUPD;
		pGpio.GPIO_PinConfig.GPIO_pinSpeed=GPIO_SPEED_LOW;


		pGpioBtn.pGPIOx=GPIOA;
		pGpioBtn.GPIO_PinConfig.GPIO_pinNumber=GPIO_PIN_0;
		pGpioBtn.GPIO_PinConfig.GPIO_pinMode= GPIO_MODE_INT_FT;
		pGpioBtn.GPIO_PinConfig.GPIO_pinOPType=GPIO_OUT_PUP;
		pGpioBtn.GPIO_PinConfig.GPIO_pinPuPd=GPIO_NOPUPD;
		pGpioBtn.GPIO_PinConfig.GPIO_pinSpeed=GPIO_SPEED_LOW;



		GPIO_clkcontrol(GPIOD ,ENABLE);
		GPIO_Init(&pGpio);

		GPIO_clkcontrol(GPIOA ,ENABLE);
		GPIO_Init(&pGpioBtn);

		GPIO_IRQConfig(IRQ_EXTI0,ENABLE);

		while(1);


}
void EXTI0_IRQHandler(void){
			delay();
			GPIO_IRQHandling(GPIO_PIN_0);
			GPIO_Togglepin(GPIOD,GPIO_PIN_12);

}


