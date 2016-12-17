/*
 * joystick.cpp
 *
 *  Created on: 16. 12. 2016
 *      Author: Patrik Bakyta
 */

#include "stm32f4xx.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stdlib.h>
#include <misc.h>
#include <joystick.h>

extern volatile uint8_t array[2];

char *char_pointer;
int i;

void initSYSTEMCLOCK(void) {

	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	RCC_SYSCLKConfig(RCC_CFGR_SW_HSI);
	SystemCoreClockUpdate();

	//uint32_t SystemClockValue = SystemCoreClock;

	return;

}

void initLED(void) {

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitTypeDef initStruct;
	initStruct.GPIO_Pin = GPIO_Pin_12;
	initStruct.GPIO_Mode = GPIO_Mode_OUT;
	initStruct.GPIO_OType = GPIO_OType_PP;
	initStruct.GPIO_PuPd = GPIO_PuPd_UP;
	initStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&initStruct);

	return;

}

void initTIMER(TIM_TypeDef* Timer, uint16_t TimerVal) {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 8000-1;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = TimerVal-1;
	timerInitStructure.TIM_ClockDivision = 0;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(Timer, &timerInitStructure);
	TIM_Cmd(Timer, ENABLE);
	TIM_ITConfig(Timer, TIM_IT_Update, ENABLE); // povolenie update eventu

	return;
}

void EnableTimerInterrupt(void) {

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    return;
}

extern "C" void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		GPIO_SetBits(GPIOD, GPIO_Pin_12); // LED sa pre posielanim zazne

		// najprv sa posle x potom y
		for (int j=0; j<2; j++) {

			// konverzia hodnoty z ADC na pole charov, funkcia vracia smernik
			char_pointer = INTconversionCHAR(array[j]);

			i = *(char_pointer); // na 1. mieste je pocet cifier
			while (i>0) {
				while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
				USART_SendData(USART1,*(char_pointer+i));
				i--;
			}

			// najprv space a pootm new line
			while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
			if (j==0) {
				USART_SendData(USART1,32);
			} else {
				USART_SendData(USART1,13);
			}

		}

		GPIO_ResetBits(GPIOD, GPIO_Pin_12); // LED po poslani zhasne

		/*
		// priame posielanie hodnot
		while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
		USART_SendData(USART1,array[0]);
		while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
		USART_SendData(USART1,array[1]);
		*/
	}

	return;

}

char *INTconversionCHAR(uint8_t value) {

	// funkcia vracia smernik na pole

	int j = 1; // index pola, zacina na 1
	static char char_array[4]; // pole, na 1. mieste pocet cifier hodnoty z ADC (1-3)

	do {
		*(char_array+j) = (char)(value % 10) + '0'; // konverzia z INT na CHAR
		value /= 10;
		j++;
	} while (value);

	*(char_array) = j-1; // teraz uz vieme pocet cifier, zapis na 1. miesto v poli

	return char_array;
}

void initUSART(void) {

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	 GPIO_InitTypeDef GPIO_InitStruct;    // this is for the GPIO pins used as TX and RX
	 USART_InitTypeDef USART_InitStruct;  // this is for the USART1 initilization
	 NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	 /* enable APB2 peripheral clock for USART1
	  * note that only USART1 and USART6 are connected to APB2
	  * the other USARTs are connected to APB1
	  */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	 /* enable the peripheral clock for the pins used by
	  * USART1, PB6 for TX and PB7 for RX
	  */
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	 /* This sequence sets up the TX and RX pins
	  * so they work correctly with the USART1 peripheral
	  */
	 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			 // the pins are configured as alternate function so the USART peripheral has access to them
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 // this defines the IO speed and has nothing to do with the baudrate!
	 GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		 // this defines the output type as push pull mode (as opposed to open drain)
	 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			 // this activates the pullup resistors on the IO pins
	 GPIO_Init(GPIOB, &GPIO_InitStruct);				 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	 /* The RX and TX pins are now connected to their AF
	  * so that the USART1 can take over control of the
	  * pins
	  */
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	 /* Now the USART_InitStruct is used to define the
	  * properties of USART1
	  */
	 USART_InitStruct.USART_BaudRate = 9600;				 // the baudrate is set to the value we passed into this init function
	 USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	 USART_InitStruct.USART_StopBits = USART_StopBits_1;	 // we want 1 stop bit (standard)
	 USART_InitStruct.USART_Parity = USART_Parity_No;		 // we don't want a parity bit (standard)
	 USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	 USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	 USART_Init(USART1, &USART_InitStruct);					 // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	 /* Here the USART1 receive interrupt is enabled
	  * and the interrupt controller is configured
	  * to jump to the USART1_IRQHandler() function
	  * if the USART1 receive interrupt occurs
	  */
	 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // we want to configure the USART1 interrupts
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8; // this sets the priority group of the USART1 interrupts
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // this sets the subpriority inside the group
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // the USART1 interrupts are globally enabled
	 NVIC_Init(&NVIC_InitStructure);							  // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	 // finally this enables the complete USART1 peripheral
	 USART_Cmd(USART1, ENABLE);

	 return;
}

void initADC(void) {

	ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	//Clock configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN,ENABLE);//Clock for the ADC port!! Do not forget about this one ;)
	//Analog pin configuration
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_0;//The channel 10 is connected to PC0
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(GPIOC,&GPIO_initStructre);//Affecting the port with the initialization structure configuration

	//Analog pin configuration
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_1;//The channel 10 is connected to PC0
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(GPIOC,&GPIO_initStructre);//Affecting the port with the initialization structure configuration

	//ADC structure configuration
	ADC_DeInit();
	ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	ADC_init_structure.ADC_Resolution = ADC_Resolution_8b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
	ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
	ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	ADC_init_structure.ADC_NbrOfConversion = 2;//I think this one is clear :p
	ADC_init_structure.ADC_ScanConvMode = ENABLE;//The scan is configured in one channel
	ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration
	//Enable ADC conversion
	ADC_Cmd(ADC1,ENABLE);

	return;
}

uint8_t ADC_Convert_x(void) {

	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_144Cycles);

	ADC_SoftwareStartConv(ADC1);//Start the conversion
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion

	return ADC_GetConversionValue(ADC1); //Return the converted data;
}

uint8_t ADC_Convert_y(void) {

	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_144Cycles);

	ADC_SoftwareStartConv(ADC1);//Start the conversion
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion

	return ADC_GetConversionValue(ADC1); //Return the converted data;
}
