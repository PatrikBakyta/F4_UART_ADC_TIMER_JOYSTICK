/*
 * joystick.h
 *
 *  Created on: 16. 12. 2016
 *      Author: Patrik Bakyta
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_


void initSYSTEMCLOCK(void);

void initLED(void);

void initTIMER(TIM_TypeDef* Timer, uint16_t TimerVal);
void EnableTimerInterrupt(void);
extern "C" void TIM2_IRQHandler(void);

void initUSART(void);

void initADC(void);
uint8_t ADC_Convert_x(void);
uint8_t ADC_Convert_y(void);

char *INTconversionCHAR(uint8_t value);


#endif /* JOYSTICK_H_ */
