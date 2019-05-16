/****************************************************************************
*  Copyright (c) 2012 by Michael Blandford. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*
****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBXLITE) || defined(PCBX9LITE)
#include "X9D/stm32f2xx.h"
#include "X9D/stm32f2xx_gpio.h"
#include "X9D/stm32f2xx_rcc.h"
#include "X9D/hal.h"
#endif

#include "core_cm3.h"

#include "ersky9x.h"
#include "timers.h"
#include "logicio.h"
#include "analog.h"
#include "myeeprom.h"


#define STICK_LV	3
#define STICK_LH  2
#define STICK_RV  0
#define STICK_RH  1
#define POT_L			6
#define POT_R			8
#define SLIDE_L		14
#define SLIDE_R		15
#define BATTERY		10
#define POT_3			9
#define V_BATT		18


static void enableRtcBattery()
{
	ADC->CCR |= ADC_CCR_VBATE ;
}

void disableRtcBattery()
{
	ADC->CCR &= ~ADC_CCR_VBATE ;
}


// Sample time should exceed 1uS
#define SAMPTIME	2		// sample time = 28 cycles

volatile uint16_t Analog_values[NUMBER_ANALOG+NUM_POSSIBLE_EXTRA_POTS] ;
uint16_t VbattRtc ;

void init_adc()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;			// Enable clock
	RCC->AHB1ENR |= RCC_AHB1Periph_GPIOADC ;	// Enable ports A&C clocks (and B for REVPLUS)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN ;		// Enable DMA2 clock

	configure_pins( PIN_STK_J1 | PIN_STK_J2 | PIN_STK_J3 | PIN_STK_J4 |
									PIN_FLP_J1 , PIN_ANALOG | PIN_PORTA ) ;
	configure_pins( PIN_FLP_J2 | PIN_FLP_J3, PIN_ANALOG | PIN_PORTB ) ;
	configure_pins( PIN_SLD_J1 | PIN_SLD_J2 | PIN_MVOLT, PIN_ANALOG | PIN_PORTC ) ;

	ADC1->CR1 = ADC_CR1_SCAN ;
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS ;

	ADC1->SQR1 = (NUMBER_ANALOG-1) << 20 ;		// NUMBER_ANALOG Channels
	ADC1->SQR2 = SLIDE_L + (SLIDE_R<<5) + (BATTERY<<10) + (POT_3<<15) + (V_BATT<<20) ;
	ADC1->SQR3 = STICK_LH + (STICK_LV<<5) + (STICK_RV<<10) + (STICK_RH<<15) + (POT_L<<20) + (POT_R<<25) ;
	ADC1->SMPR1 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) ;
	ADC1->SMPR2 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) + (SAMPTIME<<27) ;

	ADC->CCR = 0 ; //ADC_CCR_ADCPRE_0 ;		// Clock div 2

	DMA2_Stream4->CR = DMA_SxCR_PL | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC ;	// Channel 0
	DMA2_Stream4->PAR = (uint32_t) &ADC1->DR ;
	DMA2_Stream4->M0AR = (uint32_t) Analog_values ;
	DMA2_Stream4->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 ;

	enableRtcBattery() ;
}

volatile uint8_t adc_in_progress = 0;

uint32_t read_adc()
{
	uint32_t i ;

	if (adc_in_progress) {
		return 0;
	}
	adc_in_progress = 1;

	DMA2_Stream4->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	ADC1->SR &= ~(uint32_t) ( ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR ) ;
	DMA2->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 |DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4 ; // Write ones to clear bits
	DMA2_Stream4->M0AR = (uint32_t) Analog_values ;

	DMA2_Stream4->NDTR = NUMBER_ANALOG-NUM_REMOTE_ANALOG ;
	DMA2_Stream4->CR |= DMA_SxCR_EN ;		// Enable DMA
	ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART ;
	for ( i = 0 ; i < 20000 ; i += 1 )
	{
		if ( DMA2->HISR & DMA_HISR_TCIF4 )
		{
			break ;
		}
	}
	DMA2_Stream4->CR &= ~DMA_SxCR_EN ;		// Disable DMA

	AnalogData[0] = Analog_values[0] ;
	AnalogData[1] = 4096 - Analog_values[1] ;
	AnalogData[2] = Analog_values[2] ;
	AnalogData[3] = 4096 - Analog_values[3] ;
	AnalogData[4] = Analog_values[4] ;
	AnalogData[5] = Analog_values[5] ;
	AnalogData[6] = Analog_values[6] ;
	AnalogData[7] = Analog_values[7] ;
	AnalogData[12] = Analog_values[8] ;
	if (ADC->CCR & ADC_CCR_VBATE )
	{
		VbattRtc = Analog_values[9] ;
	}
	AnalogData[8] = Analog_values[9] ;
	if (ADC->CCR & ADC_CCR_VBATE )
	{
		VbattRtc = Analog_values[10] ;
	}

	adc_in_progress = 0;

	return ( i < 20000 ) ? 1 : 0 ;
}

// This to read a single channel for use as a rotary encoder
// Channel is POT_L (6) or POT_R (8) or POT_3 (9)
void init_adc2()
{
	ADC2->CR2 = 0 ;
	TIM5->CR1 = 0 ;
	if ( g_eeGeneral.analogMapping & ENC_MASK )
	{
		uint32_t channel = (g_eeGeneral.analogMapping & ENC_MASK ) + 5 ; // gives 6, 7 or 8
		if ( channel > 6 )
		{
			channel += 1 ;	// 7->8 and 8->9
		}
		RCC->APB2ENR |= RCC_APB2ENR_ADC2EN ;			// Enable clock

		ADC2->CR1 = ADC_CR1_SCAN | ADC_CR1_EOCIE ;
		ADC2->SQR1 = (1-1) << 20 ;		// NUMBER_ANALOG Channels
		ADC2->SQR2 = 0 ;
		ADC2->SQR3 = channel ;

		ADC2->SMPR1 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
									+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) ;
		ADC2->SMPR2 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
									+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) + (SAMPTIME<<27) ;

		ADC2->CR2 = ADC_CR2_ADON | ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTEN_0 ;

  	NVIC_SetPriority(ADC_IRQn, 7);
		NVIC_EnableIRQ(ADC_IRQn) ;

	// TIM5, CC1 event is ADC2 trigger
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN ;		// Enable clock
		TIM5->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;		// 0.5uS
		TIM5->ARR = 1999 ;
		TIM5->CCR1 = 1000 ;
		TIM5->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 ;
		TIM5->EGR = TIM_EGR_CC1G ;
		TIM5->CCER = TIM_CCER_CC1E ;
		TIM5->CR1 = TIM_CR1_CEN ;
	}
	else
	{
		RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN ;		// Disable clock
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN ;		// Disable clock
	}
}

#ifndef PCB9XT
uint16_t RotaryAnalogValue ;

extern "C" void ADC_IRQHandler()
{
	uint32_t x ;
	x = ADC2->DR ;
	int32_t diff = x - RotaryAnalogValue ;
	if ( diff < 0 )
	{
		diff = -diff ;
	}
	RotaryAnalogValue = x ;
	if ( diff < 20 )
	{
		valueprocessAnalogEncoder( x >> 1 ) ;
	}
}
#endif // nPCB9XT

// TODO
void stop_adc()
{
}
