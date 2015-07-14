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


#ifdef PCBSKY
#include "AT91SAM3S4.h"
#endif

#if defined(PCBX9D) || defined(PCBSP)
#include "x9d\stm32f2xx.h"
#include "x9d\stm32f2xx_gpio.h"
#include "x9d\stm32f2xx_rcc.h"
#include "x9d\hal.h"
#endif


#ifndef SIMU
#include "core_cm3.h"
#endif

#include "ersky9x.h"
#include "timers.h"
#include "logicio.h"
#include "analog.h"
#include "myeeprom.h"


#ifndef PCBSP
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
#else
#define STICK_LV	10
#define STICK_LH  11
#define STICK_RV  12
#define STICK_RH  13
#define BATTERY		8
#define DIG1			6
#define DIG2			9
#define DIG3			14
#endif // nPCBSP

#ifdef REV9E
#define FLAP_3		6
#define FLAP_4		7
#define FLAP_5		8
#define FLAP_6		9

#endif	// REV9E

#ifdef REV9E
void init_adc3( void ) ;
#endif	// REV9E

// Sample time should exceed 1uS
#define SAMPTIME	2		// sample time = 28 cycles
//#define SAMPTIME	3		// sample time = 56 cycles
//#define SAMPTIME	6		// sample time = 144 cycles

volatile uint16_t Analog_values[NUMBER_ANALOG+NUM_EXTRA_ANALOG] ;

void init_adc()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;			// Enable clock
	RCC->AHB1ENR |= RCC_AHB1Periph_GPIOADC ;	// Enable ports A&C clocks (and B for REVPLUS)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN ;		// Enable DMA2 clock

#ifndef PCBSP
	configure_pins( PIN_STK_J1 | PIN_STK_J2 | PIN_STK_J3 | PIN_STK_J4 |
									PIN_FLP_J1 , PIN_ANALOG | PIN_PORTA ) ;

#ifdef REV9E
	configure_pins( PIN_FLP_J2 | PIN_FLAP6, PIN_ANALOG | PIN_PORTB ) ;
#else	 
 #ifdef REVPLUS
	configure_pins( PIN_FLP_J2 | PIN_FLP_J3, PIN_ANALOG | PIN_PORTB ) ;
 #else	 
	configure_pins( PIN_FLP_J2, PIN_ANALOG | PIN_PORTB ) ;
 #endif	// REVPLUS
#endif	// REV9E
	
	configure_pins( PIN_SLD_J1 | PIN_SLD_J2 | PIN_MVOLT, PIN_ANALOG | PIN_PORTC ) ;
#else	 
	configure_pins( PIN_STK_J1 | PIN_STK_J2 | PIN_STK_J3 | PIN_SW3, PIN_ANALOG | PIN_PORTC ) ;
	configure_pins( PIN_MVOLT | PIN_SW2, PIN_ANALOG | PIN_PORTB ) ;
	configure_pins( PIN_SW1, PIN_ANALOG | PIN_PORTA ) ;
#endif // PCBSP
				

	ADC1->CR1 = ADC_CR1_SCAN ;
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS ;
	
	
#ifndef PCBSP
	ADC1->SQR1 = (NUMBER_ANALOG-1) << 20 ;		// NUMBER_ANALOG Channels
#ifdef REVPLUS
 #ifdef REV9E
	ADC1->SQR2 = SLIDE_L + (SLIDE_R<<5) + (BATTERY<<10) + (FLAP_6<<15) ;
 #else	 
	ADC1->SQR2 = SLIDE_L + (SLIDE_R<<5) + (BATTERY<<10) + (POT_3<<15) ;
 #endif	// REV9E
#else
	ADC1->SQR2 = SLIDE_L + (SLIDE_R<<5) + (BATTERY<<10) ;
#endif
	ADC1->SQR3 = STICK_LH + (STICK_LV<<5) + (STICK_RV<<10) + (STICK_RH<<15) + (POT_L<<20) + (POT_R<<25) ;
	ADC1->SMPR1 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) ;
	ADC1->SMPR2 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12) 
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) + (SAMPTIME<<27) ;

#else	 
// ADD PCBSP here
	ADC1->SQR1 = (NUMBER_ANALOG-1-NUM_REMOTE_ANALOG) << 20 ;		// NUMBER_ANALOG Channels
	ADC1->SQR2 = DIG2 + (DIG3<<5) ;
	ADC1->SQR3 = STICK_LH + (STICK_LV<<5) + (STICK_RV<<10) + (STICK_RH<<15) + (BATTERY<<20) + (DIG1<<25) ;
	ADC1->SMPR1 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) ;

#endif // PCBSP
	 
	ADC->CCR = 0 ; //ADC_CCR_ADCPRE_0 ;		// Clock div 2
	
	DMA2_Stream0->CR = DMA_SxCR_PL | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC ;
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR ;
	DMA2_Stream0->M0AR = (uint32_t) Analog_values ;
	DMA2_Stream0->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 ;
#ifdef REV9E
	init_adc3() ;
#endif	// REV9E
}

uint32_t read_adc()
{
	uint32_t i ;
	
	DMA2_Stream0->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	ADC1->SR &= ~(uint32_t) ( ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR ) ;
	DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 ; // Write ones to clear bits
	DMA2_Stream0->M0AR = (uint32_t) Analog_values ;
	DMA2_Stream0->NDTR = NUMBER_ANALOG-NUM_REMOTE_ANALOG ;
	DMA2_Stream0->CR |= DMA_SxCR_EN ;		// Enable DMA
#ifdef REV9E
	DMA2_Stream1->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	ADC3->SR &= ~(uint32_t) ( ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR ) ;
	DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 |DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1 ; // Write ones to clear bits
	DMA2_Stream1->M0AR = (uint32_t) &Analog_values[NUMBER_ANALOG] ;
	DMA2_Stream1->NDTR = NUM_EXTRA_ANALOG ;
	DMA2_Stream1->CR |= DMA_SxCR_EN ;		// Enable DMA
	ADC3->CR2 |= (uint32_t)ADC_CR2_SWSTART ;
#endif	// REV9E
	ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART ;
	for ( i = 0 ; i < 20000 ; i += 1 )
	{
		if ( DMA2->LISR & DMA_LISR_TCIF0 )
		{
			break ;
		}
	}
	DMA2_Stream0->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	return ( i < 20000 ) ? 1 : 0 ;
}

#ifndef REV9E
#ifndef PCBSP

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
#endif // nPCBSP
#endif // nREV9E

#ifndef PCBSP
#ifdef REV9E
void init_adc3()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN ;			// Enable clock
	RCC->AHB1ENR |= RCC_AHB1Periph_GPIOADC ;	// Enable ports A&C clocks (and B for REVPLUS)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN ;		// Enable DMA2 clock
	configure_pins( PIN_FLAP3 | PIN_FLAP4 | PIN_FLAP5, PIN_ANALOG | PIN_PORTF ) ;

	ADC3->CR1 = ADC_CR1_SCAN ;
	ADC3->CR2 = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS ;
	ADC3->SQR1 = (NUM_EXTRA_ANALOG-1) << 20 ;		// NUMBER_ANALOG Channels
	ADC3->SQR3 = FLAP_3 + (FLAP_4<<5) + (FLAP_5<<10) ;
	ADC3->SMPR1 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12)
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) ;
	ADC3->SMPR2 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12) 
								+ (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24) + (SAMPTIME<<27) ;
	ADC->CCR = 0 ; //ADC_CCR_ADCPRE_0 ;		// Clock div 2
	
  // Enable the DMA channel here, DMA2 stream 1, channel 2
	DMA2_Stream1->CR = DMA_SxCR_PL | DMA_SxCR_CHSEL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC ;
	DMA2_Stream1->PAR = (uint32_t) &ADC3->DR ;
	DMA2_Stream1->M0AR = (uint32_t) &Analog_values[NUMBER_ANALOG] ;
	DMA2_Stream1->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 ;
}
#endif	// REV9E
#endif // nPCBSP

#ifndef PCBSP
uint16_t RotaryAnalogValue ;
uint16_t REDebug1 ;

extern "C" void ADC_IRQHandler()
{
	uint32_t x ;
	x = ADC2->DR ;
	REDebug1 = x ;
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
#endif // nPCBSP

// TODO
void stop_adc()
{
}	

#ifdef PCBSP

uint16_t AnalogSwitches ;
static uint16_t oldAnalog[3] ;

void processAnalogSwitches()
{
	uint32_t i ;
	for ( i = 0 ; i < 3 ; i += 1 )
	{
		uint32_t x ;
		x = Analog_values[i+5] ;
		int32_t diff = x - oldAnalog[i] ;
		if ( diff < 0 )
		{
			diff = -diff ;
		}
		oldAnalog[i] = x ;
		if ( diff < 20 )
		{
			uint32_t y ;
			if ( x < 0x56F )
			{
				if ( x < 0x465 )
				{
					y = ( x < 0x428 ) ? 4 : 0 ;
				}
				else
				{
					y = ( x < 0x4FB ) ? 6 : 2 ;
				}
			}
			else
			{
				if ( x < 0x68B )
				{
					y = ( x < 0x5CE ) ? 5 : 1 ;
				}
				else
				{
					y = ( x < 0x780 ) ? 7 : 3 ;
				}
			}
			y <<= (i*3) ;
			uint16_t z = AnalogSwitches & ~(7 << (i*3)) ;
			AnalogSwitches = z | y ;
		}
	}
}

#endif // PCBSP

