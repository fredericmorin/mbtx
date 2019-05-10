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

#include "X9D/stm32f2xx.h"
#include "X9D/hal.h"
#include "debug.h"

#include "core_cm3.h"

#include "ersky9x.h"
#include "timers.h"
#include "logicio.h"
#include "myeeprom.h"
#include "drivers.h"
#include "pulses.h"

extern int16_t g_chans512[] ;

#define INTERNAL_RF_ON()      GPIO_SetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define INTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR)


extern void init_pxx(uint32_t port) ;
extern void disable_pxx(uint32_t port) ;

void setupPulsesPpmAll(uint32_t module) ;

//#define BindBit 0x80

// To Do
#define NUM_MODULES 2

extern void init_ppm(uint32_t port) ;
extern void disable_ppm(uint32_t port) ;

#ifndef PCBSKY

//uint16_t *PulsePtr ;
uint16_t *TrainerPulsePtr ;
uint16_t PcmCrc ;
uint8_t PcmOnesCount ;
//uint8_t Current_protocol ;
uint8_t BindRangeFlag[2] = { 0, 0 } ;
//uint8_t PxxExtra[2] = { 0, 0 } ;

volatile uint8_t Dsm_Type[2] = { 0, 0 } ;
uint8_t DsmInitCounter[2] = { 0, 0 } ;

extern uint16_t *ppmStreamPtr[NUM_MODULES];
extern uint16_t pulseStreamCount[NUM_MODULES] ;
extern uint16_t ppmStream[NUM_MODULES+1][20];
extern uint8_t s_current_protocol[NUM_MODULES] ;

uint8_t SerialData[2][28] ;

static uint8_t Pass[2] ;
#endif

// TC0 - hardware timer
// TC1 - DAC clock
// TC2 - 5mS timer
// TC3 - input capture
// TC4 - input capture clock
// TC5 - Software COM1

uint8_t PulsesPaused ;

#ifdef XFIRE
uint8_t Bit_pulses[80] ;			// To allow for Xfire telemetry
#endif

// DSM2 control bits
//#define BindBit 0x80
//#define RangeCheckBit 0x20
#define FranceBit 0x10
#define DsmxBit  0x08


// States in LineState
#define LINE_IDLE			0
#define LINE_ACTIVE		1

#define BIT_TIME_100K		20

uint16_t LastTransition ;
//extern uint16_t BitTime ;
//extern uint16_t HtoLtime ;
//extern uint16_t LtoHtime ;
//extern uint8_t LineState ;
extern void putCaptureTime( struct t_softSerial *pss, uint16_t time, uint32_t value ) ;
//extern uint8_t SoftSerInvert ;
//extern uint8_t SoftSerialEvenParity ;
extern void start_timer11(void) ;
extern void stop_timer11(void) ;
extern uint8_t TrainerPolarity ;

uint8_t setupPulsesXfire() ;

uint16_t FailsafeCounter[2] ;

void pausePulses()
{
	PulsesPaused = 1 ;
}

void resumePulses()
{
	PulsesPaused = 0 ;
	if ( g_model.Module[1].protocol != PROTO_OFF )
	{
		EXTERNAL_RF_ON() ;
	}
	if ( g_model.Module[0].protocol != PROTO_OFF )
	{
		INTERNAL_RF_ON() ;
	}
}

void startPulses()
{
//	s_current_protocol[0] = g_model.protocol + 1 ;		// Not the same!
//	s_current_protocol[1] = g_model.xprotocol + 1 ;		// Not the same!
	setupPulses(0) ;// For DSM-9XR this won't be sent
	setupPulses(1) ;// For DSM-9XR this won't be sent
	Pass[0] = 0 ;		// Force a type 0 packet
	Pass[1] = 0 ;		// Force a type 0 packet
}


void init_hw_timer()
{
	// Timer13
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN ;		// Enable clock
	TIM13->ARR = 65535 ;
	TIM13->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 10000000 - 1 ;		// 0.1uS from 30MHz
	TIM13->CCER = 0 ;
	TIM13->CCMR1 = 0 ;
	TIM13->EGR = 0 ;
	TIM13->CR1 = 1 ;
}


// delay in units of 0.1 uS up to 6.5535 mS
void hw_delay( uint16_t time )
{
	TIM13->CNT = 0 ;
	TIM13->EGR = 1 ;		// Re-start counter
	while ( TIM13->CNT < time )
	{
		// wait
	}
}

// Starts TIMER at 200Hz, 5mS period
uint8_t Dsm_mode_response = 0 ;
void init5msTimer()
{
	// Timer14
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN ;		// Enable clock
	TIM14->ARR = 4999 ;	// 5mS
	TIM14->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 1000000 - 1 ;		// 1uS from 30MHz
	TIM14->CCER = 0 ;
	TIM14->CCMR1 = 0 ;
	TIM14->EGR = 0 ;
	TIM14->CR1 = 5 ;
	TIM14->DIER |= 1 ;
  NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 4 ) ;
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn) ;
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
	TIM14->SR = TIMER9_14SR_MASK & ~TIM_SR_UIF ;
	interrupt5ms() ;
}

void setupPulses(unsigned int port)
{
	uint32_t requiredprotocol ;
  heartbeat |= HEART_TIMER_PULSES ;

	if ( port == 0 )
	{
		requiredprotocol = g_model.Module[INTERNAL_MODULE].protocol ;
		if ( PulsesPaused )
		{
			requiredprotocol = PROTO_OFF ;
		}
  	if ( s_current_protocol[INTERNAL_MODULE] != requiredprotocol )
  	{
  	  switch( s_current_protocol[INTERNAL_MODULE] )
  	  {	// stop existing protocol hardware
  	    case PROTO_PPM:
					disable_main_ppm() ;
  	    break;
  	    case PROTO_PXX:
					disable_pxx(INTERNAL_MODULE) ;
  	    break;
  	  }

  	  s_current_protocol[INTERNAL_MODULE] = requiredprotocol ;
  	  switch(s_current_protocol[INTERNAL_MODULE])
  	  { // Start new protocol hardware here
  	    case PROTO_PPM:
					init_main_ppm() ;
  	    break;
  	    case PROTO_PXX:
					init_pxx(INTERNAL_MODULE) ;
  	    break;
		    case PROTO_OFF:
				default :
					init_no_pulses( INTERNAL_MODULE ) ;
  	  	break;
  	  }
  	}

	// Set up output data here
		switch(requiredprotocol)
  	{
		  case PROTO_PPM:
  	    setupPulsesPpmAll(INTERNAL_MODULE);		// Don't enable interrupts through here
  	  break;
			case PROTO_PXX:
  	    setupPulsesPXX(INTERNAL_MODULE);
  	  break;
  	}
	}
	else
	{
		requiredprotocol = g_model.Module[1].protocol;
		if (PulsesPaused)
		{
			requiredprotocol = PROTO_OFF;
		}
		if (s_current_protocol[1] != requiredprotocol)
		{
			switch (s_current_protocol[1])
			{ // stop existing protocol hardware
			case PROTO_PPM:
				disable_ppm(EXTERNAL_MODULE);
				break;
			case PROTO_PXX:
				disable_pxx(EXTERNAL_MODULE);
				break;
			case PROTO_DSM2:
			case PROTO_MULTI:
				disable_dsm2(EXTERNAL_MODULE);
				break;
			case PROTO_XFIRE:
				disable_xfire(EXTERNAL_MODULE);
				break;
			}

			s_current_protocol[EXTERNAL_MODULE] = requiredprotocol;
			switch (s_current_protocol[EXTERNAL_MODULE])
			{ // Start new protocol hardware here
			case PROTO_PPM:
				setupPulsesPpmAll(EXTERNAL_MODULE);
				init_ppm(EXTERNAL_MODULE);
				break;
			case PROTO_PXX:
				init_pxx(EXTERNAL_MODULE);
				break;
			case PROTO_OFF:
			default:
				init_no_pulses(EXTERNAL_MODULE);
				break;
			case PROTO_MULTI:
				init_multi(EXTERNAL_MODULE);
				DsmInitCounter[port] = 0;
				Pass[port] = 0;
				break;
			case PROTO_DSM2:
				init_dsm2(EXTERNAL_MODULE);
				DsmInitCounter[port] = 0;
				Pass[port] = 0;
				break;
			case PROTO_XFIRE:
				init_xfire(EXTERNAL_MODULE);
				break;
			}
		}

		// Set up output data here
		switch (requiredprotocol)
		{
		case PROTO_PPM:
			setupPulsesPpmAll(EXTERNAL_MODULE); // Don't enable interrupts through here
			break;
		case PROTO_PXX:
			setupPulsesPXX(EXTERNAL_MODULE);
			break;
		case PROTO_DSM2:
		case PROTO_MULTI:
			setupPulsesDsm2((g_model.Module[EXTERNAL_MODULE].sub_protocol == DSM_9XR) ? 12 : 6, EXTERNAL_MODULE);
			break;
		case PROTO_XFIRE:
			setupPulsesXfire();
			break;
		}
	}
}

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)

uint16_t TrainerPpmStream[20] =
{
	2000,
	2200,
	2400,
	2600,
	2800,
	3000,
	3200,
	3400,
	45000-21600,
	0,0,0,0,0,0,0,0,0,0,0
} ;



#define PPM_CENTER 1500*2


// PPM output
// Timer 1, channel 1 on PA8 for prototype
// Pin is AF1 function for timer 1
void init_main_ppm()
{
  setupPulses(0) ;
	init_ppm(0) ;

}

void disable_main_ppm()
{
	disable_ppm(0) ;
}

// Trainer PPM output PC9, Timer 3 channel 4, (Alternate Function 2)
// Horus:
// Trainer PPM output PC7, Timer 3 channel 2, (Alternate Function 2)
// Use channel 3 not 1 for main timing
// X3:
// Trainer PPM output PD12, Timer 4 channel 1
// Use channel 3 not 1 for main timing
void setupTrainerPulses()
{
  uint32_t i ;
	uint32_t total ;
	uint32_t pulse ;
	uint16_t *ptr ;
	uint32_t p = (g_model.ppmNCH + 4) ;
	if ( p > 16 )
	{
		p = 8 ;
	}
	int16_t PPM_range = g_model.extendedLimits ? 640*2 : 512*2;   //range of 0.7..1.7msec

  if(g_model.trainPulsePol)
	{
#if defined(PCBX12D) || defined(PCBX10)
		TIM3->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P ;
	}
	else
	{
		TIM3->CCER = TIM_CCER_CC2E ;
#else
 #ifdef PCBX9LITE
		TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P ;
	}
	else
	{
		TIM4->CCER = TIM_CCER_CC1E ;
 #else
		TIM3->CCER = TIM_CCER_CC4E | TIM_CCER_CC4P ;
	}
	else
	{
		TIM3->CCER = TIM_CCER_CC4E ;
 #endif
#endif
	}
  ptr = TrainerPpmStream ;

	total = 22500u*2; //Minimum Framelen=22.5 ms
  total += (int16_t(g_model.ppmFrameLength))*1000;

	p += g_model.startChannel ;
	for ( i = g_model.startChannel ; i < p ; i += 1 )
	{
  	pulse = max( (int)min(g_chans512[i],PPM_range),-PPM_range) + PPM_CENTER;

		total -= pulse ;
		*ptr++ = pulse ;
	}
	*ptr++ = total ;
	*ptr = 0 ;
#if defined(PCBX12D) || defined(PCBX10)
	TIM3->CCR3 = total - 1000 ;		// Update time
	TIM3->CCR2 = (g_model.ppmDelay*50+300)*2 ;
#else
 #ifdef PCBX9LITE
	TIM4->CCR3 = total - 1000 ;		// Update time
	TIM4->CCR1 = (g_model.ppmDelay*50+300)*2 ;
 #else
	TIM3->CCR1 = total - 1000 ;		// Update time
	TIM3->CCR4 = (g_model.ppmDelay*50+300)*2 ;
 #endif
#endif
}


// Trainer PPM output PC9, Timer 3 channel 4, (Alternate Function 2)
// Horus:
// Trainer PPM output PC7, Timer 3 channel 2, (Alternate Function 2)
// Use channel 3 not 1 for main timing
// X3:
// Trainer PPM output PD12, Timer 4 channel 1

void init_trainer_ppm()
{
	setupTrainerPulses() ;
	TrainerPulsePtr = TrainerPpmStream ;

#ifdef PCBX9LITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portC clock
	configure_pins( PIN_TR_PPM_OUT, PIN_PERIPHERAL | PIN_PORTD| PIN_PER_2 | PIN_OS25 | PIN_PUSHPULL ) ;
	configure_pins( PIN_TRNDET, PIN_INPUT | PIN_PULLUP | PIN_PORTD) ;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;            // Enable clock

  TIM4->ARR = *TrainerPulsePtr++ ;
  TIM4->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;               // 0.5uS

  if(g_model.trainPulsePol)
	{
		TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P ;
	}
	else
	{
		TIM4->CCER = TIM_CCER_CC1E ;
	}

  TIM4->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE ;		// PWM mode 1
	TIM4->CCR1 = 600 ;		// 300 uS pulse
	TIM4->BDTR = TIM_BDTR_MOE ;
 	TIM4->EGR = 1 ;

	TIM4->SR = TIMER2_5SR_MASK & ~TIM_SR_UIF ;				// Clear flag
	TIM4->SR = TIMER2_5SR_MASK & ~TIM_SR_CC3IF ;				// Clear flag
	TIM4->DIER |= TIM_DIER_CC1IE ;
	TIM4->DIER |= TIM_DIER_UIE ;

	TIM4->CR1 = TIM_CR1_CEN ;
  NVIC_SetPriority(TIM4_IRQn, 7);
	NVIC_EnableIRQ(TIM4_IRQn) ;
#else // X3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	configure_pins( PIN_TR_PPM_OUT, PIN_PERIPHERAL | PIN_PORTC | PIN_PER_2 | PIN_OS25 | PIN_PUSHPULL ) ;
	configure_pins( PIN_TRNDET, PIN_INPUT | PIN_PULLUP | PIN_PORTB ) ;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN ;            // Enable clock

  TIM3->ARR = *TrainerPulsePtr++ ;
  TIM3->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;               // 0.5uS

  if(g_model.trainPulsePol)
	{
#if defined(PCBX12D) || defined(PCBX10)
		TIM3->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P ;
	}
	else
	{
		TIM3->CCER = TIM_CCER_CC2E ;
#else
		TIM3->CCER = TIM_CCER_CC4E | TIM_CCER_CC4P ;
	}
	else
	{
		TIM3->CCER = TIM_CCER_CC4E ;
#endif
	}

#if defined(PCBX12D) || defined(PCBX10)
  TIM3->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE ;		// PWM mode 1
	TIM3->CCR2 = 600 ;		// 300 uS pulse
#else
  TIM3->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE ;		// PWM mode 1
	TIM3->CCR4 = 600 ;		// 300 uS pulse
#endif
	TIM3->BDTR = TIM_BDTR_MOE ;
 	TIM3->EGR = 1 ;
//	TIM8->DIER = TIM_DIER_UDE ;

	TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_UIF ;				// Clear flag
#if defined(PCBX12D) || defined(PCBX10)
#else
	TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_CC1IF ;				// Clear flag
	TIM3->DIER |= TIM_DIER_CC1IE ;
#endif
	TIM3->DIER |= TIM_DIER_UIE ;

	TIM3->CR1 = TIM_CR1_CEN ;
  NVIC_SetPriority(TIM3_IRQn, 7);
	NVIC_EnableIRQ(TIM3_IRQn) ;
#endif // X3
}

// TODO - testing
void stop_trainer_ppm()
{
#ifdef PCBX9LITE
	configure_pins( PIN_TR_PPM_OUT, PIN_INPUT | PIN_PORTD ) ;
	TIM4->DIER = 0 ;
	TIM4->CR1 &= ~TIM_CR1_CEN ;				// Stop counter
	NVIC_DisableIRQ(TIM4_IRQn) ;				// Stop Interrupt
#else // X3
	configure_pins( PIN_TR_PPM_OUT, PIN_INPUT | PIN_PORTC ) ;
	TIM3->DIER = 0 ;
	TIM3->CR1 &= ~TIM_CR1_CEN ;				// Stop counter
	NVIC_DisableIRQ(TIM3_IRQn) ;				// Stop Interrupt
#endif // X3
}

// Trainer capture, PC8, Timer 3 channel 3
// Soft Serial capture, PC8, Timer 3 channel 3
// Horus:
// Trainer capture, PC6, Timer 3 channel 3
// Soft Serial capture, PC6, Timer 3 channel 1, (Alternate Function 2)
// X3
// Trainer capture, PD13, Timer 4 channel 2
// Soft Serial capture, PD13, Timer 4 channel 2

void init_trainer_capture(uint32_t mode)
{
	struct t_softSerial *pss = &SoftSerial1 ;

	if ( CaptureMode != CAP_COM1 )
	{
		CaptureMode = mode ;
	}

	stop_trainer_ppm() ;
#ifdef PCBX9LITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	configure_pins( PIN_TR_PPM_IN, PIN_PERIPHERAL | PIN_PORTD | PIN_PER_2 | PIN_PULLUP ) ;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;		// Enable clock

	TIM4->ARR = 0xFFFF ;
	TIM4->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM4->CR2 = 0 ;

#else
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	configure_pins( PIN_TR_PPM_IN, PIN_PERIPHERAL | PIN_PORTC | PIN_PER_2 | PIN_PULLUP ) ;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN ;		// Enable clock

	TIM3->ARR = 0xFFFF ;
	TIM3->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM3->CR2 = 0 ;

#endif

#ifndef PCBX12D
 #ifndef PCBX10
	if ( mode == CAP_SERIAL )
	{
		pss->lineState = LINE_IDLE ;
		pss->bitTime = BIT_TIME_100K ;
		pss->softSerialEvenParity = 1 ;
		pss->softSerInvert = TrainerPolarity ;
  #if defined(PCBX12D) || defined(PCBX10)
  #else
   #ifdef PCBX9LITE
		TIM4->CCMR1 = TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_1 ;
		TIM4->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP ;	// Both edges
		TIM4->SR = TIMER2_5SR_MASK & ~(TIM_SR_CC2IF) ;				// Clear flag
		TIM4->DIER |= TIM_DIER_CC2IE ;
   #else
		TIM3->CCMR2 = TIM_CCMR2_IC4F_0 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_CC4S_1 | TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_CC3S_0 ;
		TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC4P ;
		TIM3->SR = TIMER2_5SR_MASK & ~(TIM_SR_CC4IF | TIM_SR_CC3IF) ;				// Clear flags
		TIM3->DIER |= TIM_DIER_CC4IE | TIM_DIER_CC3IE ;
   #endif
  #endif

 #ifdef PCBX9LITE
		NVIC_SetPriority(TIM4_IRQn, 0 ) ;
 #else
		NVIC_SetPriority(TIM3_IRQn, 0 ) ;
 #endif
		start_timer11() ;
	}
	else
 #endif
#endif
	{
#if defined(PCBX12D) || defined(PCBX10)
		TIM3->CCMR1 = TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_CC1S_0 ;
		TIM3->CCER = TIM_CCER_CC1E ;
		TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_CC1IF ;				// Clear flag
		TIM3->DIER |= TIM_DIER_CC1IE ;
#else
 #ifdef PCBX9LITE
		TIM4->CCMR1 = TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0 ;
		TIM4->CCER = TIM_CCER_CC2E ;
		TIM4->SR = TIMER2_5SR_MASK & ~TIM_SR_CC2IF ;				// Clear flag
		TIM4->DIER |= TIM_DIER_CC2IE ;
 #else
		TIM3->CCMR2 = TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_CC3S_0 ;
		TIM3->CCER = TIM_CCER_CC3E ;
		TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_CC3IF ;				// Clear flag
		TIM3->DIER |= TIM_DIER_CC3IE ;
 #endif
#endif
#ifdef PCBX9LITE
		NVIC_SetPriority(TIM4_IRQn, 7 ) ;
#else
		NVIC_SetPriority(TIM3_IRQn, 7 ) ;
#endif
	}

#ifdef PCBX9LITE
	TIM4->CR1 = TIM_CR1_CEN ;
	NVIC_EnableIRQ(TIM4_IRQn) ;
#else
	TIM3->CR1 = TIM_CR1_CEN ;
	NVIC_EnableIRQ(TIM3_IRQn) ;
#endif
}

void stop_trainer_capture()
{
#ifdef PCBX9LITE
	TIM4->DIER = 0 ;
	TIM4->CR1 &= ~TIM_CR1_CEN ;				// Stop counter
	NVIC_DisableIRQ(TIM4_IRQn) ;				// Stop Interrupt
#else // X3
	TIM3->DIER = 0 ;
	TIM3->CR1 &= ~TIM_CR1_CEN ;				// Stop counter
	NVIC_DisableIRQ(TIM3_IRQn) ;				// Stop Interrupt
#endif // X3
	if ( CaptureMode == CAP_SERIAL )
	{
		stop_timer11() ;
		CaptureMode = 0 ;
	}
}

#ifndef PCBX12D
 #ifndef PCBX10
void init_serial_trainer_capture()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
	configure_pins( 0x0800, PIN_PERIPHERAL | PIN_PORTB | PIN_PER_1 ) ;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN ;		// Enable clock

	TIM2->ARR = 0xFFFF ;
	TIM2->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM2->CR2 = 0 ;
	TIM2->CCMR2 = TIM_CCMR2_IC4F_0 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_CC4S_0 ;
	TIM2->CCER = TIM_CCER_CC4E ;
	TIM2->SR = TIMER2_5SR_MASK & ~TIM_SR_CC4IF ;				// Clear flag
	TIM2->DIER |= TIM_DIER_CC4IE ;
	TIM2->CR1 = TIM_CR1_CEN ;
  NVIC_SetPriority(TIM2_IRQn, 7);
	NVIC_EnableIRQ(TIM2_IRQn) ;
}

void stop_serial_trainer_capture()
{
	TIM2->DIER = 0 ;
	TIM2->CR1 &= ~TIM_CR1_CEN ;				// Stop counter
	NVIC_DisableIRQ(TIM2_IRQn) ;				// Stop Interrupt
}

// Capture interrupt for trainer input
extern "C" void TIM2_IRQHandler()
{

	uint16_t capture = 0 ;
  static uint16_t lastCapt ;
  uint16_t val ;
	uint32_t doCapture = 0 ;

  // What mode? in or out?
  if ( (TIM2->DIER & TIM_DIER_CC4IE ) && ( TIM2->SR & TIM_SR_CC4IF ) )
    // capture mode
	{
		capture = TIM2->CCR4 ;
		doCapture = 1 ;
	}

	TrainerProfile *tProf = &g_eeGeneral.trainerProfile[g_model.trainerProfile] ;
	if ( tProf->channel[0].source != TRAINER_JACK )
	{
		doCapture = 0 ;
	}

	if ( doCapture )
	{
  	val = (uint16_t)(capture - lastCapt) / 2 ;
  	lastCapt = capture;

  	// We prcoess g_ppmInsright here to make servo movement as smooth as possible
  	//    while under trainee control
  	if ((val>4000) && (val < 19000)) // G: Prioritize reset pulse. (Needed when less than 8 incoming pulses)
		{
  	  ppmInState = 1; // triggered
		}
  	else
  	{
  		if(ppmInState && (ppmInState<=16))
			{
  	  	if((val>800) && (val<2200))
				{
					ppmInValid = 100 ;
//  		    g_ppmIns[ppmInState++ - 1] = (int16_t)(val - 1500)*(g_eeGeneral.PPM_Multiplier+10)/10; //+-500 != 512, but close enough.
  		    g_ppmIns[ppmInState++ - 1] = (int16_t)(val - 1500) ; //+-500 != 512, but close enough.

		    }else{
  		    ppmInState=0; // not triggered
  	  	}
  	  }
  	}
	}
}


#if defined(PCBXLITE) || defined(PCBX9LITE)
//#ifdef PCBX9LITE
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#endif // X3

void init_cppm_on_heartbeat_capture()
{
#ifdef PCBX9D
 #ifndef PCBX9LITE
	stop_xjt_heartbeat() ;
 #endif // X3
#endif
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	configure_pins( 0x0080, PIN_PERIPHERAL | PIN_PORTC | PIN_PER_2 ) ;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN ;		// Enable clock

	TIM3->ARR = 0xFFFF ;
	TIM3->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM3->CR2 = 0 ;
	TIM3->CCMR1 = TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0 ;
	TIM3->CCER = TIM_CCER_CC2E ;
	TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_CC2IF ;				// Clear flag
	TIM3->DIER |= TIM_DIER_CC2IE ;
	TIM3->CR1 = TIM_CR1_CEN ;
  NVIC_SetPriority(TIM3_IRQn, 7);
	NVIC_EnableIRQ(TIM3_IRQn) ;

#ifdef PCBX9LITE
	EXTERNAL_RF_ON() ;
#endif // X3

}

void stop_cppm_on_heartbeat_capture()
{
#ifdef PCBX9LITE
	EXTERNAL_RF_OFF() ;
#endif // X3
	TIM3->DIER = 0 ;
	TIM3->CR1 &= ~TIM_CR1_CEN ;				// Stop counter
	TIM3->DIER &= ~TIM_DIER_CC2IE ;		// Disable interrupt
	NVIC_DisableIRQ(TIM3_IRQn) ;			// Stop Interrupt
#ifdef PCBX9D
 #ifndef PCBX9LITE
	init_xjt_heartbeat() ;
 #endif // X3
#endif

}
 #endif
#endif

#ifdef PCBX9LITE
extern "C" void TIM4_IRQHandler()
#else // X3
extern "C" void TIM3_IRQHandler()
#endif // X3
{

	uint16_t capture = 0 ;
  static uint16_t lastCapt ;
  uint16_t val ;
	uint32_t doCapture = 0 ;
	struct t_softSerial *pss = &SoftSerial1 ;
	if ( CaptureMode == CAP_SERIAL )
	{
//		TIM3Captures += 1 ;
		// CC3 physical rising, CC4 falling edge
		// So for inverted serial CC3 is HtoL, CC4 is LtoH
		if ( LastTransition == 0 )	// LtoH
		{
#ifdef PCBX9LITE
			capture = TIM4->CCR2 ;
#else // X3
			capture = TIM3->CCR4 ;
#endif // X3
			LastTransition = 1 ;
		}
		else
		{
#ifdef PCBX9LITE
			capture = TIM4->CCR2 ;
#else // X3
			capture = TIM3->CCR4 ;
#endif // X3
			LastTransition = 0 ;
		}

  	val = LastTransition ;
		if ( pss->softSerInvert )
		{
			val = !val ;
		}

		if ( val == 0 )	// LtoH
		{
			// L to H transition
			pss->LtoHtime = capture ;
			TIM11->CNT = 0 ;
			TIM11->CCR1 = pss->bitTime * 12 ;
			uint32_t time ;
			capture -= pss->HtoLtime ;
			time = capture ;
//			putCapValues( time, 0 ) ;
			putCaptureTime( pss, time, 0 ) ;
			TIM11->DIER = TIM_DIER_CC1IE ;
		}
		else
		{
			// H to L transition
			pss->HtoLtime = capture ;
			if ( pss->lineState == LINE_IDLE )
			{
				pss->lineState = LINE_ACTIVE ;
//				if ( ++CapCount > 200 )
//				{
//					CapIndex = 0 ;
//					CapCount = 0 ;
//				}
//				putCapValues( 0, 3 ) ;
				putCaptureTime( pss, 0, 3 ) ;
			}
			else
			{
				uint32_t time ;
				capture -= pss->LtoHtime ;
				time = capture ;
//				putCapValues( time, 1 ) ;
				putCaptureTime( pss, time, 1 ) ;
			}
			TIM11->DIER = 0 ;
			TIM11->CCR1 = pss->bitTime * 20 ;
			TIM11->CNT = 0 ;
			TIM11->SR = 0 ;
		}
		return ;
	}
	else
	{
  	// What mode? in or out?
#if defined(PCBX12D) || defined(PCBX10)
  	if ( (TIM3->DIER & TIM_DIER_CC1IE ) && ( TIM3->SR & TIM_SR_CC1IF ) )
  	  // capture mode
		{
			capture = TIM3->CCR1 ;
			doCapture = 1 ;
		}
#else

 #ifdef PCBX9LITE
		if ( (TIM4->DIER & TIM_DIER_CC2IE ) && ( TIM4->SR & TIM_SR_CC2IF ) )
  	  // capture mode
		{
			capture = TIM4->CCR2 ;
			doCapture = 1 ;
		}
 #else // X3
		if ( (TIM3->DIER & TIM_DIER_CC3IE ) && ( TIM3->SR & TIM_SR_CC3IF ) )
  	  // capture mode
		{
			capture = TIM3->CCR3 ;
			doCapture = 1 ;
		}
 #endif // X3
#endif

#ifndef PCBX12D
#ifndef PCBX10
 #ifdef PCBX9LITE
  	if ( (TIM4->DIER & TIM_DIER_CC1IE ) && ( TIM3->SR & TIM_SR_CC1IF ) )
  	  // capture mode
		{
			capture = TIM4->CCR1 ;
			doCapture = 1 ;
		}
 #else
  	if ( (TIM3->DIER & TIM_DIER_CC2IE ) && ( TIM3->SR & TIM_SR_CC2IF ) )
  	  // capture mode
		{
			capture = TIM3->CCR2 ;
			doCapture = 1 ;
		}
 #endif // X3
#endif
#endif

		if ( doCapture )
		{
  		val = (uint16_t)(capture - lastCapt) / 2 ;
  		lastCapt = capture;

  		// We prcoess g_ppmInsright here to make servo movement as smooth as possible
  		//    while under trainee control
  		if ((val>4000) && (val < 19000)) // G: Prioritize reset pulse. (Needed when less than 8 incoming pulses)
			{
  		  ppmInState = 1; // triggered
			}
  		else
  		{
  			if(ppmInState && (ppmInState<=16))
				{
  		  	if((val>800) && (val<2200))
					{
						ppmInValid = 100 ;
	//  		    g_ppmIns[ppmInState++ - 1] = (int16_t)(val - 1500)*(g_eeGeneral.PPM_Multiplier+10)/10; //+-500 != 512, but close enough.
  			    g_ppmIns[ppmInState++ - 1] = (int16_t)(val - 1500) ; //+-500 != 512, but close enough.

			    }else{
  			    ppmInState=0; // not triggered
  		  	}
  		  }
  		}
		}

#ifdef PCBX9LITE
#else
  	// PPM out compare interrupt
  	if ( ( TIM3->DIER & TIM_DIER_CC1IE ) && ( TIM3->SR & TIM_SR_CC1IF ) )
		{
  	  // compare interrupt
  	  TIM3->DIER &= ~TIM_DIER_CC1IE ;         // stop this interrupt
  	  TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_CC1IF ;                             // Clear flag

  	  setupTrainerPulses() ;

			TrainerPulsePtr = TrainerPpmStream ;
  	  TIM3->DIER |= TIM_DIER_UDE ;
  	  TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_UIF ;                                       // Clear this flag
  	  TIM3->DIER |= TIM_DIER_UIE ;                            // Enable this interrupt
  	}

  	// PPM out update interrupt
  	if ( (TIM3->DIER & TIM_DIER_UIE) && ( TIM3->SR & TIM_SR_UIF ) )
		{
  	  TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_UIF ;                               // Clear flag
  	  TIM3->ARR = *TrainerPulsePtr++ ;
  	  if ( *TrainerPulsePtr == 0 )
			{
  	    TIM3->SR = TIMER2_5SR_MASK & ~TIM_SR_CC1IF ;                     // Clear this flag
  	    TIM3->DIER |= TIM_DIER_CC1IE ;  // Enable this interrupt
  	  }
  	}
	}
#endif
}

#endif


#ifndef PCBSKY

#define PPM_CENTER 1500*2

void setupPulsesPpmAll(uint32_t module)
{
  uint32_t i ;
	int32_t total ;
	uint16_t *ptr ;
	uint32_t p = (g_model.Module[module].channels + 8) ;
//	if ( p > 16 )
//	{
//		p -= 13 ;
//	}
  p += g_model.Module[module].startChannel ; //Channels *2
	if ( p > NUM_SKYCHNOUT+EXTRA_SKYCHANNELS )
	{
		p = NUM_SKYCHNOUT+EXTRA_SKYCHANNELS ;	// Don't run off the end
	}

	// Now set up pulses
	int16_t PPM_range = g_model.extendedLimits ? 640*2 : 512*2;   //range of 0.7..1.7msec

  //Total frame length = 22.5msec
  //each pulse is 0.7..1.7ms long with a 0.3ms stop tail
  //The pulse ISR is 2mhz that's why everything is multiplied by 2
  ptr = ppmStream[module] ;


	total = 22500u*2; //Minimum Framelen=22.5 ms
  total += (int16_t(g_model.Module[module].ppmFrameLength))*1000;

	for ( i = g_model.Module[module].startChannel ; i < p ; i += 1 )
	{ //NUM_SKYCHNOUT+EXTRA_SKYCHANNELS
  	int16_t v = max( (int)min(g_chans512[i],PPM_range),-PPM_range) + PPM_CENTER;
		total -= v ;
    *ptr++ = v ; /* as Pat MacKenzie suggests */
	}
	if ( total<9000 )
	{
		total = 9000 ;
	}
	*ptr++ = total ;
	*ptr = 0 ;

	TIM_TypeDef *pTimer = TIM1 ;
	uint16_t polarityBit = TIM_CCER_CC3P ;
	if ( module )
	{
		pTimer = TIM8 ;
		polarityBit = TIM_CCER_CC1NP ;
	}
	pTimer->CCR2 = total - 1000 ;		// Update time
	pTimer->CCR3 = (g_model.Module[module].ppmDelay*50+300)*2 ;
  if(!g_model.Module[module].pulsePol)
    TIM1->CCER |= polarityBit;
	else
	{
	  TIM1->CCER &= ~polarityBit ;
	}
}

extern uint16_t dsm2Stream[][400] ;
uint16_t *dsm2StreamPtr[2] ;
uint16_t dsm2Value[2] ;
uint8_t dsm2Index[2] = {0,0} ;

uint8_t BITLEN_Serial = (8*2) ;

void _send_1(uint8_t v, uint32_t module )
{
 	if (dsm2Index[module] == 0)
 	  v -= 2;
 	else
 	  v += 2;

  dsm2Value[module] += v;
  *dsm2StreamPtr[module]++ = dsm2Value[module];

  dsm2Index[module] = (dsm2Index[module]+1) % 2;
}


uint32_t DsmPassIndex ;

void sendByteDsm2(uint8_t b, uint32_t module) //max 10 changes 0 10 10 10 10 1
{
	uint32_t protocol = g_model.Module[1].protocol ;
	uint8_t parity = 0x80 ;
	uint8_t count = 8 ;
	uint8_t bitLen ;
	if ( protocol != PROTO_DSM2 )
	{ // SBUS & MULTI
		parity = 0 ;
		bitLen = b ;
		for( uint8_t i=0; i<8; i++)
		{
			parity += bitLen & 0x80 ;
			bitLen <<= 1 ;
		}
		parity &= 0x80 ;
		count = 9 ;
	}
    bool lev = 0;
    uint8_t len = BITLEN_Serial; //max val: 9*16 < 256
    for (uint8_t i=0; i<=count; i++)
		{ //8Bits + Stop=1
        bool nlev = b & 1; //lsb first
        if (lev == nlev)
				{
          len += BITLEN_Serial;
        }
        else
				{
          _send_1(len, module); // _send_1(nlev ? len-5 : len+3);
          len = BITLEN_Serial;
          lev = nlev;
        }
				b = (b>>1) | parity ; //shift in parity or stop bit
				parity = 0x80 ;				// Now a stop bit
    }
    _send_1(len+BITLEN_Serial, module); // _send_1(len+BITLEN_DSM2+3); // 2 stop bits
}

void putDsm2Flush(uint32_t module)
{
  dsm2StreamPtr[module]--; //remove last stopbits and
  *dsm2StreamPtr[module]++ = 45000 ; // Past the 44000 of the ARR
}

static uint8_t dsmDat[2][2+6*2]={{0xFF,0x00, 0x00,0xAA, 0x05,0xFF, 0x09,0xFF, 0x0D,0xFF, 0x13,0x54, 0x14,0xAA},
																 {0xFF,0x00, 0x00,0xAA, 0x05,0xFF, 0x09,0xFF, 0x0D,0xFF, 0x13,0x54, 0x14,0xAA}};


void setupPulsesDsm2(uint8_t channels, uint32_t module )
{
  uint16_t required_baudrate ;

	uint32_t protocol = g_model.Module[module].protocol ;
	uint32_t sub_protocol = g_model.Module[module].sub_protocol ;

	required_baudrate = SCC_BAUD_125000 ;

	if( (protocol == PROTO_DSM2) && ( sub_protocol == DSM_9XR ) )
	{
		required_baudrate = SCC_BAUD_115200 ;
		Dsm_Type[module] = 1 ;
		// Consider inverting COM1 here
	}
	else if(protocol == PROTO_MULTI)
	{
		required_baudrate = SCC_BAUD_100000 ;
		Dsm_Type[module] = 0 ;
	}
	else
	{
		Dsm_Type[module] = 0 ;
	}

	if ( required_baudrate != Scc_baudrate )
	{
		if ( required_baudrate == SCC_BAUD_125000 )
		{
			BITLEN_Serial = (8*2) ;
		}
		else if ( required_baudrate == SCC_BAUD_100000 )
		{
			BITLEN_Serial = (10*2) ;
		}
		else
		{
			BITLEN_Serial = 17 ;
		}
		Scc_baudrate = required_baudrate ;
	}

	if ( Dsm_Type[module] )
	{
		uint8_t channels = 12 ;
		uint8_t flags = g_model.dsmMode ;
		if ( flags == 0 )
		{
			if ( Dsm_Type[module] == 1 )
			{
				flags = ORTX_AUTO_MODE ;
			}
		}
		else
		{
			flags &= 0x7F ;
			channels = g_model.Module[module].channels ;
		}

		if ( (dsmDat[module][0]&BindBit) && (!keyState(SW_SH2) ) )
		{
			dsmDat[module][0] &= ~BindBit	;
		}

		uint8_t startChan = g_model.Module[module].startChannel ;

 		dsm2StreamPtr[module] = dsm2Stream[module] ;
  	dsm2Index[module] = 0 ;
  	dsm2Value[module] = 100 ;
  	*dsm2StreamPtr[module]++ = dsm2Value[module] ;
		sendByteDsm2( 0xAA, module );
		if ( Pass[module] == 0 )
		{
  		sendByteDsm2( Pass[module], module ) ;		// Actually is a 0
			// Do init packet
			if ( (BindRangeFlag[module] & PXX_BIND) || (dsmDat[module][0]&BindBit) )
			{
				flags |= ORTX_BIND_FLAG ;
			}
			// Need to choose dsmx/dsm2 as well
  		sendByteDsm2( flags, module ) ;
  		sendByteDsm2( (BindRangeFlag[module] & PXX_RANGE_CHECK) ? 4: 7, module ) ;		//
  		sendByteDsm2( channels, module ) ;			// Max channels
			sendByteDsm2( g_model.Module[module].pxxRxNum, module ) ;	// Model number
	  	putDsm2Flush(module);
			Pass[module] = 1 ;
		}
		else
		{
			DsmPassIndex = 0 ;
			if ( Pass[module] == 2 )
			{
				startChan += 7 ;
			}
  		sendByteDsm2( Pass[module], module );

			for(uint8_t i=0 ; i<7 ; i += 1 )
			{
				uint16_t pulse ;
				int16_t value = g_chans512[startChan] ;
				if ( ( flags & ORTX_USE_11bit ) == 0 )
				{
					pulse = limit(0, ((value*13)>>5)+512,1023) | (startChan << 10) ;
				}
				else
				{
					pulse = limit(0, ((value*349)>>9)+1024,2047) | (startChan << 11) ;
				}
				startChan += 1 ;
				if ( startChan <= channels )
				{
  		  	sendByteDsm2( pulse >> 8, module ) ;
    			sendByteDsm2( pulse & 0xff, module ) ;
				}
				else
				{
    			sendByteDsm2( 0xff, module ) ;
    			sendByteDsm2( 0xff, module ) ;
				}
			}
	  	putDsm2Flush(module);
			if ( ++Pass[module] > 2 )
			{
				Pass[module] = 1 ;
			}
			if ( channels < 8 )
			{
				Pass[module] = 1 ;
			}
			DsmInitCounter[module] += 1 ;
			if( DsmInitCounter[module] > 100)
			{
				DsmInitCounter[module] = 0 ;
				Pass[module] = 0 ;
			}
			if (dsmDat[module][0]&BindBit)
			{
				Pass[module] = 0 ;		// Stay here
			}
		}
	}
	else
	{
  	dsm2Index[module] = 0 ;
 		dsm2StreamPtr[module] = dsm2Stream[module] ;
  	dsm2Value[module] = 100;
  	*dsm2StreamPtr[module]++ = dsm2Value[module];
		if(protocol == PROTO_DSM2)
		{
			setDsmHeader( dsmDat[module], module ) ;
		}
//		else // Multi
//		{
//			dsmDat[module][0] = sub_protocol+1;
//			if (BindRangeFlag[module] & PXX_BIND)	dsmDat[module][0] |=BindBit;		//set bind bit if bind menu is pressed
//			if (BindRangeFlag[module] & PXX_RANGE_CHECK)	dsmDat[module][0] |=RangeCheckBit;		//set bind bit if bind menu is pressed
//		}

		if ( protocol == PROTO_MULTI )
		{
			uint32_t i ;
			setMultiSerialArray( SerialData[module], module ) ;
			for ( i = 0 ; i < 26 ; i += 1 )
			{
				sendByteDsm2( SerialData[module][i], module) ;
			}
	  	putDsm2Flush(module);
		}
		else
		{
			dsmDat[module][1] = g_model.Module[module].pxxRxNum ;  //DSM2 Header second byte for model match
  		for( uint8_t i = 0 ; i<channels; i += 1 )
  		{
				uint16_t pulse = limit(0, ((g_chans512[g_model.Module[module].startChannel+i]*13)>>5)+512,1023);
 				dsmDat[module][2+2*i] = (i<<2) | ((pulse>>8)&0x03);
  			dsmDat[module][3+2*i] = pulse & 0xff;
  		}

	  	for (int i=0; i<14; i++)
			{
	  	  sendByteDsm2(dsmDat[module][i], module) ;
	  	}
	  	putDsm2Flush(module) ;
		}
	}
}


extern uint16_t pxxStream[2][400];
uint16_t *PtrPxx ;
uint16_t PxxValue ;
uint16_t *PtrPxx_x ;
uint16_t PxxValue_x ;

static void crc( uint8_t data )
{
  PcmCrc=(PcmCrc<<8) ^ CRCTable((PcmCrc>>8)^data) ;
}


inline __attribute__ ((always_inline)) void putPcmPart( uint8_t value )
{
	uint16_t lpxxValue = PxxValue += 18 ;

	*PtrPxx++ = lpxxValue ;
	lpxxValue += 14 ;
	if ( value )
	{
		lpxxValue += 16 ;
	}
	*PtrPxx++ = lpxxValue ;	// Output 0 for this time
	PxxValue = lpxxValue ;
}

void putPcmFlush()
{
	*PtrPxx++ = 19000 ;		// Past the 18000 of the ARR
}


void putPcmBit( uint8_t bit )
{
    if ( bit )
    {
        PcmOnesCount += 1 ;
        putPcmPart( 1 ) ;
    }
    else
    {
        PcmOnesCount = 0 ;
        putPcmPart( 0 ) ;
    }
    if ( PcmOnesCount >= 5 )
    {
        putPcmBit( 0 ) ;				// Stuff a 0 bit in
    }
}

void putPcmByte( uint8_t byte )
{
    crc( byte ) ;
    uint8_t i ;
    for ( i = 0 ; i < 8 ; i += 1 )
    {
        putPcmBit( byte & 0x80 ) ;
        byte <<= 1 ;
    }
}


void putPcmHead()
{
    // send 7E, do not CRC
    // 01111110
    putPcmPart( 0 ) ;
    putPcmPart( 1 ) ;
    putPcmPart( 1 ) ;
    putPcmPart( 1 ) ;
    putPcmPart( 1 ) ;
    putPcmPart( 1 ) ;
    putPcmPart( 1 ) ;
    putPcmPart( 0 ) ;
}


uint16_t PcmCrc_x ;
uint8_t PcmOnesCount_x ;

void crc_x( uint8_t data )
{
    //	uint8_t i ;

  PcmCrc_x =(PcmCrc_x<<8) ^ CRCTable((PcmCrc_x>>8)^data) ;
}


inline __attribute__ ((always_inline)) void putPcmPart_x( uint8_t value )
{
	uint16_t lpxxValue = PxxValue_x += 18 ;

	*PtrPxx_x++ = lpxxValue ;
	lpxxValue += 14 ;
	if ( value )
	{
		lpxxValue += 16 ;
	}
	*PtrPxx_x++ = lpxxValue ;	// Output 0 for this time
	PxxValue_x = lpxxValue ;
}

void putPcmFlush_x()
{
	*PtrPxx_x++ = 19000 ;		// Past the 18000 of the ARR
}


void putPcmBit_x( uint8_t bit )
{
    if ( bit )
    {
        PcmOnesCount_x += 1 ;
        putPcmPart_x( 1 ) ;
    }
    else
    {
        PcmOnesCount_x = 0 ;
        putPcmPart_x( 0 ) ;
    }
    if ( PcmOnesCount_x >= 5 )
    {
        putPcmBit_x( 0 ) ;				// Stuff a 0 bit in
    }
}



void putPcmByte_x( uint8_t byte )
{
    crc_x( byte ) ;
    uint8_t i ;

    for ( i = 0 ; i < 8 ; i += 1 )
    {
        putPcmBit_x( byte & 0x80 ) ;
        byte <<= 1 ;
    }
}


void putPcmHead_x()
{
    // 01111110
    putPcmPart_x( 0 ) ;
    putPcmPart_x( 1 ) ;
    putPcmPart_x( 1 ) ;
    putPcmPart_x( 1 ) ;
    putPcmPart_x( 1 ) ;
    putPcmPart_x( 1 ) ;
    putPcmPart_x( 1 ) ;
    putPcmPart_x( 0 ) ;
}


void setupPulsesPXX(uint8_t module)
{
  uint8_t i ;
  uint16_t chan ;
  uint16_t chan_1 ;
	uint8_t lpass ;

	if ( module == 0 )
	{
		lpass = Pass[module] ;

		PtrPxx = &pxxStream[module][0] ;
		PxxValue = 0 ;
		if ( g_model.Module[module].pxxDoubleRate )
		{
			if (lpass & 1)
			{
				PxxValue = 9000 ;
			}
		}

		PcmCrc = 0 ;
  	PcmOnesCount = 0 ;
  	putPcmPart( 0 ) ;
  	putPcmPart( 0 ) ;
  	putPcmPart( 0 ) ;
  	putPcmPart( 0 ) ;
  	putPcmHead(  ) ;  // sync byte
  	putPcmByte( g_model.Module[module].pxxRxNum ) ;     // putPcmByte( g_model.rxnum ) ;  //
 		uint8_t flag1;
 		if (BindRangeFlag[module] & PXX_BIND)
		{
 		  flag1 = (g_model.Module[module].sub_protocol<< 6) | (g_model.Module[module].country << 1) | BindRangeFlag[module] ;
 		}
 		else
		{
 		  flag1 = (g_model.Module[module].sub_protocol << 6) | BindRangeFlag[module] ;
		}

		if ( ( flag1 & (PXX_BIND | PXX_RANGE_CHECK )) == 0 )
		{
  		if (g_model.Module[module].failsafeMode != FAILSAFE_NOT_SET && g_model.Module[module].failsafeMode != FAILSAFE_RX )
			{
    		if ( FailsafeCounter[module] )
				{
	    		if ( FailsafeCounter[module]-- == 1 )
					{
    	  		flag1 |= PXX_SEND_FAILSAFE ;
					}
    			if ( ( FailsafeCounter[module] == 1 ) && (g_model.Module[module].sub_protocol == 0 ) )
					{
  	    		flag1 |= PXX_SEND_FAILSAFE ;
					}
				}
	    	if ( FailsafeCounter[module] == 0 )
				{
//					if ( g_model.Module[module].failsafeRepeat == 0 )
//					{
						FailsafeCounter[module] = 1000 ;
//					}
				}
			}
		}

		putPcmByte( flag1 ) ;     // First byte of flags
  	putPcmByte( 0 ) ;     // Second byte of flags

		uint8_t startChan = g_model.Module[module].startChannel ;
		if ( lpass & 1 )
		{
			startChan += 8 ;
		}
		chan = 0 ;
  	for ( i = 0 ; i < 8 ; i += 1 )		// First 8 channels only
  	{																	// Next 8 channels would have 2048 added
    	if (flag1 & PXX_SEND_FAILSAFE)
			{
				if ( g_model.Module[module].failsafeMode == FAILSAFE_HOLD )
				{
					chan_1 = 2047 ;
				}
				else if ( g_model.Module[module].failsafeMode == FAILSAFE_NO_PULSES )
				{
					chan_1 = 0 ;
				}
				else
				{
					// Send failsafe value
					int32_t value ;
					value = ( startChan < 16 ) ? g_model.Module[module].failsafe[startChan] : 0 ;
					value = ( value *3933 ) >> 9 ;
					value += 1024 ;
					chan_1 = limit( (int16_t)1, (int16_t)value, (int16_t)2046 ) ;
				}
			}
			else
			{
				chan_1 = scaleForPXX( startChan ) ;
			}
 			if ( lpass & 1 )
			{
				chan_1 += 2048 ;
			}
			startChan += 1 ;

//			if ( ( lpass & 1 ) == 0 )
//			{
//				if ( startChan == 1 )
//				{
//					chan_1 = 1024 ;
//					if ( ++PxxTestCounter > 10 )
//					{
//						chan_1 = 1024 * 3 / 4 + 1024 ;
//  					GPIO_ResetBits(GPIOA, PIN_EXTPPM_OUT) ; // Set high
//					}
//					else
//					{
//  					GPIO_SetBits(GPIOA, PIN_EXTPPM_OUT) ; // Set high
//					}
//					if ( PxxTestCounter > 21 )
//					{
//						PxxTestCounter = 0 ;
//					}
//				}
//			}

			if ( i & 1 )
			{
	  	  putPcmByte( chan ) ; // Low byte of channel
				putPcmByte( ( ( chan >> 8 ) & 0x0F ) | ( chan_1 << 4) ) ;  // 4 bits each from 2 channels
  		  putPcmByte( chan_1 >> 4 ) ;  // High byte of channel
			}
			else
			{
				chan = chan_1 ;
			}
  	}
	  uint8_t extra_flags = 0 ;

//		/* Ext. flag (holds antenna selection on Horus internal module, 0x00 otherwise) */
//#if defined(PCBHORUS)
//  if (port == INTERNAL_MODULE) {
//    extra_flags |= g_model.moduleData[port].pxx.external_antenna;
//  }
//#endif
//#if defined(BINDING_OPTIONS)
//  extra_flags |= g_model.moduleData[port].pxx.receiver_telem_off << 1;
//  extra_flags |= g_model.moduleData[port].pxx.receiver_channel_9_16 << 2;
//#endif
//  if (IS_MODULE_R9M(port)) {
//    extra_flags |= g_model.moduleData[port].pxx.power << 3;
//    // Disable s.port if internal module is active
//    if (IS_TELEMETRY_INTERNAL_MODULE || !g_model.moduleData[port].pxx.sport_out)
//      extra_flags |=  (1<< 5);
//  }
		if ( g_model.Module[module].highChannels )
//		if ( PxxExtra[module] & 1 )
		{
			extra_flags = (1 << 2 ) ;
		}
		if ( g_model.Module[module].disableTelemetry )
//		if ( PxxExtra[module] & 2 )
		{
			extra_flags |= (1 << 1 ) ;
		}
		if ( g_model.Module[module].sub_protocol == 3 )	// R9M
		{
			extra_flags |= g_model.Module[module].r9mPower << 3 ;
			if ( g_model.Module[module].r9MflexMode == 2 )
			{
				extra_flags |= 1 << 6 ;
			}
		}
		putPcmByte( extra_flags ) ;
  	chan = PcmCrc ;		        // get the crc
  	putPcmByte( chan >> 8 ) ; // Checksum hi
  	putPcmByte( chan ) ; 			// Checksum lo
  	putPcmHead(  ) ;      // sync byte
  	putPcmFlush() ;
		if (g_model.Module[module].sub_protocol == 1 )		// D8
		{
			lpass = 0 ;
		}
		else
		{
			lpass += 1 ;
			if ( g_model.Module[module].channels == 1 )
			{
				lpass = 0 ;
			}
		}
		Pass[module] = lpass ;

		if ( ( g_model.Module[module].pxxDoubleRate ) && ( (BindRangeFlag[module] & PXX_BIND) == 0 ) )
//		if ( g_model.Module[module].pxxDoubleRate )
		{
			if (lpass & 1)
			{
				TIM1->CCR2 = 8000 ;	            // Update time
			}
			else
			{
	  		TIM1->CCR2 = 17000 ;            // Update time
			}
		}
		else
		{
  		TIM1->CCR2 = 17000 ;            // Update time
		}
	}
	else // module == 1
	{
		lpass = Pass[module] ;
		PtrPxx_x = &pxxStream[module][0] ;
		PxxValue_x = 0 ;
		if ( g_model.Module[module].pxxDoubleRate )
		{
			if (lpass & 1)
			{
				PxxValue_x = 9000 ;
			}
		}

		PcmCrc_x = 0 ;
  	PcmOnesCount_x = 0 ;
		putPcmPart_x( 0 ) ;
  	putPcmPart_x( 0 ) ;
  	putPcmPart_x( 0 ) ;
  	putPcmPart_x( 0 ) ;

		putPcmHead_x(  ) ;  // sync byte
  	putPcmByte_x( g_model.Module[module].pxxRxNum ) ;     // putPcmByte( g_model.rxnum ) ;  //
 		uint8_t flag1;
 		if (BindRangeFlag[module] & PXX_BIND)
		{
 		  flag1 = (g_model.Module[module].sub_protocol<< 6) | (g_model.Module[module].country << 1) | BindRangeFlag[module] ;
 		}
 		else
		{
 		  flag1 = (g_model.Module[module].sub_protocol << 6) | BindRangeFlag[module] ;
		}

		if ( ( flag1 & (PXX_BIND | PXX_RANGE_CHECK )) == 0 )
		{
  		if (g_model.Module[module].failsafeMode != FAILSAFE_NOT_SET && g_model.Module[module].failsafeMode != FAILSAFE_RX )
			{
    		if ( FailsafeCounter[module] )
				{
	    		if ( FailsafeCounter[module]-- == 1 )
					{
    	  		flag1 |= PXX_SEND_FAILSAFE ;
					}
    			if ( ( FailsafeCounter[module] == 1 ) && (g_model.Module[module].sub_protocol == 0 ) )
					{
  	    		flag1 |= PXX_SEND_FAILSAFE ;
					}
				}
	    	if ( FailsafeCounter[module] == 0 )
				{
						FailsafeCounter[module] = 1000 ;
				}
			}
		}

		putPcmByte_x( flag1 ) ;     // First byte of flags
  	putPcmByte_x( 0 ) ;     // Second byte of flags

		uint8_t startChan = g_model.Module[module].startChannel ;
		if ( lpass & 1 )
		{
			startChan += 8 ;
		}
		chan = 0 ;
  	for ( i = 0 ; i < 8 ; i += 1 )		// First 8 channels only
  	{																	// Next 8 channels would have 2048 added
    	if (flag1 & PXX_SEND_FAILSAFE)
			{
				if ( g_model.Module[module].failsafeMode == FAILSAFE_HOLD )
				{
					chan_1 = 2047 ;
				}
				else if ( g_model.Module[module].failsafeMode == FAILSAFE_NO_PULSES )
				{
					chan_1 = 0 ;
				}
				else
				{
					// Send failsafe value
					int32_t value ;
					value = ( startChan < 16 ) ? g_model.Module[module].failsafe[startChan] : 0 ;
					value = ( value *3933 ) >> 9 ;
					value += 1024 ;
					chan_1 = limit( (int16_t)1, (int16_t)value, (int16_t)2046 ) ;
				}
			}
			else
			{
				chan_1 = scaleForPXX( startChan ) ;
			}
			if ( lpass & 1 )
			{
				chan_1 += 2048 ;
			}
			startChan += 1 ;

			if ( i & 1 )
			{
				putPcmByte_x( chan ) ; // Low byte of channel
				putPcmByte_x( ( ( chan >> 8 ) & 0x0F ) | ( chan_1 << 4) ) ;  // 4 bits each from 2 channels
  		  putPcmByte_x( chan_1 >> 4 ) ;  // High byte of channel
			}
			else
			{
				chan = chan_1 ;
			}
  	}
	  uint8_t extra_flags = 0 ;

//		/* Ext. flag (holds antenna selection on Horus internal module, 0x00 otherwise) */
//#if defined(PCBHORUS)
//  if (port == INTERNAL_MODULE) {
//    extra_flags |= g_model.moduleData[port].pxx.external_antenna;
//  }
//#endif
//#if defined(BINDING_OPTIONS)
//  extra_flags |= g_model.moduleData[port].pxx.receiver_telem_off << 1;
//  extra_flags |= g_model.moduleData[port].pxx.receiver_channel_9_16 << 2;
//#endif
//  if (IS_MODULE_R9M(port)) {
//    extra_flags |= g_model.moduleData[port].pxx.power << 3;
//    // Disable s.port if internal module is active
//    if (IS_TELEMETRY_INTERNAL_MODULE || !g_model.moduleData[port].pxx.sport_out)
//      extra_flags |=  (1<< 5);
//  }

// Bit 0: 0 internal, 1 external antenna
// Bit 1: 0 Telemetry ON, 1 Telemetry OFF
// Bit 2: 0 PPM 1-8, 1 PPM 9-16
// Bit 4:3: R9M power nonEU 10, 100, 500, 1000
// Bit 4:3: R9M power EU 25, 500
// Bit 5: 0 Sport enabled, 1 Sport disabled
// Bits 7:6 unused


		if ( g_model.Module[module].highChannels )
//		if ( PxxExtra[module] & 1 )
		{
			extra_flags = (1 << 2 ) ;
		}
		if ( g_model.Module[module].disableTelemetry )
//		if ( PxxExtra[module] & 2 )
		{
			extra_flags |= (1 << 1 ) ;
		}
		if ( g_model.Module[module].sub_protocol == 3 )	// R9M
		{
			extra_flags |= g_model.Module[module].r9mPower << 3 ;
			if ( g_model.Module[module].r9MflexMode == 2 )
			{
				extra_flags |= 1 << 6 ;
			}
		}
		putPcmByte_x( extra_flags ) ;
		chan = PcmCrc_x ;		        // get the crc
  	putPcmByte_x( chan >> 8 ) ; // Checksum hi
  	putPcmByte_x( chan ) ; 			// Checksum lo
  	putPcmHead_x(  ) ;      // sync byte
  	putPcmFlush_x() ;
		if (g_model.Module[module].sub_protocol == 1 )		// D8
		{
			lpass = 0 ;
		}
		else
		{
			lpass += 1 ;
			if ( g_model.Module[module].channels == 1 )
			{
				lpass = 0 ;
			}
		}
		Pass[module] = lpass ;

		if ( ( g_model.Module[module].pxxDoubleRate ) && ( (BindRangeFlag[module] & PXX_BIND) == 0 ) )
		{
			if (lpass & 1)
			{
				TIM8->CCR2 = 8000 ;	            // Update time
			}
			else
			{
	  		TIM8->CCR2 = 17000 ;            // Update time
			}
		}
		else
		{
  		TIM8->CCR2 = 17000 ;            // Update time
		}
	}
}



#endif
