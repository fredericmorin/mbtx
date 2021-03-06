/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
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
 */

#include <stdlib.h>
#include "ersky9x.h"
#include "stm32f2xx.h"
#include "logicio.h"
#include "X9D/hal.h"
#include "X9D/stm32f2xx_usart.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_gpio.h"
#include "core_cm3.h"
#include "timers.h"
#include "myeeprom.h"
#include "drivers.h"
#include "frsky.h"
#define CONVERT_PTR(x) ((uint32_t)(uint64_t)(x))

#define NUM_MODULES 2

#define INTERNAL_RF_ON()      GPIO_SetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define INTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR)

#define EXT_TYPE_PXX		0
#define EXT_TYPE_DSM		1
#define EXT_TYPE_MULTI	2


uint8_t s_current_protocol[NUM_MODULES] = { 255, 255 } ;

extern void setupPulses(unsigned int port);
void setupPulsesPPM(unsigned int port);
uint8_t setupPulsesXfire() ;

uint16_t *ppmStreamPtr[NUM_MODULES];
uint16_t pulseStreamCount[NUM_MODULES] ;
uint16_t ppmStream[NUM_MODULES+1][20];
uint16_t pxxStream[NUM_MODULES][400];
uint16_t dsm2Stream[2][400];

#ifdef XFIRE
extern uint8_t Bit_pulses[] ;
uint16_t XfireLength ;
#endif

static void init_int_pxx( void ) ;
static void init_ext_pxx( void ) ;
static void disable_ext_pxx( void ) ;
static void disable_int_pxx( void ) ;

#if defined(PCBXLITE) || defined(PCBX9LITE)
#else
static void init_pa10_ppm( void ) ;
static void disable_pa10_ppm( void ) ;
static void init_ext_dsm2( void ) ;
static void disable_ext_dsm2( void ) ;
static void init_ext_ppm( void ) ;
static void disable_ext_ppm( void ) ;
static void init_ext_multi( void ) ;
#ifdef XFIRE
static void init_pa7_xfire( void ) ;
static void disable_pa7_xfire( void ) ;
#endif
static void init_pa10_none( void ) ;
static void disable_pa10_none( void ) ;
static void init_pa7_none( void ) ;
static void disable_pa7_none( void ) ;
#endif

void init_pxx(uint32_t port)
{
  if (port == INTERNAL_MODULE)
    init_int_pxx() ;
  else
    init_ext_pxx() ;
}

void disable_pxx(uint32_t port)
{
  if (port == INTERNAL_MODULE)
    disable_int_pxx() ;
  else
    disable_ext_pxx() ;
}

void init_dsm2(uint32_t port)
{
  if (port == EXTERNAL_MODULE)
	{
    init_ext_dsm2() ;
  }
}

void init_multi(uint32_t port)
{
  if (port == EXTERNAL_MODULE)
	{
    init_ext_multi() ;
  }
}

void disable_dsm2(uint32_t port)
{
  if (port == EXTERNAL_MODULE)
	{
    disable_ext_dsm2();
  }
}


#ifdef XFIRE
void init_xfire(uint32_t port)
{
  if (port == EXTERNAL_MODULE)
	{
    init_pa7_xfire() ;
  }
}

void disable_xfire(uint32_t port)
{
  if (port == EXTERNAL_MODULE)
	{
    disable_pa7_xfire() ;
  }
}
#endif

void init_ppm(uint32_t port)
{
  if (port == INTERNAL_MODULE)
    init_pa10_ppm(); // TODO needed?
  else
		init_ext_ppm() ;
}

void disable_ppm(uint32_t port)
{
  if (port == INTERNAL_MODULE)
    disable_pa10_ppm(); // TODO needed?
  else
    disable_ext_ppm();
}

void init_no_pulses(uint32_t port)
{
  if (port == INTERNAL_MODULE)
    init_pa10_none();
  else
    init_pa7_none();
}

void disable_no_pulses(uint32_t port)
{
  if (port == INTERNAL_MODULE)
    disable_pa10_none();
  else
    disable_pa7_none();
}

static void init_pa10_none()
{
  INTERNAL_RF_OFF();

  // Timer1, channel 3
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;           // Enable portA clock

	configure_pins( PIN_INTPPM_OUT, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTA ) ;

  GPIO_SetBits(GPIO_INTPPM, PIN_INTPPM_OUT) ; // Set high

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;            // Enable clock

  TIM1->CR1 &= ~TIM_CR1_CEN ;
  TIM1->ARR = 35999 ;             // 18mS
  TIM1->CCR2 = 32000 ;            // Update time
  TIM1->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;               // 0.5uS from 30MHz

  TIM1->CCER = TIM_CCER_CC3E ;

  TIM1->CCMR2 = 0 ;
  TIM1->EGR = 1 ;                                                         // Restart

  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 ;                     // Toggle CC1 o/p
  TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
  TIM1->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  TIM1->CR1 |= TIM_CR1_CEN ;
	NVIC_SetPriority( TIM1_CC_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM1_CC_IRQn) ;
}

static void disable_pa10_none()
{
  NVIC_DisableIRQ(TIM1_CC_IRQn) ;
  TIM1->DIER &= ~TIM_DIER_CC2IE ;
  TIM1->CR1 &= ~TIM_CR1_CEN ;
}

static void init_pa7_none()
{
  EXTERNAL_RF_OFF() ;

  // Timer8

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;           // Enable portA clock

	configure_pins( PIN_EXTPPM_OUT, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTA ) ;

  GPIO_SetBits(GPIOA, PIN_EXTPPM_OUT) ; // Set high

  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN ;            // Enable clock

  TIM8->CR1 &= ~TIM_CR1_CEN ;
  TIM8->ARR = 35999 ;             // 18mS
  TIM8->CCR2 = 32000 ;            // Update time
  TIM8->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;               // 0.5uS from 30MHz

  TIM8->CCMR2 = 0 ;
  TIM8->EGR = 1 ;                                                         // Restart

  TIM8->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 ;                     // Toggle CC1 o/p
  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
  TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  TIM8->CR1 |= TIM_CR1_CEN ;
	NVIC_SetPriority( TIM8_CC_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM8_CC_IRQn) ;
}

static void disable_pa7_none()
{
  NVIC_DisableIRQ(TIM8_CC_IRQn) ;
  TIM8->DIER &= ~TIM_DIER_CC2IE ;
  TIM8->CR1 &= ~TIM_CR1_CEN ;
}

#define PA10_TYPE_PXX			0
#define PA10_TYPE_DSM			1
#define PA10_TYPE_MULTI		2
#define PA10_TYPE_ACCESS	3

void init_pa10_serial( uint32_t type )
{
  INTERNAL_RF_ON();
	if ( type == PA10_TYPE_PXX )
	{
  	setupPulsesPXX(INTERNAL_MODULE);
	}
	else
	{
  		setupPulsesDsm2(6, INTERNAL_MODULE) ;
	}

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;           // Enable portA clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_INTPPM, ENABLE);

	configure_pins( PIN_INTPPM_OUT, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_1 | PIN_OS25 | PIN_PUSHPULL ) ;
//#ifdef ACCESS
//	configure_pins( INTMODULE_RX_GPIO_PIN, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_1 ) ;
//#endif

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;            // Enable clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN ;            // Enable DMA2 clock

  TIM1->CR1 &= ~TIM_CR1_CEN ;

	if ( type == PA10_TYPE_PXX )
	{
		{
			TIM1->ARR = 17999 ;                     // 9mS
  		TIM1->CCR2 = 16000 ;            // Update time
		}
	}
	else if ( type == PA10_TYPE_DSM )
	{
		TIM1->ARR = 43999 ;                     // 22mS
  	TIM1->CCR2 = 40000 ;            // Update time
	}
	else // type == PA10_TYPE_MULTI
	{
//  	TIM1->ARR = 14000 ;             // 7mS
//  	TIM1->CCR2 = 10000 ;            // Update time
		uint32_t x ;
		x = g_model.Module[0].ppmFrameLength ;
		if ( x > 4 )
		{
			x = 0 ;
		}
		x *= 2000 ;
		x += 7000 * 2 - 1 ;
  	TIM8->ARR = x ;             // 11mS
  	TIM8->CCR2 = x-4000 ;       // Update time
	}
  TIM1->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;               // 0.5uS from 30MHz

  TIM1->CCER = TIM_CCER_CC3E ;

  TIM1->CR2 = TIM_CR2_OIS3 ;              // O/P idle high
  TIM1->BDTR = TIM_BDTR_MOE ;             // Enable outputs
	if ( type == PA10_TYPE_PXX )
	{
	  TIM1->CCR3 = pxxStream[INTERNAL_MODULE][0];
	  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0 ;                     // Force O/P high
	}
	else
	{
	  TIM1->CCR3 = dsm2Stream[INTERNAL_MODULE][0] ;
	  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 ; // Force O/P low, hardware inverts it
	}
  TIM1->EGR = 1 ;                                                         // Restart

  DMA2_Stream6->CR &= ~DMA_SxCR_EN ;              // Disable DMA

	TIM1->DIER |= TIM_DIER_CC3DE ;          		// Enable DMA on CC3 match
  TIM1->DCR = 15 ;                            // DMA to CC1

  // Enable the DMA channel here, DMA2 stream 6, channel 6
  DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6 ; // Write ones to clear bits
  DMA2_Stream6->CR = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_0
                                                         | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_PFCTRL ;
  DMA2_Stream6->PAR = CONVERT_PTR(&TIM1->DMAR);
	if ( type == PA10_TYPE_PXX )
	{
  	DMA2_Stream6->M0AR = CONVERT_PTR(&pxxStream[INTERNAL_MODULE][1]);
	}
	else
	{
  	DMA2_Stream6->M0AR = CONVERT_PTR(&dsm2Stream[0][1]);
	}
  DMA2_Stream6->CR |= DMA_SxCR_EN ;               // Enable DMA

  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 ;                     // Toggle CC1 o/p
  TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
  TIM1->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  TIM1->CR1 |= TIM_CR1_CEN ;
	NVIC_SetPriority( TIM1_CC_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM1_CC_IRQn) ;
}


static void init_int_pxx()
{
	init_pa10_serial( PA10_TYPE_PXX ) ;
}

static void disable_int_pxx()
{
  DMA2_Stream6->CR &= ~DMA_SxCR_EN ;              // Disable DMA
  NVIC_DisableIRQ(TIM1_CC_IRQn) ;
  TIM1->DIER &= ~TIM_DIER_CC2IE ;
  TIM1->CR1 &= ~TIM_CR1_CEN ;
  INTERNAL_RF_OFF();
}

// PPM output
// Timer 1, channel 1 on PA8 for prototype
// Pin is AF1 function for timer 1
static void init_pa10_ppm()
{
  INTERNAL_RF_ON();
  // Timer1
//  setupPulsesPPM(INTERNAL_MODULE) ;
  ppmStreamPtr[INTERNAL_MODULE] = ppmStream[INTERNAL_MODULE];

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;           // Enable portA clock

  configure_pins( PIN_INTPPM_OUT, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_1 | PIN_OS25 | PIN_PUSHPULL ) ;

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;            // Enable clock

  TIM1->CR1 &= ~TIM_CR1_CEN ;
  TIM1->ARR = *ppmStreamPtr[INTERNAL_MODULE]++ ;
  TIM1->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;               // 0.5uS from 30MHz

  TIM1->CCER = TIM_CCER_CC3E ;
  if(!g_model.Module[0].pulsePol)
    TIM1->CCER |= TIM_CCER_CC3P;

  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 ;     // PWM mode 1
  TIM1->CCMR1 = TIM_CCMR1_OC2PE ;                   			// PWM mode 1
  TIM1->CCR3 = (g_model.Module[0].ppmDelay*50+300)*2;
  TIM1->BDTR = TIM_BDTR_MOE ;
  TIM1->EGR = 1 ;
  TIM1->DIER = TIM_DIER_UDE ;

  TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_UIF ;                               // Clear flag
  TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
  TIM1->DIER |= TIM_DIER_CC2IE ;
  TIM1->DIER |= TIM_DIER_UIE ;

  TIM1->CR1 = TIM_CR1_CEN ;
	NVIC_SetPriority( TIM1_CC_IRQn, 3 ) ; // Lower priority interrupt
	NVIC_SetPriority( TIM1_UP_TIM10_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM1_CC_IRQn) ;
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn) ;
}

static void disable_pa10_ppm()
{
  NVIC_DisableIRQ(TIM1_CC_IRQn) ;
  NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn) ;
  TIM1->DIER &= ~TIM_DIER_CC2IE & ~TIM_DIER_UIE ;
  TIM1->CR1 &= ~TIM_CR1_CEN ;

  INTERNAL_RF_OFF();
}

uint16_t XjtHbeatOffset ;

//extern uint16_t g_timePXX;
extern "C" void TIM1_CC_IRQHandler()
{
  TIM1->DIER &= ~TIM_DIER_CC2IE ;         // stop this interrupt
  TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag

//  DMA2_Stream6->CR &= ~DMA_SxCR_EN ;              // Disable DMA
	setupPulses(INTERNAL_MODULE) ;

  if (s_current_protocol[INTERNAL_MODULE] == PROTO_PXX)
	{
		XjtHbeatOffset = TIM7->CNT - XjtHeartbeatCapture.value ;
		if ( XjtHeartbeatCapture.valid )
		{
			if ( XjtHbeatOffset > 0x2A00 )
//			if ( XjtHbeatOffset > 0x2200 )
			{
				TIM1->ARR = 17979 ;                     // 9mS
			}
			else
			{
				TIM1->ARR = 18019 ;                     // 9mS
			}
		}
    DMA2_Stream6->CR &= ~DMA_SxCR_EN ;              // Disable DMA
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6 ; // Write ones to clear bits
    DMA2_Stream6->M0AR = CONVERT_PTR(&pxxStream[INTERNAL_MODULE][1]);
    DMA2_Stream6->CR |= DMA_SxCR_EN ;               // Enable DMA
    TIM1->CCR3 = pxxStream[INTERNAL_MODULE][0];
    TIM1->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  }
  else if ( (s_current_protocol[INTERNAL_MODULE] == PROTO_DSM2 ) || (s_current_protocol[INTERNAL_MODULE] == PROTO_MULTI ) )
	{
    DMA2_Stream6->CR &= ~DMA_SxCR_EN ;              // Disable DMA
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6 ; // Write ones to clear bits
    DMA2_Stream6->M0AR = CONVERT_PTR(&dsm2Stream[0][1]);
	  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 ; // Force O/P low, hardware inverts it
	  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 ;                     // Toggle CC1 o/p
    DMA2_Stream6->CR |= DMA_SxCR_EN ;               // Enable DMA
    TIM1->CCR3 = dsm2Stream[0][0];
    TIM1->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  }
  else if (s_current_protocol[INTERNAL_MODULE] == PROTO_PPM)
	{
    ppmStreamPtr[INTERNAL_MODULE] = ppmStream[INTERNAL_MODULE];
    TIM1->DIER |= TIM_DIER_UDE ;
    TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_UIF ;                                       // Clear this flag
    TIM1->DIER |= TIM_DIER_UIE ;                            // Enable this interrupt
  }
  else {
    TIM1->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  }
//  t0 = TIM3->CNT - t0;
//	g_timePXX = t0 ;

}

extern "C" void TIM1_UP_TIM10_IRQHandler()
{
 		TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_UIF ;                               // Clear flag

 		TIM1->ARR = *ppmStreamPtr[INTERNAL_MODULE]++ ;
 		if ( *ppmStreamPtr[INTERNAL_MODULE] == 0 )
 		{
 		  TIM1->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                     // Clear this flag
 		  TIM1->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
 		}
}

void init_ext_serial( uint32_t type )
{
  EXTERNAL_RF_ON() ;
	if ( type == EXT_TYPE_PXX )
	{
  	setupPulsesPXX(EXTERNAL_MODULE);
	}
	else
	{
  	setupPulsesDsm2(6, EXTERNAL_MODULE) ;
	}

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;           // Enable portA clock
  configure_pins( PIN_EXTPPM_OUT, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_3 | PIN_OS25 | PIN_PUSHPULL ) ;
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN ;            // Enable clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN ;            // Enable DMA2 clock

	TIM8->CR1 &= ~TIM_CR1_CEN ;

	if ( type == EXT_TYPE_PXX )
	{
		TIM8->ARR = 17999 ;                     // 9mS
  	TIM8->CCR2 = 16000 ;            // Update time
	}
	else if ( type == EXT_TYPE_DSM )
	{
		TIM8->ARR = 43999 ;                     // 22mS
  	TIM8->CCR2 = 40000 ;            // Update time
	}
	else // type == EXT_TYPE_MULTI
	{
//  	TIM8->ARR = 14000 ;             // 7mS
//  	TIM8->CCR2 = 10000 ;            // Update time
		uint32_t x ;
		x = g_model.Module[1].ppmFrameLength ;
		if ( x > 4 )
		{
			x = 0 ;
		}
		x *= 2000 ;
		x += 7000 * 2 ;
  	// TIM8->ARR = x ;             // 11mS
  	// TIM8->CCR2 = x-4000 ;       // Update time
    TIM8->ARR = 17999 ;    // 9mS
    TIM8->CCR2 = 16000 ;   // Update time

	}
  TIM8->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;               // 0.5uS from 30MHz
  TIM8->CCER = TIM_CCER_CC1NE ;
  TIM8->CR2 = TIM_CR2_OIS1 ;                      // O/P idle high
  TIM8->BDTR = TIM_BDTR_MOE ;             // Enable outputs
	if ( type == EXT_TYPE_PXX )
	{
	  TIM8->CCR1 = pxxStream[EXTERNAL_MODULE][0] ;
  	TIM8->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0 ;                     // Force O/P high
	}
	else
	{
	  TIM8->CCR1 = dsm2Stream[EXTERNAL_MODULE][0] ;
	  TIM8->CCMR1 = TIM_CCMR1_OC1M_2 ; // Force O/P low, hardware inverts it
	}
  TIM8->EGR = 1 ;                                                         // Restart

	TIM8->DIER |= TIM_DIER_CC1DE ;          // Enable DMA on CC1 match
  TIM8->DCR = 13 ;                                                                // DMA to CC1


  // Enable the DMA channel here, DMA2 stream 2, channel 7
  DMA2_Stream2->CR &= ~DMA_SxCR_EN ;              // Disable DMA
  DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2 ; // Write ones to clear bits
  DMA2_Stream2->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_0
																												 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_PFCTRL ;

	DMA2_Stream2->PAR = CONVERT_PTR(&TIM8->DMAR);

	if ( type == EXT_TYPE_PXX )
	{
		DMA2_Stream2->M0AR = CONVERT_PTR(&pxxStream[EXTERNAL_MODULE][1]);
  }
	else
	{
  	DMA2_Stream2->M0AR = CONVERT_PTR(&dsm2Stream[1][1]);
	}

	DMA2_Stream2->CR |= DMA_SxCR_EN ;               // Enable DMA

  TIM8->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 ;                     // Toggle CC1 o/p
  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
  TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  TIM8->CR1 |= TIM_CR1_CEN ;
	NVIC_SetPriority( TIM8_CC_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM8_CC_IRQn) ;
}

static void init_ext_pxx()
{
	init_ext_serial( EXT_TYPE_PXX ) ;
}

static void disable_ext_pxx()
{
  DMA2_Stream2->CR &= ~DMA_SxCR_EN ;              // Disable DMA
  NVIC_DisableIRQ(TIM8_CC_IRQn) ;
  TIM8->DIER &= ~TIM_DIER_CC2IE ;
  TIM8->CR1 &= ~TIM_CR1_CEN ;
  EXTERNAL_RF_OFF();
}


#ifdef XFIRE
static void init_pa7_xfire()
{
	com1_Configure( XFIRE_BAUD_RATE, 0, 0 ) ;
  EXTERNAL_RF_ON();
	setupPulsesXfire() ;

	configure_pins( PIN_EXTPPM_OUT, PIN_INPUT | PIN_PORTA ) ;

//	configure_pins( PIN_EXTPPM_OUT, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTA ) ;
//  GPIO_SetBits(GPIOA, PIN_EXTPPM_OUT) ; // Set high

	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN ;            // Enable clock

  TIM8->CR1 &= ~TIM_CR1_CEN ;
  TIM8->ARR = 7999 ;    // 4mS
  TIM8->CCR2 = 5000 ;   // Update time
//  TIM8->CCR1 = 10000 ;   // Tx back on time
//  TIM8->CCR3 = 1936*2 ;   // Tx hold on until time
  TIM8->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;  // 0.5uS from 30MHz
  TIM8->CCER = TIM_CCER_CC1NE ;
  TIM8->EGR = 0 ;                                                         // Restart
  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
//  TIM8->DIER |= TIM_DIER_CC2IE | TIM_DIER_CC1IE | TIM_DIER_CC3IE ;  // Enable these interrupts
  TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  TIM8->DIER |= TIM_DIER_UIE ;
  TIM8->CR1 |= TIM_CR1_CEN ;
	NVIC_SetPriority( TIM8_CC_IRQn, 3 ) ; // Lower priority interrupt
	NVIC_SetPriority( TIM8_UP_TIM13_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM8_CC_IRQn) ;
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn) ;
}

static void disable_pa7_xfire()
{
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn) ;
  NVIC_DisableIRQ(TIM8_CC_IRQn) ;
  TIM8->DIER &= ~( TIM_DIER_CC2IE | TIM_DIER_CC1IE | TIM_DIER_CC3IE | TIM_DIER_UIE ) ;
  TIM8->CR1 &= ~TIM_CR1_CEN ;
  EXTERNAL_RF_OFF() ;
}
#endif

static void init_ext_dsm2()
{
	init_ext_serial( EXT_TYPE_DSM ) ;
}

static void init_ext_multi()
{
	init_ext_serial( EXT_TYPE_MULTI ) ;
}

static void disable_ext_dsm2()
{
  DMA2_Stream2->CR &= ~DMA_SxCR_EN ;              // Disable DMA
  NVIC_DisableIRQ(TIM8_CC_IRQn) ;
  TIM8->DIER &= ~TIM_DIER_CC2IE ;
  TIM8->CR1 &= ~TIM_CR1_CEN ;
  EXTERNAL_RF_OFF();
}

// PPM output
// Timer 1, channel 1 on PA8 for prototype
// Pin is AF1 function for timer 1
static void init_ext_ppm()
{
  EXTERNAL_RF_ON();
  // Timer8
//  setupPulsesPpmx() ;
  ppmStreamPtr[EXTERNAL_MODULE] = ppmStream[EXTERNAL_MODULE];

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;           // Enable portA clock
  configure_pins( PIN_EXTPPM_OUT, PIN_PERIPHERAL | PORT_EXTPPM | PIN_PER_3 | PIN_OS25 | PIN_PUSHPULL ) ;
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN ;            // Enable clock

  TIM8->CR1 &= ~TIM_CR1_CEN ;

  TIM8->ARR = *ppmStreamPtr[EXTERNAL_MODULE]++ ;
  TIM8->PSC = (PeripheralSpeeds.Peri2_frequency * PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;               // 0.5uS from 30MHz
  TIM8->CCER = TIM_CCER_CC1NE ;
  if(!g_model.Module[1].pulsePol)
    TIM8->CCER |= TIM_CCER_CC1NP;
  TIM8->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2PE ;                   // PWM mode 1
  TIM8->CCR1 = (g_model.Module[1].ppmDelay*50+300)*2 ;
  TIM8->BDTR = TIM_BDTR_MOE ;
  TIM8->EGR = 1 ;
  TIM8->DIER = TIM_DIER_UDE ;

  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_UIF ;                               // Clear flag
  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag
  TIM8->DIER |= TIM_DIER_CC2IE ;
  TIM8->DIER |= TIM_DIER_UIE ;

  TIM8->CR1 = TIM_CR1_CEN ;
	NVIC_SetPriority( TIM8_CC_IRQn, 3 ) ; // Lower priority interrupt
	NVIC_SetPriority( TIM8_UP_TIM13_IRQn, 3 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(TIM8_CC_IRQn) ;
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn) ;
}

static void disable_ext_ppm()
{
  NVIC_DisableIRQ(TIM8_CC_IRQn) ;
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn) ;
  TIM8->DIER &= ~TIM_DIER_CC2IE & ~TIM_DIER_UIE ;
  TIM8->CR1 &= ~TIM_CR1_CEN ;
  EXTERNAL_RF_OFF();
}

extern int16_t scaleAnalog( int16_t v, uint8_t channel ) ;
extern void getADC_single( void ) ;
extern uint32_t MixerCount ;
extern SKYMixData *mixAddress( uint32_t index ) ;
extern int16_t calcExpo( uint8_t channel, int16_t value ) ;

extern "C" void TIM8_CC_IRQHandler()
{
  TIM8->DIER &= ~TIM_DIER_CC2IE ;         // stop this interrupt
  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                             // Clear flag

  ////////////////////////////////////////////////
  // Refresh gimbals values right before the outbound packet is built by setupPulses()
  getADC_single();
  MixerCount += 1 ;  // help us see in the debug screen that we actually did something here
  // re-re-re-reeeeeemiiiixx
  {  // very simplified version of the mixer that distribute pots into channel outs.
    uint8_t stickIndex = g_eeGeneral.stickMode*4 ;
    int16_t rawSticks[4] ;
    int16_t v;
    int16_t anas[6] = {0};
    // process value of the first 6 pots.
    for (uint8_t i = 0; i < 6; i++) {
      v = anaIn( i ) ;
      v = scaleAnalog( v, i ) ;
      uint8_t index = i ;
      if ( i < 4 ) {
        index = stickScramble[stickIndex+i];
        rawSticks[index] = v; //set values for mixer
        v = calcExpo( index, v ) ;
      }
      if ( i < 6 ) {
        anas[index] = v ; //set values for mixer
      }
    }
    // simple mix the first 4 outputs
    for(uint8_t i=0;i<MAX_SKYMIXERS+EXTRA_SKYMIXERS;i++) {
      SKYMixData *md = mixAddress(i);
      if((md->destCh==0) || (md->destCh>NUM_SKYCHNOUT+EXTRA_SKYCHANNELS)) break;

      uint8_t dest = md->destCh-1;
      if (dest >= 4) continue;  // we dont output channel other than 0..3

      uint8_t k = md->srcRaw - 1;
      if (k >= 6) continue;  // we dont support analog source other than 0..5

      v = 0;
      if (k < 6) {
        v = anas[k]; //Switch is on. MAX=FULL=512 or value.
      }
      if (k < 4) {
        v = rawSticks[k]; //Switch is on. MAX=FULL=512 or value.
      }

      g_chans512[dest] = v;
    }
  }
  ////////////////////////////////////////////////

  // build packet going out using g_chans512 values
  setupPulses(EXTERNAL_MODULE) ;

  if (s_current_protocol[EXTERNAL_MODULE] == PROTO_PXX)
	{
	  if (s_current_protocol[INTERNAL_MODULE] != PROTO_PXX)
		{
			XjtHbeatOffset = TIM7->CNT - XjtHeartbeatCapture.value ;
			if ( XjtHeartbeatCapture.valid )
			{
				if ( XjtHbeatOffset > 0x2200 )  // 8704  = 4352us
				{
					TIM8->ARR = 17979 ;                     // 9mS
				}
				else
				{
					TIM8->ARR = 18019 ;                     // 9mS
				}
			}
		}
    DMA2_Stream2->CR &= ~DMA_SxCR_EN ;              // Disable DMA
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2 ; // Write ones to clear bits
    DMA2_Stream2->M0AR = CONVERT_PTR(&pxxStream[EXTERNAL_MODULE][1]);
    DMA2_Stream2->CR |= DMA_SxCR_EN ;               // Enable DMA
    TIM8->CCR1 = pxxStream[EXTERNAL_MODULE][0];
    TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  }
  else if ( (s_current_protocol[EXTERNAL_MODULE] == PROTO_DSM2 ) || (s_current_protocol[EXTERNAL_MODULE] == PROTO_MULTI ) )
	{
		MultiHeartbeatCapture.offset = (TIM7->CNT - MultiHeartbeatCapture.value) % 18000 ;  // 9ms
		if ( MultiHeartbeatCapture.valid )
		{
      #define CORRECTION 200                       // 0.1ms
			if ( MultiHeartbeatCapture.offset > 11000 )  // 5.5ms
			{
				TIM8->ARR = (18000 - CORRECTION);          // 9mS - corr
			}
			else
			{
				TIM8->ARR = (18000 + CORRECTION);          // 9mS + corr
			}
		}
    DMA2_Stream2->CR &= ~DMA_SxCR_EN ;              // Disable DMA
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2 ; // Write ones to clear bits
    DMA2_Stream2->M0AR = CONVERT_PTR(&dsm2Stream[1][1]);
  	TIM8->CCMR1 = TIM_CCMR1_OC1M_2 ; // Force O/P low, hardware inverts it
	  TIM8->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 ;                     // Toggle CC1 o/p
    DMA2_Stream2->CR |= DMA_SxCR_EN ;               // Enable DMA
    TIM8->CCR1 = dsm2Stream[1][0];
    TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  }
  else if (s_current_protocol[EXTERNAL_MODULE] == PROTO_PPM) {
    ppmStreamPtr[EXTERNAL_MODULE] = ppmStream[EXTERNAL_MODULE];
    TIM8->DIER |= TIM_DIER_UDE ;
    TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_UIF ;                                       // Clear this flag
    TIM8->DIER |= TIM_DIER_UIE ;                            // Enable this interrupt
  }
  else
	{
    TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler()
{
  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_UIF ;                               // Clear flag
#ifdef XFIRE
	if (s_current_protocol[EXTERNAL_MODULE] == PROTO_XFIRE )
	{
		x9dSPortTxStart( (uint8_t *)Bit_pulses, XfireLength, 0 ) ;
	}
	else
#endif
	{
  	TIM8->ARR = *ppmStreamPtr[EXTERNAL_MODULE]++ ;
	  if (*ppmStreamPtr[EXTERNAL_MODULE] == 0)
		{
  	  TIM8->SR = TIMER1_8SR_MASK & ~TIM_SR_CC2IF ;                     // Clear this flag
	    TIM8->DIER |= TIM_DIER_CC2IE ;  // Enable this interrupt
  	}
	}
}
