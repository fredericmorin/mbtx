/*
 * Author - Mike Blandford
 *
 * Based on th9x -> http://code.google.com/p/th9x/
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

#include <stdint.h>
#include <stdlib.h>

#if defined(PCBX9D) || defined(PCB9XT)
#include "X9D/stm32f2xx.h"
//#include "X9D/stm32f2xx_gpio.h"
#include "X9D/stm32f2xx_rcc.h"
#include "X9D/stm32f2xx_usart.h"
#include "X9D/hal.h"
//#include "timers.h"
#endif

#include "ersky9x.h"
#include "myeeprom.h"
#include "drivers.h"
#include "logicio.h"
#include "frsky.h"
#include "timers.h"

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)

// States in LineState
#define LINE_IDLE			0
#define LINE_ACTIVE		1

// States in BitState
#define BIT_IDLE			0
#define BIT_ACTIVE		1
#define BIT_FRAMING		2

extern struct t_telemetryTx TelemetryTx ;
uint16_t USART_ERRORS ;
uint16_t USART_ORE ;
uint16_t USART_NE ;
uint16_t USART_FE ;
uint16_t USART_PE ;

#define XJT_HEARTBEAT_BIT	0x0080		// PC7

void putCaptureTime( struct t_softSerial *pss, uint16_t time, uint32_t value ) ;

uint8_t LastReceivedSportByte ;

extern struct t_serial_tx *Current_Com2 ;

void USART6_Sbus_configure()
{
	stop_xjt_heartbeat() ;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN ;		// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	configure_pins( 0x0080, PIN_PERIPHERAL | PIN_PORTC | PIN_PER_8 ) ;
	USART6->BRR = PeripheralSpeeds.Peri2_frequency / 100000 ;
	USART6->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE ;
	USART6->CR2 = 0 ;
	USART6->CR3 = 0 ;
	(void) USART6->DR ;
	NVIC_SetPriority( USART6_IRQn, 5 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(USART6_IRQn) ;

}

void stop_USART6_Sbus()
{
	configure_pins( 0x0080, PIN_INPUT | PIN_PORTC ) ;
  NVIC_DisableIRQ(USART6_IRQn) ;
	init_xjt_heartbeat() ;
}

extern "C" void USART6_IRQHandler()
{
	put_fifo64( &Sbus_fifo, USART6->DR ) ;
}


void UART_Sbus_configure( uint32_t masterClock )
{
	USART3->BRR = PeripheralSpeeds.Peri1_frequency / 100000 ;
	USART3->CR1 |= USART_CR1_M | USART_CR1_PCE ;
}

void UART_Sbus57600_configure( uint32_t masterClock )
{
	USART3->BRR = PeripheralSpeeds.Peri1_frequency / 57600 ;
	USART3->CR1 |= USART_CR1_M | USART_CR1_PCE ;
}


void ConsoleInit()
{
	// Serial configure
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN ;		// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
	GPIOB->MODER = (GPIOB->MODER & 0xFF0FFFFF ) | 0x00A00000 ;	// Alternate func.
	GPIOB->AFR[1] = (GPIOB->AFR[1] & 0xFFFF00FF ) | 0x00007700 ;	// Alternate func.
	USART3->BRR = PeripheralSpeeds.Peri1_frequency / CONSOLE_BAUDRATE ;		// 97.625 divider => 19200 baud
	USART3->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE ;
	USART3->CR2 = 0 ;
	USART3->CR3 = 0 ;
	NVIC_SetPriority( USART3_IRQn, 5 ) ; // Lower priority interrupt
  NVIC_EnableIRQ(USART3_IRQn) ;
}

void com2_Configure( uint32_t baudrate, uint32_t invert, uint32_t parity )
{
	ConsoleInit() ;
	USART3->BRR = PeripheralSpeeds.Peri1_frequency / baudrate ;
	com2Parity( parity ) ;
}

void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity )
{
	// Serial configure
	if ( invert )
	{
	  NVIC_DisableIRQ(USART2_IRQn) ;
		init_software_com1( baudRate, invert, parity ) ;
		return ;
	}
	disable_software_com1() ;

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN ;		// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	GPIOD->BSRRH = PIN_SPORT_ON ;		// output disable
	GPIOD->MODER = (GPIOD->MODER & 0xFFFFC0FF ) | 0x00002900 ;	// Alternate func.
	GPIOD->AFR[0] = (GPIOD->AFR[0] & 0xF00FFFFF ) | 0x07700000 ;	// Alternate func.
	if ( baudRate > 10 )
	{
		USART2->BRR = PeripheralSpeeds.Peri1_frequency / baudRate ;
	}
	else if ( baudRate == 0 )
	{
		USART2->BRR = PeripheralSpeeds.Peri1_frequency / 57600 ;		// 16.25 divider => 57600 baud
	}
	else
	{
		USART2->BRR = PeripheralSpeeds.Peri1_frequency / 9600 ;		// 97.625 divider => 9600 baud
	}
	USART2->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE ;
	USART2->CR2 = 0 ;
	USART2->CR3 = 0 ;
	if ( parity )
	{
		USART2->CR1 |= USART_CR1_PCE | USART_CR1_M ;	// Need 9th bit for parity
		USART2->CR2 |= 0x2000 ;	// 2 stop bits
	}
	if ( baudRate == 115200 )		// ASSAN DSM
	{
		GPIOD->BSRRL = 0x0010 ;		// output enable
	}
	NVIC_SetPriority( USART2_IRQn, 2 ) ; // Quite high priority interrupt
  NVIC_EnableIRQ(USART2_IRQn);
}

struct t_sendingSport
{
	uint8_t *buffer ;
	uint32_t count ;
} SendingSportPacket ;


void x9dHubTxStart( uint8_t *buffer, uint32_t count )
{
	if ( FrskyTelemetryType == FRSKY_TEL_HUB )		// Hub
	{
		SendingSportPacket.buffer = buffer ;
		SendingSportPacket.count = count ;
		TelemetryTx.sportBusy = 1 ;
		USART2->CR1 |= USART_CR1_TXEIE ;
	}
}

uint32_t hubTxPending()
{
	return TelemetryTx.sportBusy ;
}

void com1Parity( uint32_t even )
{
	if ( even )
	{
		USART2->CR1 |= USART_CR1_PCE | USART_CR1_M ;
	}
	else
	{
		USART2->CR1 &= ~(USART_CR1_PCE | USART_CR1_M) ;
	}
}


void com2Parity( uint32_t even )
{
	if ( even )
	{
		USART3->CR1 |= USART_CR1_PCE | USART_CR1_M ;
	}
	else
	{
		USART3->CR1 &= ~(USART_CR1_PCE | USART_CR1_M) ;
	}
}

//uint16_t SportStartDebug ;

void x9dSPortTxStart( uint8_t *buffer, uint32_t count, uint32_t receive )
{
//SportStartDebug	+= 1 ;
	USART2->CR1 &= ~USART_CR1_TE ;

	SendingSportPacket.buffer = buffer ;
	SendingSportPacket.count = count ;
	GPIOD->BSRRL = 0x0010 ;		// output enable
	if ( receive == 0 )
	{
		USART2->CR1 &= ~USART_CR1_RE ;
	}
	USART2->CR1 |= USART_CR1_TXEIE | USART_CR1_TE ;	// Force an idle frame
}



uint16_t RxIntCount ;

extern "C" void USART2_IRQHandler()
{
  uint32_t status;
  uint8_t data;

	RxIntCount += 0x1000 ;

  status = USART2->SR ;

	if ( status & USART_SR_TXE )
	{
		if ( SendingSportPacket.count )
		{
			USART2->DR = *SendingSportPacket.buffer++ ;
			if ( --SendingSportPacket.count == 0 )
			{
				USART2->CR1 &= ~USART_CR1_TXEIE ;	// Stop Tx interrupt
				USART2->CR1 |= USART_CR1_TCIE ;	// Enable complete interrupt
			}
		}
		else
		{
			USART2->CR1 &= ~USART_CR1_TXEIE ;	// Stop Tx interrupt
		}
	}

	if ( ( status & USART_SR_TC ) && (USART2->CR1 & USART_CR1_TCIE ) )
	{
		USART2->CR1 &= ~USART_CR1_TCIE ;	// Stop Complete interrupt
		{
			GPIOD->BSRRH = PIN_SPORT_ON ;		// output disable
			TelemetryTx.sportCount = 0 ;
			TelemetryTx.sportBusy = 0 ;
			USART2->CR1 |= USART_CR1_RE ;
		}
	}

  while (status & (USART_FLAG_RXNE | USART_FLAG_ERRORS))
	{
    data = USART2->DR;

    if (!(status & USART_FLAG_ERRORS))
		{
			RxIntCount += 1 ;
			put_fifo128( &Com1_fifo, data ) ;
			if ( FrskyTelemetryType == FRSKY_TEL_SPORT )		// SPORT
			{
				if ( LastReceivedSportByte == 0x7E && TelemetryTx.sportCount > 0 && data == TelemetryTx.SportTx.index )
				{
					x9dSPortTxStart( TelemetryTx.SportTx.ptr, TelemetryTx.sportCount, 0 ) ;
      		TelemetryTx.SportTx.index = 0x7E ;
				}
				LastReceivedSportByte = data ;
			}
		}
		else
		{
			if ( status & USART_FLAG_FE )
			{
				USART_FE += 1 ;
				if ( data )
				{ // A 0 byte is a line break
					put_fifo128( &Com1_fifo, data ) ;
				}
			}
			else
			{
				put_fifo128( &Com1_fifo, data ) ;
			}
			USART_ERRORS += 1 ;
			if ( status & USART_FLAG_ORE )
			{
				USART_ORE += 1 ;
			}
			if ( status & USART_FLAG_NE )
			{
				USART_NE += 1 ;
			}
			if ( status & USART_FLAG_PE )
			{
				USART_PE += 1 ;
			}
		}
    status = USART2->SR ;
  }
}


uint16_t rxTelemetry()
{
	return get_fifo128( &Com1_fifo ) ;
}

void txmit( uint8_t c )
{
	/* Wait for the transmitter to be ready */
  while ( (USART3->SR & USART_SR_TXE) == 0 ) ;

  /* Send character */
	USART3->DR = c ;
}

uint32_t txPdcCom2( struct t_serial_tx *data )
{
	data->ready = 1 ;
	Current_Com2 = data ;
	USART3->CR1 |= USART_CR1_TXEIE ;
	return 1 ;			// Sent OK
}

extern "C" void USART3_IRQHandler()
{
  uint32_t status;
  uint8_t data;
	USART_TypeDef *puart = USART3 ;

  status = puart->SR ;

	if ( ( status & USART_SR_TXE ) && (puart->CR1 & USART_CR1_TXEIE ) )
	{
		if ( Current_Com2 )
		{
			if ( Current_Com2->size )
			{
				puart->DR = *Current_Com2->buffer++ ;
				if ( --Current_Com2->size == 0 )
				{
					puart->CR1 &= ~USART_CR1_TXEIE ;	// Stop Tx interrupt
					puart->CR1 |= USART_CR1_TCIE ;	// Enable complete interrupt
				}
			}
			else
			{
				puart->CR1 &= ~USART_CR1_TXEIE ;	// Stop Tx interrupt
			}
		}
	}

	if ( ( status & USART_SR_TC ) && (puart->CR1 & USART_CR1_TCIE ) )
	{
		puart->CR1 &= ~USART_CR1_TCIE ;	// Stop Complete interrupt
		Current_Com2->ready = 0 ;
	}

  while (status & (USART_FLAG_RXNE | USART_FLAG_ERRORS))
	{
    data = puart->DR ;
    if (!(status & USART_FLAG_ERRORS))
		{
			if ( ( g_model.com2Function == COM2_FUNC_SBUSTRAIN ) || ( g_model.com2Function == COM2_FUNC_SBUS57600 ) )
			{
				put_fifo64( &Sbus_fifo, data ) ;
			}
			else
			{
				put_fifo128( &Com2_fifo, data ) ;
			}
		}
		else
		{
			if ( status & USART_FLAG_ORE )
			{
				USART_ORE += 1 ;
			}
		}
    status = puart->SR ;
	}
}

void start_timer11()
{

	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN ;		// Enable clock
	TIM11->PSC = (PeripheralSpeeds.Peri2_frequency*PeripheralSpeeds.Timer_mult2) / 2000000 - 1 ;		// 0.5uS
	TIM11->CCER = 0 ;
	TIM11->DIER = 0 ;
	TIM11->CCMR1 = 0 ;

	TIM11->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 ;

	TIM11->CR1 = TIM_CR1_CEN ;

	NVIC_SetPriority( TIM1_TRG_COM_TIM11_IRQn, 14 ) ; // Low priority interrupt
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn) ;
}

void stop_timer11()
{
	TIM11->CR1 = 0 ;
	NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn) ;
}

// Handle software serial on COM1 input (for non-inverted input)
void init_software_com1(uint32_t baudrate, uint32_t invert, uint32_t parity )
{
	struct t_softSerial *pss = &SoftSerial1 ;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN ;		// Enable clock

	pss->bitTime = 2000000 / baudrate ;
	pss->softSerialEvenParity = parity ? 1 : 0 ;

	pss->softSerInvert = invert ? 0 : GPIO_Pin_6 ;	// Port D6
	pss->pfifo = &Com1_fifo ;

	pss->lineState = LINE_IDLE ;
	CaptureMode = CAP_COM1 ;

#define EXT_BIT_MASK	0x00000040
	SYSCFG->EXTICR[1] |= 0x0300 ;
	EXTI->IMR |= EXT_BIT_MASK ;
	EXTI->RTSR |= EXT_BIT_MASK ;
	EXTI->FTSR |= EXT_BIT_MASK ;

	configure_pins( GPIO_Pin_6, PIN_INPUT | PIN_PORTD ) ;
	NVIC_SetPriority( EXTI9_5_IRQn, 0 ) ; // Highest priority interrupt
	NVIC_EnableIRQ( EXTI9_5_IRQn) ;

	start_timer11() ;
}

void disable_software_com1()
{
	CaptureMode = CAP_PPM ;
	stop_timer11() ;
	EXTI->IMR &= ~EXT_BIT_MASK ;
}


extern "C" void EXTI9_5_IRQHandler()
{
  register uint32_t capture ;
  register uint32_t dummy ;
	struct t_softSerial *pss = &SoftSerial1 ;

	capture =  TIM7->CNT ;	// Capture time

	if ( EXTI->PR & XJT_HEARTBEAT_BIT )
	{
		XjtHeartbeatCapture.value = capture ;
		EXTI->PR = XJT_HEARTBEAT_BIT ;
	}
	if ( ( EXTI->PR & EXT_BIT_MASK ) == 0 )
	{
		return ;
	}
	EXTI->PR = EXT_BIT_MASK ;

	dummy = GPIOD->IDR ;
	if ( ( dummy & GPIO_Pin_6 ) == pss->softSerInvert )
	{
		// L to H transition
		pss->LtoHtime = capture ;
		TIM11->CNT = 0 ;
		TIM11->CCR1 = pss->bitTime * 12 ;
		uint32_t time ;
		capture -= pss->HtoLtime ;
		time = capture ;
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
			putCaptureTime( pss, 0, 3 ) ;
		}
		else
		{
			uint32_t time ;
			capture -= pss->LtoHtime ;
			time = capture ;
			putCaptureTime( pss, time, 1 ) ;
		}
		TIM11->DIER = 0 ;
		TIM11->CCR1 = pss->bitTime * 20 ;
		TIM11->CNT = 0 ;
		TIM11->SR = 0 ;
	}
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler()
{
	uint32_t status ;
	struct t_softSerial *pss = &SoftSerial1 ;

	status = TIM11->SR ;
	if ( status & TIM_SR_CC1IF )
	{
		uint32_t time ;
		time = TIM7->CNT - pss->LtoHtime ;
		putCaptureTime( pss, time, 2 ) ;
		pss->lineState = LINE_IDLE ;
		TIM11->DIER = 0 ;
		TIM11->SR = 0 ;
	}

}

//#endif // nX12D
