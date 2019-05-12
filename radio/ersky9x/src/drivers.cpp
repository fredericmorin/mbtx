/*
 * Author - Erez Raviv <erezraviv@gmail.com>
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

#ifdef PCBSKY
 #ifndef PCBDUE
  #include "AT91SAM3S4.h"
 #else
	#include "sam3x8e.h"
 #endif
#ifndef SIMU
#include "core_cm3.h"
#endif
#endif

#include "ersky9x.h"
#include "myeeprom.h"
#include "drivers.h"
#include "logicio.h"
#include "pulses.h"
#include "lcd.h"
#include "debug.h"
#include "frsky.h"
#ifndef SIMU
#include "CoOS.h"
#endif

#if defined(PCBX9D) || defined(PCB9XT)
#include "diskio.h"
#include "X9D/stm32f2xx.h"
#include "X9D/stm32f2xx_gpio.h"
#include "X9D/stm32f2xx_rcc.h"
#include "X9D/stm32f2xx_usart.h"
#include "X9D/hal.h"
#include "timers.h"
#endif

#if defined(PCBX12D) || defined(PCBX10)
#include "X12D/stm32f4xx.h"
#include "X12D/stm32f4xx_usart.h"
//#include "X12D/stm32f4xx_gpio.h"
#include "X12D/stm32f4xx_rcc.h"
#include "X12D/hal.h"
extern void wdt_reset() ;
#define wdt_reset()
#endif

#ifdef BLUETOOTH
#include "bluetooth.h"
#endif

#define SERIAL_TRAINER	1

//#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)

// Timer usage
// TIMER3 for input capture
// Timer4 to provide 0.5uS clock for input capture
// TIMER0 at full speed (MCK/2) for delay timing
// TIMER2 at 200Hz, provides 5mS for sound and 10mS tick on interrupt
// Timer1 used for DAC output timing
// Timer5 is currently UNUSED

uint8_t Scc_baudrate ;				// 0 for 125000, 1 for 115200
//uint8_t LastReceivedSportByte ;

uint16_t Temperature ;				// Raw temp reading
uint16_t Max_temperature ;		// Max raw temp reading
uint16_t WatchdogTimeout ;
extern uint8_t PulsesPaused ;

//uint8_t JetiTxBuffer[16] ;

struct t_fifo128 Com1_fifo ;
struct t_fifo128 Com2_fifo ;

#if defined(LUA) || defined(BASIC)
struct t_fifo128 Script_fifo ;
#endif
struct t_fifo64 Sbus_fifo ;

struct t_telemetryTx TelemetryTx ;

struct t_serial_tx *Current_Com1 ;
struct t_serial_tx *Current_Com2 ;

volatile uint8_t Spi_complete ;

void putEvent( register uint8_t evt) ;
void per10ms( void ) ;
uint8_t getEvent( void ) ;
void pauseEvents(uint8_t event) ;
void killEvents(uint8_t event) ;



void UART_Configure( uint32_t baudrate, uint32_t masterClock) ;
void txmit( uint8_t c ) ;
uint16_t rxCom2( void ) ;
void UART3_Configure( uint32_t baudrate, uint32_t masterClock) ;

uint32_t keyState( enum EnumKeys enuk) ;
void init_spi( void ) ;
uint32_t eeprom_read_status( void ) ;
uint32_t  eeprom_write_one( uint8_t byte, uint8_t count ) ;
void eeprom_write_enable( void ) ;
uint32_t spi_operation( uint8_t *tx, uint8_t *rx, uint32_t count ) ;
uint32_t spi_PDC_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count ) ;

void read_adc(void ) ;
void init_adc( void ) ;
void init_ssc( uint16_t baudrate ) ;
void disable_ssc( void ) ;

/** Usart Hw interface used by the console (UART0). */
#define CONSOLE_USART       UART0
/** Usart Hw ID used by the console (UART0). */
#define CONSOLE_ID          ID_UART0
/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define CONSOLE_PINS        {PINS_UART}

/** Second serial baudrate 9600. */
#define SECOND_BAUDRATE    9600
/** Usart Hw interface used by the console (UART0). */
#define SECOND_USART       USART0
/** Usart Hw ID used by the console (UART0). */
#define SECOND_ID          ID_USART0
/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define SECOND_PINS        {PINS_USART0}

extern uint8_t ExternalKeys ;
uint16_t ExternalSwitches ;
uint8_t ExternalSwitchesValid ;
uint8_t ExternalSwitchByte1 ;
extern uint8_t ExternalSet ;

static uint8_t s_evt;
void putEvent( register uint8_t evt)
{
  s_evt = evt;
	Tevent = evt ;
}

uint8_t menuPressed()
{
	if ( keys[KEY_MENU].isKilled() )
	{
		return 0 ;
	}
	return ( read_keys() & 2 ) == 0 ;
}

uint8_t encoderPressed()
{
	if ( keys[BTN_RE].isKilled() )
	{
		return 0 ;
	}
	return keys[BTN_RE].state() ;
}


uint8_t getEvent()
{
  register uint8_t evt = s_evt;
  s_evt=0;
  return evt;
}

Key keys[NUM_KEYS] ;

void Key::input(bool val, EnumKeys enuk)
{
  //  uint8_t old=m_vals;
  m_vals <<= 1;  if(val) m_vals |= 1; //portbit einschieben
  m_cnt++;

  if(m_state && m_vals==0){  //gerade eben sprung auf 0
    if(m_state!=KSTATE_KILLED) {
      putEvent(EVT_KEY_BREAK(enuk));
#ifdef KSTATE_RPTDELAY
      if(!( m_state == KSTATE_RPTDELAY && m_cnt<20)){
#else
      if(!( m_state == 16 && m_cnt<16)){
#endif
        m_dblcnt=0;
      }
        //      }
    }
    m_cnt   = 0;
    m_state = KSTATE_OFF;
  }
  switch(m_state){
    case KSTATE_OFF:
      if(m_vals==FFVAL){ //gerade eben sprung auf ff
        m_state = KSTATE_START;
        if(m_cnt>20) m_dblcnt=0; //pause zu lang fuer double
        m_cnt   = 0;
      }
			else
			{
				if( m_vals == 0 )
				{
	        if(m_cnt>30) m_dblcnt=0; //pause zu lang fuer double
				}
			}
      break;
      //fallthrough
    case KSTATE_START:
      putEvent(EVT_KEY_FIRST(enuk));
      m_dblcnt++;
#ifdef KSTATE_RPTDELAY
      m_state   = KSTATE_RPTDELAY;
#else
      m_state   = 16;
#endif
      m_cnt     = 0;
      break;
#ifdef KSTATE_RPTDELAY
    case KSTATE_RPTDELAY: // gruvin: longer delay before first key repeat
      if(m_cnt == 38) putEvent(EVT_KEY_LONG(enuk)); // need to catch this inside RPTDELAY time
      if (m_cnt == 40) {
        m_state = 16;
        m_cnt = 0;
      }
      break;
#endif
    case 16:
#ifndef KSTATE_RPTDELAY
      if(m_cnt == 38) putEvent(EVT_KEY_LONG(enuk));
      //fallthrough
#endif
    case 8:
    case 4:
    case 2:
      if(m_cnt >= 48)  { //3 6 12 24 48 pulses in every 480ms
        m_state >>= 1;
        m_cnt     = 0;
      }
      //fallthrough
    case 1:
      if( (m_cnt & (m_state-1)) == 0)  putEvent(EVT_KEY_REPT(enuk));
      break;

    case KSTATE_PAUSE: //pause
      if(m_cnt >= 64)      {
        m_state = 8;
        m_cnt   = 0;
      }
      break;

    case KSTATE_KILLED: //killed
      break;
  }
}

void pauseEvents(uint8_t event)
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  keys[event].pauseEvents();
}

void killEvents(uint8_t event)
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  keys[event].killEvents();
}

uint8_t getEventDbl(uint8_t event)
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  return keys[event].getDbl();
  return 0;
}

volatile uint8_t  g_blinkTmr10ms;

volatile uint16_t g_tmr10ms ;
volatile uint32_t g_ltmr10ms ;
extern uint8_t StickScrollTimer ;

//#ifdef PCBSKY
//struct t_serial_tx FmsTx ;
//uint8_t FmsTxBuffer[12] ;

//void doFms()
//{
//	uint32_t i ;
//	if ( g_tmr10ms & 1 )
//	{
//		// Send a Fms packet
//		FmsTxBuffer[0] = 0xFF ;
//		for ( i = 0 ; i < 8 ; i += 1 )
//		{
//			int32_t x = g_chans512[i] ;
//			x /= 12 ;
//			x += 128 ;
//		FmsTxBuffer[i+1] = x ;
//		}
//		FmsTx.size = 9 ;
//		FmsTx.buffer = FmsTxBuffer ;
//		txPdcCom2( &FmsTx ) ;
//	}
//}
//#endif

#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX9D)
#ifndef PCBX7
#ifndef PCBXLITE
#ifndef PCBX9LITE
struct t_serial_tx LcdDumpBuf ;

void doLcdDump()
{
#ifdef PCBX9D
#define X9D_EXTRA		84
extern uint8_t ExtDisplayBuf[1024 + X9D_EXTRA*8 + 2] ;
#else
extern uint8_t ExtDisplayBuf[DISPLAY_W*DISPLAY_H/8 + 2] ;
#endif
extern uint8_t ExtDisplaySend ;
	if ( ExtDisplaySend )
	{
		ExtDisplaySend = 0 ;
		LcdDumpBuf.buffer = ExtDisplayBuf ;
		LcdDumpBuf.size = sizeof(ExtDisplayBuf) ;
#ifdef PCBX9D
		txPdcCom2( &LcdDumpBuf ) ;
#else
#ifdef BLUETOOTH
		if ( g_model.BTfunction == BT_LCDDUMP )
		{
//extern uint8_t BtReady ;
			if ( BtControl.BtReady )
			{
				txPdcBt( &LcdDumpBuf ) ;
			}
		}
#endif
#ifndef PCB9XT
#ifdef BLUETOOTH
		else
#endif
		{
			txPdcCom2( &LcdDumpBuf ) ;
		}
#endif
#endif
	}
}
#endif // PCBX9LITE
#endif // PCBXLITE
#endif // PCBX7

#endif

#ifdef PCBX7
struct t_serial_tx LcdDumpBuf ;

void doLcdDump()
{
extern uint8_t ExtDisplayBuf[DISPLAY_W*DISPLAY_H/8 + 2] ;
extern uint8_t ExtDisplaySend ;
	if ( ExtDisplaySend )
	{
		ExtDisplaySend = 0 ;
		LcdDumpBuf.buffer = ExtDisplayBuf ;
		LcdDumpBuf.size = sizeof(ExtDisplayBuf) ;
		if ( g_model.BTfunction == BT_LCDDUMP )
		{
//extern uint8_t BtReady ;
#ifdef BLUETOOTH
			if ( BtControl.BtReady )
			{
				txPdcBt( &LcdDumpBuf ) ;
			}
#endif
		}
	}
}
#endif // PCBX7

#ifdef PCBX9LITE
struct t_serial_tx LcdDumpBuf ;

void doLcdDump()
{
extern uint8_t ExtDisplayBuf[DISPLAY_W*DISPLAY_H/8 + 2] ;
extern uint8_t ExtDisplaySend ;
	if ( ExtDisplaySend )
	{
		ExtDisplaySend = 0 ;
		LcdDumpBuf.buffer = ExtDisplayBuf ;
		LcdDumpBuf.size = sizeof(ExtDisplayBuf) ;
		if ( g_model.com2Function == COM2_FUNC_LCD )
		{
			txPdcCom2( &LcdDumpBuf ) ;
		}
	}
}
#endif // PCBX9LITE


//static uint8_t LcdDumpState ;
//#define LCD_DUMP_UNSYNC 	0
//#define LCD_DUMP_FIRST		1
//#define LCD_DUMP_2ND			2
//#define LCD_DUMP_3RD			3

void per10ms()
{
	register uint32_t i ;

  g_tmr10ms++ ;
	g_ltmr10ms += 1 ;
  if (WatchdogTimeout)
	{
  	if (WatchdogTimeout>300)
		{
  		WatchdogTimeout = 300 ;
		}
    WatchdogTimeout -= 1;
    wdt_reset();  // Retrigger hardware watchdog
  }

  g_blinkTmr10ms++;
	if ( ExternalSet )
	{
		if ( --ExternalSet == 0 )
		{
			ExternalSwitchesValid = 0 ;
		}
	}
  uint8_t enuk = KEY_MENU;
  uint8_t    in = ~read_keys() ;
	// Bits 3-6 are down, up, right and left
	// Try to only allow one at a
#ifdef REVX
	static uint8_t current ;
	uint8_t dir_keys ;
	uint8_t lcurrent ;

	dir_keys = in & 0x78 ;		// Mask to direction keys
	if ( ( lcurrent = current ) )
	{ // Something already pressed
		if ( ( lcurrent & dir_keys ) == 0 )
		{
			lcurrent = 0 ;	// No longer pressed
		}
		else
		{
			in &= lcurrent | 0x06 ;	// current or MENU or EXIT allowed
		}
	}
	if ( lcurrent == 0 )
	{ // look for a key
		if ( dir_keys & 0x20 )	// right
		{
			lcurrent = 0x60 ;		// Allow L and R for 9X
		}
		else if ( dir_keys & 0x40 )	// left
		{
			lcurrent = 0x60 ;		// Allow L and R for 9X
		}
		else if ( dir_keys & 0x08 )	// down
		{
			lcurrent = 0x08 ;
		}
		else if ( dir_keys & 0x10 )	// up
		{
			lcurrent = 0x10 ;
		}
		in &= lcurrent | 0x06 ;	// current or MENU or EXIT allowed
	}
	current = lcurrent ;
#endif

  for( i=1; i<7; i++)
  {
		uint8_t value = in & (1<<i) ;
#if !defined(SIMU)
		if ( value )
		{
			StickScrollTimer = STICK_SCROLL_TIMEOUT ;
		}
#endif
    //INP_B_KEY_MEN 1  .. INP_B_KEY_LFT 6
    keys[enuk].input(value,(EnumKeys)enuk);
    ++enuk;
  }

	in = read_trims() ;

	for( i=1; i<256; i<<=1)
  {
    // INP_D_TRM_RH_UP   0 .. INP_D_TRM_LH_UP   7
    keys[enuk].input(in & i,(EnumKeys)enuk);
    ++enuk;
  }

#ifdef PCBSKY
#if !defined(SIMU)
	uint8_t value = ~PIOB->PIO_PDSR & 0x40 ;

//extern uint8_t AnaEncSw ;
//	value |= AnaEncSw ;
	keys[enuk].input( value,(EnumKeys)enuk); // Rotary Enc. Switch
	if ( value )
	{
		StickScrollTimer = STICK_SCROLL_TIMEOUT ;
	}
#endif

#endif

#ifdef PCB9XT
extern uint16_t M64Switches ;
extern uint8_t EncoderI2cData[] ;
	uint8_t value = (M64Switches & 0x0200) ? 1 : 0 ;
	if ( value == 0 )
	{
		value = EncoderI2cData[1] ? 1 : 0 ;
	}
	keys[enuk].input( value,(EnumKeys)enuk); // Rotary Enc. Switch
	if ( value )
	{
		StickScrollTimer = STICK_SCROLL_TIMEOUT ;
	}
#endif

#ifdef PCBX9D
 #if !defined(SIMU)
  #ifdef PCBX7
   #ifdef PCBT12
	uint8_t value = 0 ;
   #else
	uint8_t value = (~GPIOE->IDR & PIN_BUTTON_ENCODER) ? 1 : 0 ;
   #endif
  #endif // PCBX7
  #ifdef REV9E
	uint8_t value = ~GPIOF->IDR & PIN_BUTTON_ENCODER ;
  #endif // REV9E
  #ifdef PCBX9LITE
	uint8_t value = (~GPIOE->IDR & PIN_BUTTON_ENCODER) ? 1 : 0 ;
//	uint8_t value = 0 ;
  #endif // X3
  #if defined(REVPLUS) || defined(REVNORM)
  #ifndef REV9E
extern uint8_t AnaEncSw ;
	uint8_t value = AnaEncSw ;
  #endif // nREV9E
  #endif // norm/plus
	#ifdef PCBXLITE
	uint8_t value = 0 ;
	#endif
	keys[enuk].input( value,(EnumKeys)enuk); // Rotary Enc. Switch
	if ( value )
	{
		StickScrollTimer = STICK_SCROLL_TIMEOUT ;
	}
 #endif // SIMU
#endif // X9D

#if defined(PCBX12D)
	uint8_t value = (~GPIOC->IDR & 0x0002) ? 1 : 0 ;
#endif // X12D
#if defined(PCBX10)
	uint8_t value = (~GPIOI->IDR & 0x0100) ? 1 : 0 ;
#endif // X10
#if defined(PCBX12D) || defined(PCBX10)
	keys[enuk].input( value,(EnumKeys)enuk); // Rotary Enc. Switch
	if ( value )
	{
		StickScrollTimer = STICK_SCROLL_TIMEOUT ;
	}
#endif // X12D

#if defined(PCBX9D) || defined(PCB9XT)
	sdPoll10mS() ;
#endif

#ifdef PCBSKY
//	if ( g_model.com2Function == COM2_FUNC_FMS )
//	{
//		doFms() ;
//	}
	if ( ( g_model.com2Function == COM2_FUNC_LCD ) || ( g_model.BTfunction == BT_LCDDUMP ) )
	{
		doLcdDump() ;
		if ( g_model.com2Function == COM2_FUNC_LCD )
		{
			int32_t y ;

			while ( ( y = get_fifo128( &Com2_fifo ) ) != -1 )
			{
				ExternalKeys = y ;
				ExternalSet = 50 ;
			}
//			int32_t y ;
//			while ( ( y = get_fifo128( &Com2_fifo ) ) != -1 )
//			{
//				switch ( LcdDumpState )
//				{
//					case LCD_DUMP_UNSYNC :
//						if ( y & 0x80 )
//						{
//							LcdDumpState = LCD_DUMP_FIRST ;
//							ExternalKeys = y & 0x7F ;
//							ExternalSet = 50 ;
//						}
//					break ;
//					case LCD_DUMP_FIRST :
//						ExternalSwitchByte1 = y ;
//						LcdDumpState = LCD_DUMP_2ND ;
//					break ;
//					case LCD_DUMP_2ND :
//						ExternalSwitches = ( y << 8 ) | ExternalSwitchByte1 ;
//						ExternalSwitchesValid = 1 ;
//						LcdDumpState = LCD_DUMP_UNSYNC ;
//					break ;
//				}
//			}
		}
	}
#endif
#ifdef PCB9XT
	if ( g_model.BTfunction == BT_LCDDUMP )
	{
		doLcdDump() ;
	}
#endif
#ifdef PCBX7
 #ifdef BLUETOOTH
	if ( g_model.BTfunction == BT_LCDDUMP )
	{
		doLcdDump() ;
	}
 #endif
#endif

#ifdef PCBX9LITE
	if ( g_model.com2Function == COM2_FUNC_LCD )
	{
		doLcdDump() ;
	}
#endif

#ifdef PCBX9D
#if defined(REVPLUS) || defined(REVNORM) || defined(REV9E)
	if ( g_model.com2Function == COM2_FUNC_LCD )
	{
		doLcdDump() ;
		int32_t y ;
		while ( ( y = get_fifo128( &Com2_fifo ) ) != -1 )
		{
extern uint8_t ExternalKeys ;
extern uint8_t ExternalSet ;
			ExternalKeys = y ;
			ExternalSet = 50 ;
		}
	}
#endif // norm/plus
#endif
}


void put_fifo64( struct t_fifo64 *pfifo, uint8_t byte )
{
  uint32_t next = (pfifo->in + 1) & 0x3f;
	if ( next != pfifo->out )
	{
		pfifo->fifo[pfifo->in] = byte ;
		pfifo->in = next ;
	}
}

int32_t get_fifo64( struct t_fifo64 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x3F ;
		return rxbyte ;
	}
	return -1 ;
}

void put_fifo128( struct t_fifo128 *pfifo, uint8_t byte )
{
  uint32_t next = (pfifo->in + 1) & 0x7f;
	if ( next != pfifo->out )
	{
		pfifo->fifo[pfifo->in] = byte ;
		pfifo->in = next ;
	}
}

uint32_t fifo128Space( struct t_fifo128 *pfifo )
{
	uint32_t space ;
	space = pfifo->out + 127 ;
	if ( pfifo->out > pfifo->in )
	{
		space -= 128 ;
	}
	return space - pfifo->in ;
}

int32_t get_fifo128( struct t_fifo128 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x7F ;
		return rxbyte ;
	}
	return -1 ;
}

int32_t peek_fifo128( struct t_fifo128 *pfifo )
{
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		return pfifo->fifo[pfifo->out] ;
	}
	return -1 ;
}

//#ifdef ACCESS
//void put_16bit_fifo64( struct t_16bit_fifo64 *pfifo, uint16_t word )
//{
//  uint32_t next = (pfifo->in + 1) & 0x3f;
//	if ( next != pfifo->out )
//	{
//		pfifo->fifo[pfifo->in] = word ;
//		pfifo->in = next ;
//	}
//}

//int32_t get_16bit_fifo64( struct t_16bit_fifo64 *pfifo )
//{
//	int32_t rxbyte ;
//	if ( pfifo->in != pfifo->out )				// Look for char available
//	{
//		rxbyte = pfifo->fifo[pfifo->out] ;
//		pfifo->out = ( pfifo->out + 1 ) & 0x3F ;
//		return rxbyte ;
//	}
//	return -1 ;
//}
//#endif

#ifdef REVX
void put_16bit_fifo32( struct t_16bit_fifo32 *pfifo, uint16_t word )
{
  uint32_t next = (pfifo->in + 1) & 0x1f;
	if ( next != pfifo->out )
	{
		pfifo->fifo[pfifo->in] = word ;
		pfifo->in = next ;
	}
}

int32_t get_16bit_fifo32( struct t_16bit_fifo32 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x1F ;
		return rxbyte ;
	}
	return -1 ;
}
#endif

uint16_t rxCom2()
{
	return get_fifo128( &Com2_fifo ) ;
}

//#ifdef BLUETOOTH
//int32_t rxBtuart()
//{
//#ifdef BT_PDC
//	return rxPdcBt() ;
//#else
//	return get_fifo128( &BtRx_fifo ) ;
//#endif
//}
//#endif


#ifdef SERIAL_TRAINER
// 9600 baud, bit time 104.16uS
#define BIT_TIME_9600		208
// 19200 baud, bit time 52.08
#define BIT_TIME_19200	104
// 38400 baud, bit time 26.04
#define BIT_TIME_38400	52
// 57600 baud, bit time 17.36uS
#define BIT_TIME_57600	35
// 100000 baud, bit time 10uS (SBUS)
#define BIT_TIME_100K		20
// 115200 baud, bit time 8.68uS
#define BIT_TIME_115K		17

// States in LineState
#define LINE_IDLE			0
#define LINE_ACTIVE		1

// States in BitState
#define BIT_IDLE			0
#define BIT_ACTIVE		1
#define BIT_FRAMING		2

struct t_softSerial SoftSerial1 ;
struct t_softSerial SoftSerial2 ;

//uint8_t LineState ;
uint8_t CaptureMode ;
//uint16_t BitTime ;
//uint16_t HtoLtime ;
//uint16_t LtoHtime ;
//uint16_t Byte ;
//uint8_t SoftSerInvert = 0 ;
//uint8_t BitState ;
//uint8_t BitCount ;
//uint8_t Tc5Count ;
//uint8_t SoftSerialEvenParity ;

//uint16_t USART_ERRORS ;
//uint16_t USART_ORE ;
//uint16_t USART_NE ;
//uint16_t USART_FE ;
extern uint16_t USART_PE ;

// time in units of 0.5uS, value is 1 or 0
void putCaptureTime( struct t_softSerial *pss, uint16_t time, uint32_t value )
{
	time += pss->bitTime/2 ;
	time /= pss->bitTime ;		// Now number of bits
	if ( value == 3 )
	{
		return ;
	}

	if ( pss->bitState == BIT_IDLE )
	{ // Starting, value should be 0
		pss->bitState = BIT_ACTIVE ;
		pss->bitCount = 0 ;
		if ( time > 1 )
		{
			pss->byte >>= time-1 ;
			pss->bitCount = time-1 ;
		}
	}
	else
	{
		if ( value )
		{
			uint32_t len = pss->softSerialEvenParity ? 9 : 8 ;
			while ( time )
			{
				if ( pss->bitCount >= len )
				{ // Got a byte
					if ( len == 9 )
					{
						// check parity (even)
						uint32_t parity = pss->byte ;
						parity ^= parity >> 4 ;
						parity ^= parity >> 2 ;
						parity ^= parity >> 1 ;
						parity ^= pss->byte >> 8 ;
						if ( ( parity & 1 ) == 0 )
						{
							if ( CaptureMode == CAP_SERIAL )
							{
								put_fifo64( &Sbus_fifo, pss->byte ) ;
							}
							else
							{
								if ( pss->pfifo )
								{
									put_fifo128( pss->pfifo, pss->byte ) ;
								}
							}
						}
						else
						{
							USART_PE += 1 ;
						}
					}
					else
					{
						if ( pss->pfifo )
						{
							put_fifo128( pss->pfifo, pss->byte ) ;
						}
					}
					pss->bitState = BIT_IDLE ;
					time = 0 ;
					break ;
				}
				else
				{
					pss->byte >>= 1 ;
					pss->byte |= ( len == 9 ) ? 0x100 : 0x80 ;
					time -= 1 ;
					pss->bitCount += 1 ;
				}
			}
		}
		else
		{
			pss->byte >>= time ;
			pss->bitCount += time ;
		}
	}
}

#endif


#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)


#if !defined(SIMU)


// Start TIMER7 at 2000000Hz
void start_2Mhz_timer()
{
	// Now for timer 7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN ;		// Enable clock

	TIM7->ARR = 0xFFFF ;
	TIM7->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM7->CR2 = 0 ;
	TIM7->CR2 = 0x20 ;
	TIM7->CR1 = TIM_CR1_CEN ;
}

#endif	// SIMU


#ifndef PCBX12D
 #ifdef PCBX9D
  #ifndef PCBXLITE
   #ifndef PCBX9LITE
    #ifndef PCBX10
#define XJT_HEARTBEAT_BIT	0x0080		// PC7


struct t_XjtHeartbeatCapture XjtHeartbeatCapture ;
struct t_MultiHeartbeatCapture MultiHeartbeatCapture = {0,0,0} ;

void init_xjt_heartbeat()
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN ;		// Enable clock
	SYSCFG->EXTICR[1] |= 0x2000 ;		// PC7
	EXTI->RTSR |= XJT_HEARTBEAT_BIT ;	// Falling Edge
	EXTI->IMR |= XJT_HEARTBEAT_BIT ;
	configure_pins( GPIO_Pin_7, PIN_INPUT | PIN_PORTC ) ;
	NVIC_SetPriority( EXTI9_5_IRQn, 0 ) ; // Highest priority interrupt
	NVIC_EnableIRQ( EXTI9_5_IRQn) ;
	XjtHeartbeatCapture.valid = 1 ;
}

void stop_xjt_heartbeat()
{
	EXTI->IMR &= ~XJT_HEARTBEAT_BIT ;
	XjtHeartbeatCapture.valid = 0 ;
}
		#endif
   #endif // n PCBX9LITE
  #endif // n PCBXLITE
 #endif // PCBX9D
#endif // n PCBX12D


#endif // X9D || SP


uint32_t sportPacketSend( uint8_t *pdata, uint8_t index )
{
	uint32_t i ;
	uint32_t j ;
	uint32_t crc ;
	uint32_t byte ;

	if ( pdata == 0 )	// Test for buffer available
	{
		return TelemetryTx.sportCount ? 0 : 1 ;
	}

	if ( TelemetryTx.sportCount )
	{
		return 0 ;	// Can't send, packet already queued
	}
	crc = 0 ;
	j = 0 ;
	for ( i = 0 ; i < 8 ; i += 1 )
	{
		byte = *pdata++ ;
		if ( i == 7 )
		{
			byte = 0xFF-crc ;
		}
		crc += byte ;
		crc += crc >> 8 ;
		crc &= 0x00FF ;
		if ( ( byte == 0x7E ) || ( byte == 0x7D ) )
		{
			TelemetryTx.SportTx.data[j++] = 0x7D ;
			byte &= ~0x20 ;
		}
		TelemetryTx.SportTx.data[j++] = byte ;
	}
	TelemetryTx.SportTx.ptr = TelemetryTx.SportTx.data ;
	TelemetryTx.SportTx.index = index ;
	TelemetryTx.sportCount = j ;
	return 1 ;
}

uint32_t xfirePacketSend( uint8_t length, uint8_t command, uint8_t *data )
{
	uint32_t i ;

	if ( length == 0 )	// Test for buffer available
	{
		return TelemetryTx.XfireTx.count ? 0 : 1 ;
	}

	if ( TelemetryTx.XfireTx.count )
	{
		return 0 ;	// Can't send, packet already queued
	}
	TelemetryTx.XfireTx.command = command ;
//	j = TelemetryTx.XfireTx.count ;
	if ( length > 64 )
	{
		return 0 ;	// Can't send, packet too long
	}
	for ( i = 0 ; i < length ; i += 1 )
	{
		TelemetryTx.XfireTx.data[i] = *data++ ;
	}
	TelemetryTx.XfireTx.count = length ;
	return 1 ;
}
