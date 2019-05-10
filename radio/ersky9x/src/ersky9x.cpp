
/****************************************************************************
*  Copyright (c) 2011 by Michael Blandford. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED ARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
****************************************************************************/
#define __ERSKY9X_CPP__

#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

#ifdef PCBSKY
#include "AT91SAM3S4.h"
#ifndef SIMU
#include "core_cm3.h"
#endif
#endif

//#define LATENCY 1

#include "ersky9x.h"
#include "myeeprom.h"
#include "audio.h"
#include "sound.h"
#include "lcd.h"
#include "drivers.h"

#ifdef PCBSKY
#include "file.h"
#endif

#include "menus.h"
#include "mixer.h"
#include "timers.h"
#if defined(PCBX12D) || defined(PCBX10)
#include "X12D/stm32f4xx_gpio.h"
#endif
#include "logicio.h"
#include "pulses.h"
#include "stringidx.h"


#include "frsky.h"

#ifdef PCBX9D
#include "analog.h"
#include "diskio.h"
#include "X9D/eeprom_rlc.h"
#include "X9D/stm32f2xx.h"
#include "X9D/stm32f2xx_gpio.h"
#include "X9D/stm32f2xx_rcc.h"
#include "X9D/hal.h"
#include "X9D/i2c_ee.h"

#include "X9D/usb_dcd_int.h"
#include "X9D/usb_bsp.h"
#include "X9D/usbd_conf.h"

extern "C" uint8_t USBD_HID_SendReport(USB_OTG_CORE_HANDLE  *pdev,
                                 uint8_t *report,
                                 uint16_t len) ;

#endif // PCBX9D


#ifdef  LUA
#include "lua/lua_api.h"
#endif
#ifdef  BASIC
#include "basic/basic.h"
#endif

#include "sbus.h"

#include "ff.h"
#include "maintenance.h"


#ifndef SIMU
#include "CoOS.h"
#endif

#include "../../common/hand.lbm"

//#define STARTUP_DEBUG 1
//#define STACK_PROBES	1

#if defined(PCBX12D) || defined(PCBX10)
 #define STACK_EXTRA	100
#else
 #define STACK_EXTRA	0
#endif

#ifndef SIMU
#ifdef LUA
#define MAIN_STACK_SIZE		(1400 + STACK_EXTRA)
#else
#ifdef BASIC
//#define MAIN_STACK_SIZE		2000
//#define MAIN_STACK_SIZE		1400
#define MAIN_STACK_SIZE		(660 + STACK_EXTRA)
#else
#define MAIN_STACK_SIZE		(500 + STACK_EXTRA)
#endif
#endif
#ifdef BLUETOOTH
#define BT_STACK_SIZE			(100 + STACK_EXTRA)
#endif
#define LOG_STACK_SIZE		(350 + STACK_EXTRA)
#define DEBUG_STACK_SIZE	(300 + STACK_EXTRA)
#define VOICE_STACK_SIZE	(130+200 + STACK_EXTRA)

#define CHECKRSSI		1

OS_TID MainTask;
OS_STK main_stk[MAIN_STACK_SIZE] ;

OS_TID LogTask;
OS_STK Log_stk[LOG_STACK_SIZE] ;
OS_TID VoiceTask;
OS_STK voice_stk[VOICE_STACK_SIZE] ;

#ifdef	DEBUG
OS_TID DebugTask;
OS_STK debug_stk[DEBUG_STACK_SIZE] ;
#endif

#ifdef SERIAL_HOST
void host( void* pdata ) ;
#define HOST_STACK_SIZE	300
OS_TID HostTask ;
OS_STK Host_stk[HOST_STACK_SIZE] ;
uint8_t Host10ms ;

#endif

#endif

#ifdef PCBX9LITE
void ledBlue( void ) ;
#endif

#ifdef USB_JOYSTICK
extern "C" void startJoystick(void) ;
extern "C" uint8_t HIDDJoystickDriver_Change( uint8_t *data ) ;
extern "C" void USBD_Connect(void) ;
extern "C" void USBD_Disconnect(void) ;
#endif

void processSwitches( void ) ;
uint32_t check_power_or_usb( void ) ;

#ifdef BASIC
uint8_t ScriptFlags ;
#endif

extern uint8_t TrainerPolarity ;

uint32_t IdleCount ;
uint32_t IdlePercent ;
uint32_t BasicExecTime ;

#ifdef PCBSKY
uint8_t HwDelayScale = 1 ;
#endif

uint8_t UserTimer1 ;


const char * const *Language = English ;

// const uint8_t splashdata[] = { 'S','P','S',0,
// #ifdef PCBX9LITE
// #include "FrSplash.lbm"
// #else
// #include "sTxsplash.lbm"
// #endif
// 	'S','P','E',0};

#include "debug.h"

t_time Time ;

uint8_t unexpectedShutdown = 0;
uint8_t SdMounted = 0;
uint8_t SectorsPerCluster ;
#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
uint8_t CurrentTrainerSource ;
#endif
uint8_t HardwareMenuEnabled = 0 ;

#define SW_STACK_SIZE	6
uint8_t Last_switch[NUM_SKYCSW] ;
uint8_t Now_switch[NUM_SKYCSW] ;
int16_t CsTimer[NUM_SKYCSW] ;
int8_t SwitchStack[SW_STACK_SIZE] ;

uint8_t MuteTimer ;

int8_t NumExtraPots ;
uint8_t ExtraPotBits;

uint16_t AnalogData[ANALOG_DATA_SIZE] ;


extern uint16_t g_timeMain;
extern uint16_t g_timeRfsh ;
extern uint16_t g_timeMixer ;

volatile int32_t Rotary_position ;
volatile int32_t Rotary_count ;
int32_t LastRotaryValue ;
int32_t Rotary_diff ;
uint8_t Vs_state[NUM_SKYCHNOUT+NUM_VOICE+EXTRA_SKYCHANNELS] ;

struct t_NvsControl
{
	uint8_t nvs_state ;
	uint8_t nvs_delay ;
	int16_t nvs_timer ;
	int16_t nvs_last_value ;
} NvsControl[NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS + NUM_GLOBAL_VOICE_ALARMS] ;
uint8_t CurrentVolume ;
uint8_t HoldVolume ;
int8_t RotaryControl ;
uint8_t ppmInValid = 0 ;
uint8_t Activated = 0 ;
uint8_t Tevent ;
extern uint16_t SbusTimer ;

struct t_MusicSwitches MusicSwitches ;

void log_task(void* pdata) ;
void main_loop( void* pdata ) ;
void mainSequence( uint32_t no_menu ) ;
void doSplash( void ) ;
void perMain( uint32_t no_menu ) ;
static void processVoiceAlarms( void ) ;
void txmit( uint8_t c ) ;
void uputs( char *string ) ;
uint16_t rxCom2( void ) ;

uint16_t SixPositionTable[5] ;

#ifdef PCBSKY
uint16_t anaIn( uint8_t chan ) ;
#endif
void getADC_single( void ) ;
void getADC_osmp( void ) ;
void getADC_filt( void ) ;
#ifdef PCBSKY
void read_adc( void ) ;
void init_adc( void ) ;
#endif

#ifdef PCBX9D
void check6pos( void ) ;
#endif

void check_backlight( void ) ;

static uint8_t checkTrim(uint8_t event) ;

void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att) ;
void putsChn(uint8_t x,uint8_t y,uint8_t idx1,uint8_t att) ;
void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att) ;//, bool nc) ;
const char *get_switches_string( void ) ;
bool getSwitch(int8_t swtch, bool nc, uint8_t level) ;
void init_soft_power( void ) ;
uint32_t check_soft_power( void ) ;
void soft_power_off( void ) ;
int8_t getGvarSourceValue( uint8_t src ) ;
static void	processAdjusters( void ) ;

/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/

uint32_t Master_frequency ;
volatile uint32_t Tenms ;						// Modified in interrupt routine
volatile uint8_t tick10ms = 0 ;
volatile uint8_t tick5ms = 0 ;
uint16_t g_LightOffCounter ;
uint8_t  InactivityMonitor = 0 ;
volatile uint32_t PowerOnTime ;						// Modified in interrupt routine

uint16_t S_anaFilt[ANALOG_DATA_SIZE] ;				// Analog inputs after filtering

uint8_t sysFlags = 0 ;
uint8_t SystemOptions ;

int16_t g_ppmIns[16];
uint8_t ppmInState = 0; //0=unsync 1..8= wait for value i-1
uint8_t Main_running ;
uint8_t LastVoiceFlushSwitch ;
int main( void ) ;

EEGeneral  g_eeGeneral;
SKYModelData  g_model;
//ProtocolData Protocols[2] ;

const uint8_t bchout_ar[] = {
															0x1B, 0x1E, 0x27, 0x2D, 0x36, 0x39,
															0x4B, 0x4E, 0x63, 0x6C, 0x72, 0x78,
                              0x87, 0x8D, 0x93, 0x9C, 0xB1, 0xB4,
                              0xC6, 0xC9, 0xD2, 0xD8, 0xE1, 0xE4		} ;


//new audio object
audioQueue  audio;

#define	ALERT_TYPE	0
#define MESS_TYPE		1

const char *AlertMessage ;
uint8_t AlertType ;

uint8_t AlarmTimer = 100 ;		// Units of 10 mS
uint8_t AlarmCheckFlag = 0 ;
uint8_t VoiceTimer = 10 ;		// Units of 10 mS
uint8_t VoiceCheckFlag100mS = 0 ;
//uint8_t CheckFlag50mS = 0 ;
uint8_t CheckFlag20mS = 0 ;
uint8_t CheckTimer = 2 ;
uint8_t DsmCheckTimer = 50 ;		// Units of 10 mS
uint8_t DsmCheckFlag = 0 ;

const char *Str_OFF = PSTR(STR_OFF) ;
const char *Str_ON = PSTR(STR_ON) ;



const char stickScramble[]= {
    0, 1, 2, 3,
    0, 2, 1, 3,
    3, 1, 2, 0,
    3, 2, 1, 0 };

uint8_t modeFixValue( uint8_t value )
{
	return stickScramble[g_eeGeneral.stickMode*4+value]+1 ;
}

MenuFuncP g_menuStack[6];

uint8_t  g_menuStackPtr = 0;
uint8_t  EnterMenu = 0 ;


// Temporary to allow compile
uint8_t g_vbat100mV ;//= 98 ;
uint8_t heartbeat ;
uint8_t heartbeat_running ;
uint32_t ResetReason ;
uint32_t ChipId ;


#if defined(LUA) || defined(BASIC)
uint32_t mainScreenDisplaying()
{
	return g_menuStack[g_menuStackPtr] == menuProc0 ;
}
#endif

void usbJoystickUpdate(void) ;

#if defined(PCB9XT) || defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
static bool usbPlugged(void)
{
  return GPIO_ReadInputDataBit(GPIOA, PIN_FS_VBUS);
}
#endif

void handleUsbConnection()
{
#if (defined(PCBX9D) || defined(PCB9XT)) && !defined(SIMU) || defined(PCBX12D) || defined(PCBX10)
  static bool usbStarted = false;

  if (!usbStarted && usbPlugged())
	{
    usbStarted = true;

    /*
      We used to initialize USB peripheral and driver here.
      According to my tests this is way too late. The USB peripheral
      therefore does not have enough information to start responding to
      USB host request, which causes very slow USB device recognition,
      multiple USB device resets, etc...

      If we want to change the USB profile, the procedure is simple:
        * USB cable must be disconnected
        * call usbDeInit();
        * call usbUnit(); which initializes USB with the new profile.
          Obviously the usbInit() should be modified to have a runtime
          selection of the USB profile.
    */

  }
  if (usbStarted && !usbPlugged())
	{
    usbStarted = false;
  }

  if (usbStarted )
	{
    usbJoystickUpdate();
  }

#endif //#if defined(PCB9XT) && !defined(SIMU)

}



#ifdef PCBX9D
// Needs to be in pulses_driver.h
extern void init_no_pulses(uint32_t port) ;
extern void init_pxx(uint32_t port) ;

void init_i2s1( void ) ;
#endif

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
extern void initWatchdog( void ) ;
#endif

uint8_t throttleReversed()
{
	return g_model.throttleReversed ^	g_eeGeneral.throttleReversed ;
}

void setLanguage()
{
	switch ( g_eeGeneral.language )
	{
		case 1 :
			Language = French ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = font_fr_h_extra ;
			ExtraHorusBigFont = font_fr_h_big_extra ;
#else
			ExtraFont = font_fr_extra ;
			ExtraBigFont = font_fr_big_extra ;
#endif
		break ;
		case 2 :
			Language = German ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = font_de_h_extra ;
			ExtraHorusBigFont = font_de_h_big_extra ;
#else
			ExtraFont = font_de_extra ;
			ExtraBigFont = font_de_big_extra ;
#endif
		break ;
#ifndef SMALL
		case 3 :
			Language = Norwegian ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = font_se_h_extra ;
			ExtraHorusBigFont = font_se_h_big_extra ;
#else
			ExtraFont = font_se_extra ;
			ExtraBigFont = font_se_big_extra ;
#endif
		break ;
		case 4 :
			Language = Swedish ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = font_se_h_extra ;
			ExtraHorusBigFont = font_se_h_big_extra ;
#else
			ExtraFont = font_se_extra ;
			ExtraBigFont = font_se_big_extra ;
#endif
		break ;
		case 5 :
			Language = Italian ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = font_it_h_extra ;
			ExtraHorusBigFont = NULL ;
#else
			ExtraFont = font_it_extra ;
			ExtraBigFont = NULL ;
#endif
		break ;
		case 6 :
			Language = Polish ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = font_pl_h_extra ;
			ExtraHorusBigFont = NULL ;
#else
			ExtraFont = font_pl_extra ;
			ExtraBigFont = NULL ;
#endif
		break ;
		case 7 :
			Language = Vietnamese ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = NULL ;
			ExtraHorusBigFont = NULL ;
#else
			ExtraFont = NULL ;
			ExtraBigFont = NULL ;
#endif
		break ;
		case 8 :
			Language = Spanish ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = NULL ;
			ExtraHorusBigFont = NULL ;
#else
			ExtraFont = NULL ;
			ExtraBigFont = NULL ;
#endif
		break ;
#endif
		default :
			Language = English ;
#if defined(PCBX12D) || defined(PCBX10)
			ExtraHorusFont = NULL ;
			ExtraHorusBigFont = NULL ;
#else
			ExtraFont = NULL ;
			ExtraBigFont = NULL ;
#endif
		break ;
	}
}

const uint8_t csTypeTable[] =
{ CS_VOFS, CS_VOFS, CS_VOFS, CS_VOFS, CS_VBOOL, CS_VBOOL, CS_VBOOL,
 CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VCOMP, CS_VBOOL, CS_VBOOL, CS_TIMER, CS_TIMER, CS_TMONO, CS_TMONO, CS_VOFS, CS_U16, CS_VCOMP, CS_VOFS
} ;

uint8_t CS_STATE( uint8_t x)
{
	return csTypeTable[x-1] ;
}


static void checkAlarm() // added by Gohst
{
    if(g_eeGeneral.disableAlarmWarning) return;
    if(!g_eeGeneral.beeperVal) alert(PSTR(STR_ALRMS_OFF));
}

static void checkWarnings()
{
    if(sysFlags & sysFLAG_OLD_EEPROM)
    {
        alert(PSTR(STR_OLD_EEPROM)); //will update on next save
        sysFlags &= ~(sysFLAG_OLD_EEPROM); //clear flag
    }
}

inline uint8_t keyDown()
{
#if defined(REV9E) || defined(PCBX7) || defined(PCBX12D) || defined(PCBX9LITE) || defined(PCBX10)
#ifdef PCBX7
 #ifndef PCBT12
	uint8_t value = (~GPIOE->IDR & PIN_BUTTON_ENCODER) ? 0x80 : 0 ;
 #endif
#endif // PCBX7
#ifdef PCBX9LITE
	uint8_t value = (~GPIOE->IDR & PIN_BUTTON_ENCODER) ? 0x80 : 0 ;
#endif // PCBX9LITE
#ifdef REV9E
	uint8_t value = (~GPIOF->IDR & PIN_BUTTON_ENCODER) ? 0x80 : 0 ;
#endif // REV9E
#if defined(PCBX12D)
	uint8_t value = (~GPIOC->IDR & 0x0002) ? 0x80 : 0 ;
#endif // PCBX12D
#if defined(PCBX10)
	uint8_t value = (~GPIOI->IDR & 0x0100) ? 0x80 : 0 ;
#endif // PCBX12D
 #ifndef PCBT12
	return (~read_keys() & 0x7E ) | value ;
 #else
	return ~read_keys() & 0x7E ;
 #endif
#else
	return ~read_keys() & 0x7E ;
#endif
}

void clearKeyEvents()
{
    while(keyDown())
		{
			  // loop until all keys are up
#if defined(PCBX12D) || defined(PCBX10)
			getADC_single() ;	// For nav joystick on left
#endif
			wdt_reset() ;
		}
    putEvent(0);
}

int32_t isAgvar(uint8_t value)
{
	if ( value >= 70 )
	{
		if ( value <= 76 )
		{
			return 1 ;
		}
	}
	return 0 ;
}


extern void maintenanceBackground( void ) ;

uint8_t SetByEncoder ;

void update_mode(void* pdata)
{
	uint32_t displayTimer = 0 ;
	g_menuStack[0] = menuUpdate ;
	g_menuStack[1] = menuUp1 ;	// this is so the first instance of [MENU LONG] doesn't freak out!
	MaintenanceRunning = 1 ;
	com1_Configure( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
#ifdef ACCESS
#ifdef PCBX9LITE
	configure_pins( INTMODULE_TX_GPIO_PIN, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTB | PIN_LOW ) ;
	GPIO_ResetBits( INTMODULE_TX_GPIO, INTMODULE_TX_GPIO_PIN) ;
#endif
#endif
#ifdef PCB9XT
	BlSetAllColours( 0, 30, 60 ) ;
#endif
	{
		uint8_t evt=getEvent() ;
		killEvents( evt ) ;
		putEvent(0) ;
	}
#ifdef PCBX9LITE
	ledBlue() ;
#endif
  while (1)
	{
		if ( (g_menuStackPtr==0) && (g_menuStack[0] == menuUpdate) )
		{
			if ( ( check_soft_power() == POWER_OFF )/* || ( goto_usb ) */ )		// power now off
			{
				soft_power_off() ;		// Only turn power off if necessary
			}
		}

	  static uint16_t lastTMR;
		uint16_t t10ms ;
		t10ms = get_tmr10ms() ;
  	tick10ms = ((uint16_t)(t10ms - lastTMR)) != 0 ;
	  lastTMR = t10ms ;

		maintenanceBackground() ;

		if(!tick10ms) continue ; //make sure the rest happen only every 10ms.
	  uint8_t evt=getEvent();
//#if defined(REV9E) || defined(PCBX7)
		{
			int32_t x ;
			if ( g_eeGeneral.rotaryDivisor == 1)
			{
				x = Rotary_count >> 2 ;
			}
			else if ( g_eeGeneral.rotaryDivisor == 2)
			{
				x = Rotary_count >> 1 ;
			}
			else
			{
				x = Rotary_count ;
			}
			Rotary_diff = x - LastRotaryValue ;
			LastRotaryValue = x ;
		}
		SetByEncoder = 0 ;
		if ( evt == 0 )
		{
	extern int32_t Rotary_diff ;
			if ( Rotary_diff > 0 )
			{
				evt = EVT_KEY_FIRST(KEY_DOWN) ;
				SetByEncoder = 1 ;
			}
			else if ( Rotary_diff < 0 )
			{
				evt = EVT_KEY_FIRST(KEY_UP) ;
				SetByEncoder = 1 ;
			}
			Rotary_diff = 0 ;
		}
//#endif

    lcd_clear() ;
		if ( EnterMenu )
		{
			evt = EnterMenu ;
			EnterMenu = 0 ;
		}
	 	Tevent = evt ;
		g_menuStack[g_menuStackPtr](evt);
		// Only update display every 40mS, improves SPort update throughput
		if ( ++displayTimer >= 4 )
		{
			displayTimer = 0 ;
    	refreshDisplay() ;
	#if defined(PCBX12D) || defined(PCBX10)
			lcd_clearBackground() ;	// Start clearing other frame
	#endif
		}

		wdt_reset();

		if ( Tenms )
		{
			Tenms = 0 ;
		}
#ifndef SIMU
		sdPoll10mS() ;
#endif

#ifndef SIMU
		CoTickDelay(1) ;					// 2mS for now
#endif
	}
}



#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
extern uint8_t TrainerMode ;
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#endif

#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
void checkTrainerSource()
{
	uint32_t tSource = g_eeGeneral.trainerProfile[g_model.trainerProfile].channel[0].source ;
	if ( CurrentTrainerSource	!= tSource )
	{
		switch ( CurrentTrainerSource )
		{
			case 0 :
				stop_trainer_capture() ;
			break ;
#ifndef PCBX12D
 #ifndef PCBX10
			case 1 :
				stop_USART6_Sbus() ;
				EXTERNAL_RF_OFF() ;
			break ;
			case 2 :
				stop_cppm_on_heartbeat_capture() ;
				EXTERNAL_RF_OFF() ;
			break ;
 #endif
#endif
			case 3 :
				stop_trainer_ppm() ;
			break ;
			case 4 :
				stop_trainer_capture() ;
				TrainerMode = 0 ;
//				init_trainer_capture(0) ;
			break ;
		}
		CurrentTrainerSource = tSource ;
		switch ( CurrentTrainerSource )
		{
			case 0 :
				init_trainer_capture(CAP_PPM) ;
//				EXTERNAL_RF_OFF() ;
			break ;
#ifndef PCBX12D
 #ifndef PCBX10
			case 1 :
				USART6_Sbus_configure() ;
				EXTERNAL_RF_ON() ;
			break ;
			case 2 :
				init_cppm_on_heartbeat_capture()  ;
				EXTERNAL_RF_ON() ;
			break ;
 #endif
#endif
			case 3 :	// Slave so output
				init_trainer_ppm() ;
				EXTERNAL_RF_OFF() ;
			break ;
			case 4 :
				init_trainer_capture(CAP_PPM) ;
				uint32_t tPolarity = g_eeGeneral.trainerProfile[g_model.trainerProfile].channel[1].source ;
				TrainerPolarity = tPolarity ;
				TrainerMode = 1 ;
				init_trainer_capture( CAP_SERIAL ) ;
			break ;
		}
	}
}
#endif


void com2Configure()
{
	if ( g_model.com2Function == COM2_FUNC_SBUSTRAIN )
	{
		UART_Sbus_configure( Master_frequency ) ;
	}
	else if ( g_model.com2Function == COM2_FUNC_SBUS57600 )
	{
		UART_Sbus57600_configure( Master_frequency ) ;
	}
	else if( g_model.com2Function == COM2_FUNC_CPPMTRAIN )
	{
		init_serial_trainer_capture() ;
	}
	else if ( g_model.com2Function == COM2_FUNC_LCD )
	{
		com2_Configure( 115200, SERIAL_NORM, 0 ) ;
	}
	else if ( g_model.com2Function == COM2_FUNC_BT_ENC )
	{
		com2_Configure( 100000, SERIAL_NORM, 0 ) ;
	}
	else
	{
		ConsoleInit() ;
	}
}

#if defined(PCB9XT) || defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
extern "C" void usbInit(void) ;
#endif

int main( void )
{
	uint32_t i ;
	for ( i = 0 ; i < 81 ; i += 1 )
	{
		NVIC->IP[i] = 0x80 ;
	}

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBXLITE) || defined(PCBX10)
	ChipId = *((uint16_t *)0x1FFF7A22) ;
	ResetReason = RCC->CSR ;
  RCC->CSR |= RCC_CSR_RMVF ;
#endif

	init_soft_power() ;

	initWatchdog() ;

	init_keys() ;

	setup_switches() ;

	ConsoleInit() ;

	init5msTimer() ;
  WatchdogTimeout = 100 ;

	init_hw_timer() ;

	init_adc() ;

	I2C_EE_Init() ;
	setVolume( 0 ) ;

	// SD card detect pin
	configure_pins( GPIO_Pin_CP, PIN_PORTD | PIN_INPUT | PIN_PULLUP ) ;

	disk_initialize( 0 ) ;
	sdInit() ;

	__enable_irq() ;

	lcdInit() ;

#ifdef PCBX9D
  lcd_clear() ;
	refreshDisplay() ;
#endif

	g_menuStack[0] =  menuProc0 ;

	init_trims() ;
	initHaptic() ;
	start_2Mhz_timer() ;
	setVolume( 0 ) ;
	start_sound() ;
	eeReadAll() ;
	protocolsToModules() ;

		g_eeGeneral.physicalRadioType = PHYSICAL_TARANIS_PLUS ;

	SportStreamingStarted = 0 ;
	setLanguage() ;
	lcdSetRefVolt(g_eeGeneral.contrast) ;

	init_adc2() ;
	createSwitchMapping() ;
	create6posTable() ;
	com2Configure() ;
	if ( ( read_trims() & 0x81 )== 0x81 )
	{
		// Do maintenance mode
		CoInitOS();

		MainTask = CoCreateTask( update_mode,NULL,5,&main_stk[MAIN_STACK_SIZE-1],MAIN_STACK_SIZE);
		CoStartOS();
		while(1) ;
	}

	while ( ( read_trims() & 0x01 )== 0x01 )
	{
		wdt_reset() ;
		HardwareMenuEnabled = 1 ;
	  WatchdogTimeout = 100 ;
		lcd_puts_Pleft( FH, XPSTR("Hardware Menu Enabled") ) ;
		refreshDisplay() ;

		if ( ( ( ResetReason & RCC_CSR_WDGRSTF ) == RCC_CSR_WDGRSTF ) || unexpectedShutdown )	// Not watchdog
		{
			break ;
		}
	}

  resetTimer();
	if ( g_eeGeneral.unexpectedShutdown )
	{
		unexpectedShutdown = 1 ;
	}

	telemetry_init( decodeTelemetryType( g_model.telemetryProtocol ) ) ;

	{
  	uint8_t i = keyDown(); //check for keystate
		if ( ( i & 6 ) == 6 )
		{
			SystemOptions |= SYS_OPT_MUTE ;
  		while ( keyDown() )
			{
				wdt_reset() ;
				lcd_puts_Pleft( FH, XPSTR("Mute Activated") ) ;
				refreshDisplay() ;
			}
		}
	}

	backlight_set( g_eeGeneral.bright, 0 ) ;
	backlight_set( g_eeGeneral.bright_white, 1 ) ;
	usbInit() ;


	uint16_t x ;
	x = g_eeGeneral.volume ;
	if ( g_model.anaVolume )	// Only check if on main screen
	{
		uint16_t divisor ;
		if ( g_model.anaVolume < 5 )
		{
		  getADC_single() ;
			x = anaIn(g_model.anaVolume+3) ;
			divisor = 2048 ;
			x = x * (NUM_VOL_LEVELS-1) / divisor ;
		}
		// Not checking for GVARS yet
	}
	setVolume( x ) ;

	// Choose here between PPM and PXX
	g_menuStack[1] = menuProcModelSelect ;	// this is so the first instance of [MENU LONG] doesn't freak out!

  //we assume that startup is like pressing a switch and moving sticks.  Hence the lightcounter is set
  //if we have a switch on backlight it will be able to turn on the backlight.
  if(g_eeGeneral.lightAutoOff > g_eeGeneral.lightOnStickMove)
    g_LightOffCounter = g_eeGeneral.lightAutoOff*500;
  else //if(g_eeGeneral.lightAutoOff <= g_eeGeneral.lightOnStickMove)
    g_LightOffCounter = g_eeGeneral.lightOnStickMove*500;
  check_backlight();

	// moved here and logic added to only play statup tone if splash screen enabled.
  // that way we save a bit, but keep the option for end users!

	if ( g_eeGeneral.welcomeType == 0 )
	{
		if(!g_eeGeneral.disableSplashScreen)
    {
			voiceSystemNameNumberAudio( SV_WELCOME, V_HELLO, AU_TADA ) ;
    }
	}
	else if ( g_eeGeneral.welcomeType == 2 )
	{
		putNamedVoiceQueue( (char *)g_eeGeneral.welcomeFileName, VLOC_USER ) ;
	}

	CoInitOS();

	MainTask = CoCreateTask( main_loop,NULL,5,&main_stk[MAIN_STACK_SIZE-1],MAIN_STACK_SIZE);
	LogTask = CoCreateTask(log_task,NULL,17,&Log_stk[LOG_STACK_SIZE-1],LOG_STACK_SIZE);
	VoiceTask = CoCreateTaskEx( voice_task,NULL,5,&voice_stk[VOICE_STACK_SIZE-1], VOICE_STACK_SIZE, 2, FALSE );
#ifdef	DEBUG
	DebugTask = CoCreateTaskEx( handle_serial,NULL,18,&debug_stk[DEBUG_STACK_SIZE-1],DEBUG_STACK_SIZE, 1, FALSE );
#endif
#ifdef SERIAL_HOST
	HostTask = CoCreateTaskEx( host, NULL,19, &Host_stk[HOST_STACK_SIZE-1], HOST_STACK_SIZE, 1, FALSE ) ;
#endif

	Main_running = 1 ;

	CoStartOS();

	while(1);
  return(0);
}

#ifndef SIMU

extern const char *openLogs( void ) ;
extern void writeLogs( void ) ;
extern void closeLogs( void ) ;

uint8_t LogsRunning = 0 ;
uint16_t LogTimer = 0 ;
extern uint8_t RawLogging ;
extern void rawStartLogging() ;

void log_task(void* pdata)
{
	while ( Activated == 0 )
	{
		CoTickDelay(10) ;					// 20mS
	}

  uint16_t tgtime = get_tmr10ms() ;		// 1 sec

	while(1)
	{
		// This needs to be a bit more accurate than
		// just a delay to get the correct logging rate
		do
		{
			CoTickDelay(5) ;					// 10mS
			if ( ( RawLogging ) && ( LogsRunning & 1 ) )
			{
				writeLogs() ;
			}
		} while( (uint16_t)(get_tmr10ms() - tgtime ) < 50 ) ;
  	tgtime += 50 ;
		LogTimer += 1 ;

		if ( g_model.logSwitch )
		{
			if ( getSwitch00( g_model.logSwitch ) )
			{	// logs ON
				if ( ( LogsRunning & 1 ) == 0 )
				{	// were off
					LogsRunning = 3 ;		// On and changed
					LogTimer = 0 ;
				}
			}
			else
			{	// logs OFF
				if ( LogsRunning & 1 )
				{	// were on
					LogsRunning = 2 ;		// Off and changed
				}
			}

			if ( LogsRunning & 2 )
			{
				if ( LogsRunning & 1 )
				{
					const char *result ;
					// Start logging
					SdAccessRequest = 1 ;
					while (lockOutVoice() == 0 )
					{
						CoTickDelay(1) ;					// 2mS
					}
					SdAccessRequest = 0 ;
					result = openLogs() ;
					unlockVoice() ;
					rawStartLogging() ;
					if ( result != NULL )
					{
    				audioDefevent( AU_SIREN ) ;
					}
				}
				else
				{
					// Stop logging
					closeLogs() ;
				}
				LogsRunning &= ~2 ;
			}

			if ( LogsRunning & 1 )
			{
				// log Data (depending on Rate)
				uint8_t mask = 0x0001 ;
				if ( g_model.logRate == 2 )		// 0.5 secs
				{
					mask = 0 ;
				}
				else if ( g_model.logRate )		// 2.0 secs
				{
					mask = 0x0003 ;
				}
				if ( ( LogTimer & mask ) == 0 )
				{
					writeLogs() ;
				}
			}
		}
	}
}


#endif	// SIMU


uint16_t PowerStatus ;

void speakModelVoice()
{
	if ( g_model.modelVoice == -1 )
	{
		putNamedVoiceQueue( g_model.modelVname, VLOC_MNAMES ) ;
	}
	else
	{
		putVoiceQueue( ( g_model.modelVoice + 260 ) | VLOC_NUMUSER  ) ;
	}
}

void prepareForShutdown()
{
	if ( LogsRunning & 1 )
	{
		closeLogs() ;
	}
  g_eeGeneral.unexpectedShutdown = 0 ;
	g_eeGeneral.SavedBatteryVoltage = g_vbat100mV ;
	STORE_MODELVARS ;			// To make sure we write model persistent timer
  STORE_GENERALVARS ;		// To make sure we write "unexpectedShutdown"

}

#ifdef POWER_BUTTON
	static uint8_t powerIsOn = 1 ;
#endif

#define RSSI_POWER_OFF	1
#define RSSI_STAY_ON		0

#ifdef CHECKRSSI
uint32_t checkRssi()
{
	static uint16_t timer ;
  if(g_eeGeneral.disableRxCheck) return RSSI_POWER_OFF ;

	if ( FrskyHubData[FR_RXRSI_COPY] == 0 )
	{
		return RSSI_POWER_OFF ;
	}

  // first - display warning

	lcd_clear();
  lcd_img( 1, 0, HandImage,0,0 ) ;
  lcd_putsAtt(36 + X12OFFSET,0*FH,XPSTR("Receiver"),DBLSIZE|CONDENSED);
  lcd_putsAtt(36 + X12OFFSET,2*FH,PSTR(STR_WARNING),DBLSIZE|CONDENSED);
	lcd_puts_P(0 + X12OFFSET,5*FH,  XPSTR("Rx still powered") ) ;
	lcd_puts_P(0 + X12OFFSET,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;
  refreshDisplay();
  clearKeyEvents();
	if ( (uint16_t)(get_tmr10ms() - timer ) > 49 )
	{
		putSystemVoice( SV_ALERT, 0 ) ;
		timer = get_tmr10ms() ;
	}

  while (1)
  {
		check_backlight() ;

		if ( FrskyHubData[FR_RXRSI_COPY] == 0 )
		{
			return RSSI_POWER_OFF ;
		}
    if( keyDown() )
    {
			clearKeyEvents() ;
			return RSSI_POWER_OFF ;
    }
    wdt_reset();
		CoTickDelay(5) ;					// 10mS for now
		getADC_osmp() ;
		perOutPhase(g_chans512, 0);
		check_frsky( 0 ) ;
		if ( ( check_soft_power() != POWER_OFF ) )		// power back on?
		{
			return RSSI_STAY_ON ;
    }
	}
}
#endif


// This is the main task for the RTOS
void main_loop(void* pdata)
{
	NumExtraPots = NUM_EXTRA_POTS ;

	if ( ( ( ResetReason & RCC_CSR_WDGRSTF ) != RCC_CSR_WDGRSTF ) && !unexpectedShutdown )	// Not watchdog
	{
		uint8_t evt ;
		// doSplash() ;

#ifndef SMALL
		if ( g_eeGeneral.calibMid[0] == 0x0400
				 && g_eeGeneral.calibSpanPos[0] == 0x0300
				 && g_eeGeneral.calibSpanNeg[0] == 0x0300 )
		{
			startupCalibration() ;
		}
#endif
		getADC_single();
  	checkTHR();
		checkCustom() ;
  	checkSwitches();
		checkAlarm();
		checkWarnings();
		check6pos() ;
		checkMultiPower() ;
		clearKeyEvents(); //make sure no keys are down before proceeding
		wdt_reset() ;
  	WatchdogTimeout = 200 ;
		VoiceCheckFlag100mS |= 6 ;// Set switch current states (global)
		processVoiceAlarms() ;
		VoiceCheckFlag100mS = 0 ;
		speakModelVoice() ;
		parseMultiData() ;
		evt = getEvent() ;
		killEvents( evt ) ;
		disableRtcBattery() ;

#ifdef LUA
		luaLoadModelScripts() ;
#endif
	}
#ifdef BASIC
	basicLoadModelScripts() ;
#endif
	VoiceCheckFlag100mS |= 2 ;// Set switch current states
	processSwitches() ;	// Guarantee unused switches are cleared

// Preload battery voltage
  int32_t ab = anaIn(12);

  ab = ( ab + ab*(g_eeGeneral.vBatCalib)/128 ) * 4191 ;
	ab /= 57165  ;
	g_vbat100mV = ab ;

// Switches PE2,7,8,9,13,14
	configure_pins( 0x6384, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;

	init_no_pulses( 0 ) ;
	init_no_pulses( 1 ) ;

	init_trainer_capture(0) ;

	rtcInit() ;

	init_xjt_heartbeat() ;

	heartbeat_running = 1 ;

  if (!g_eeGeneral.unexpectedShutdown)
	{
    g_eeGeneral.unexpectedShutdown = 1;
    STORE_GENERALVARS ;
  }

 	perOut( g_chans512, NO_DELAY_SLOW | FADE_FIRST | FADE_LAST ) ;
	startPulses() ;		// using the required protocol

extern uint8_t ModelImageValid ;
	if ( !ModelImageValid )
	{
		loadModelImage() ;
	}
	Activated = 1 ;


	if ( ( ( ResetReason & RCC_CSR_WDGRSTF ) != RCC_CSR_WDGRSTF ) && !unexpectedShutdown )	// Not watchdog
	{
		if ( g_vbat100mV > g_eeGeneral.SavedBatteryVoltage + 3 )
		{
			uint8_t result ;
  		clearKeyEvents() ;
			while(1)
			{
				lcd_clear() ;

				lcd_puts_P(0 + X12OFFSET,2*FH,  XPSTR("\002Battery Charged?") ) ;
				lcd_puts_P(0 + X12OFFSET,3*FH,  XPSTR("\004Reset Timer?") ) ;
			  lcd_puts_Pleft( 5*FH,PSTR(STR_YES_NO));
			  lcd_puts_Pleft( 6*FH,PSTR(STR_MENU_EXIT));
			  refreshDisplay() ;

				result = keyDown() & 0x86 ;
      	if( result )
	      {
				  clearKeyEvents() ;
					if ( result & 0x82 )
					{
						g_eeGeneral.totalElapsedTime = 0 ;
						result = getEvent() ;
						killEvents(result) ;
					}
    	    break ;
	      }
		    wdt_reset();
				CoTickDelay(5) ;					// 10mS for now
				if ( check_power_or_usb() ) break ;		// Usb on or power off
			}
		}
	}

	while (1)
	{

#if (not defined(REVA)) || defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)

		if ( ( check_soft_power() == POWER_OFF ) )		// power now off
		{
			if ( checkRssi() == RSSI_STAY_ON )
			{
				continue ;
			}
			// Time to switch off
			putSystemVoice( SV_SHUTDOWN, AU_TADA ) ;
			lcd_clear() ;
			lcd_puts_P( 4*FW + X12OFFSET, 3*FH, PSTR(STR_SHUT_DOWN) ) ;

			refreshDisplay() ;

			// Wait for OK to turn off
			// Currently wait 1 sec, needs to check eeprom finished

			prepareForShutdown() ;
#ifdef stm32f205
			disableRtcBattery() ;
#endif
  		uint16_t tgtime = get_tmr10ms() ;
  		uint16_t long_tgtime = tgtime ;
			uint32_t saved = 0 ;

	  	while( (uint16_t)(get_tmr10ms() - tgtime ) < 70 ) // 50 - Half second
  		{
				if ( (uint16_t)(get_tmr10ms() - tgtime ) > 60 )
				{
					if ( check_soft_power() == POWER_ON )
					{
						break ;		// Power back on
					}
				}
				wdt_reset() ;
				if ( AudioActive )
				{
					if ( (uint16_t)(get_tmr10ms() - long_tgtime ) < 600 )		// 6 seconds
					{
						tgtime = get_tmr10ms() ;
					}
				}

				if ( ! saved )
				{
					saved = 1 ;
					lcd_putsn_P( 5*FW, 5*FH, "EEPROM BUSY", 11 ) ;
					refreshDisplay() ;
					ee32_check_finished() ;
					lcd_putsn_P( 5*FW + X12OFFSET, 5*FH, "           ", 11 ) ;
 					tgtime = get_tmr10ms() ;
				}

				refreshDisplay() ;
  		}


				lcd_clear() ;
				lcd_putsn_P( 6*FW, 3*FH, "POWER OFF", 9 ) ;

				refreshDisplay() ;

				soft_power_off() ;		// Only turn power off if necessary

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
				for(;;)
				{
					wdt_reset() ;
  				PWR->CR |= PWR_CR_CWUF;
  				/* Select STANDBY mode */
  				PWR->CR |= PWR_CR_PDDS;
  				/* Set SLEEPDEEP bit of Cortex System Control Register */
  				SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  				/* Request Wait For Event */
  				__WFE();
				}
#endif // PCBX9D

//			}
		}
#endif
		mainSequence( MENUS ) ;
#ifndef SIMU
//		CoTickDelay(2) ;					// 4mS for now
		CoTickDelay(1) ;					// 2mS for now
#endif
	}

}


uint16_t getTmr2MHz()
{
#ifdef PCBSKY
	return TC1->TC_CHANNEL[0].TC_CV ;
#endif
#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
	return TIM7->CNT ;
#endif
}

uint32_t OneSecTimer ;
uint8_t StickScrollAllowed ;
uint8_t StickScrollTimer ;

extern int16_t AltOffset ;

static void almess( const char * s, uint8_t type )
{
	const char *h ;
  lcd_clear();
#if defined(PCBX12D) || defined(PCBX10)
  lcd_puts_P( X12OFFSET, 4*FW, s ) ;
#else
  lcd_puts_Pleft(4*FW,s);
#endif
	if ( type == ALERT_TYPE)
	{
    lcd_puts_P(64-6*FW + X12OFFSET,7*FH,"press any Key");
		h = PSTR(STR_ALERT) ;
//#ifdef PCBX12D
//  lcd_img( 1 + X12OFFSET, 0, HandImage,0,0, LCD_RED ) ;
//#else
//  lcd_img( 1, 0, HandImage,0,0 ) ;
//#endif

	}
	else
	{
		h = PSTR(STR_MESSAGE) ;
	}
  lcd_putsAtt(64-7*FW + X12OFFSET,0*FH, h,DBLSIZE);
  refreshDisplay();
}

int8_t getAndSwitch( SKYCSwData &cs )
{
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
	return cs.andsw ;
#endif
}

void doVoiceAlarmSource( VoiceAlarmData *pvad )
{
	if ( pvad->source )
	{
		// SORT OTHER values here
		if ( pvad->source >= NUM_XCHNRAW )
		{
			voice_telem_item( pvad->source - NUM_SKYXCHNRAW - 1 ) ;
		}
		else
		{
			int16_t value ;
			value = getValue( pvad->source - 1 ) ;
			voice_numeric( value, 0, 0 ) ;
		}
	}
}

uint32_t rssiOffsetValue( uint32_t type )
{
	uint32_t offset = 45 ;
	if ( ( ( g_model.Module[1].protocol == PROTO_DSM2) && ( g_model.Module[1].sub_protocol == DSM_9XR ) ) || ( g_model.telemetryProtocol == TELEMETRY_DSM ) )
	{
		if ( !g_model.dsmAasRssi )
		{
			offset = 20 ;
		}
	}
	if ( type )
	{
		offset -= 3 ;
		if ( offset == 17 )
		{
			offset = 18 ;
		}
	}
	return offset ;
}


static void processVoiceAlarms()
{
	uint32_t i ;
	uint32_t curent_state ;
	uint8_t flushSwitch ;
	VoiceAlarmData *pvad = &g_model.vad[0] ;
	i = 0 ;
	if ( VoiceCheckFlag100mS & 4 )
	{
		i = NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS ;
	}
	flushSwitch = getSwitch00( g_model.voiceFlushSwitch ) ;
	if ( ( VoiceCheckFlag100mS & 2 ) == 0 )
	{
		if ( flushSwitch && ( LastVoiceFlushSwitch == 0 ) )
		{
			flushVoiceQueue() ;
		}
	}
	LastVoiceFlushSwitch = flushSwitch ;
  for ( ; i < NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS + NUM_GLOBAL_VOICE_ALARMS ; i += 1 )
	{
		struct t_NvsControl *pc = &NvsControl[i] ;
		uint32_t play = 0 ;
		uint32_t functionTrue = 0 ;
		curent_state = 0 ;
		int16_t ltimer = pc->nvs_timer ;
	 	if ( i == NUM_VOICE_ALARMS )
		{
			pvad = &g_model.vadx[0] ;
		}
	 	if ( i == NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS )
		{
			pvad = &g_eeGeneral.gvad[0] ;
		}
		if ( pvad->func )		// Configured
		{
  		int16_t x ;
			int16_t y = pvad->offset ;
			x = getValue( pvad->source - 1 ) ;
  		switch (pvad->func)
			{
				case 1 :
					x = x > y ;
				break ;
				case 2 :
					x = x < y ;
				break ;
				case 3 :
					x = abs(x) > y ;
				break ;
				case 4 :
					x = abs(x) < y ;
				break ;
				case 5 :
				{
					if ( isAgvar( pvad->source ) )
					{
						x *= 10 ;
						y *= 10 ;
					}
    			x = abs(x-y) < 32 ;
				}
				break ;
				case 6 :
					x = x == y ;
				break ;
				case 7 :
					x = (x & y) != 0 ;
				break ;
				case 8 :
				{
  				int16_t z ;
					z = x - pc->nvs_last_value ;
					z = abs(z) ;
					if ( z > y )
					{
						pc->nvs_last_value = x ;
						x = 1 ;
					}
					else
					{
						x = 0 ;
					}
				}
				break ;
			}
			functionTrue = x ;
// Start of invalid telemetry detection
//					if ( pvad->source > ( CHOUT_BASE - NUM_SKYCHNOUT ) )
//					{ // Telemetry item
//						if ( !telemItemValid( pvad->source - 1 - CHOUT_BASE - NUM_SKYCHNOUT ) )
//						{
//							x = 0 ;	// Treat as OFF
//						}
//					}
// End of invalid telemetry detection
			if ( pvad->swtch )
			{
				if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
				{
					if ( getFlightPhase() == 0 )
					{
						x = 0 ;
					}
				}
				else if ( getSwitch00( pvad->swtch ) == 0 )
				{
					x = 0 ;
				}
			}
			if ( x == 0 )
			{
				ltimer = 0 ;
			}
			else
			{
				play = 1 ;
			}
		}
		else // No function
		{
			if ( pvad->swtch )
			{
				if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
				{
					curent_state = getFlightPhase() ? 1 : 0 ;
				}
				else
				{
					curent_state = getSwitch00( pvad->swtch ) ;
				}
				if ( curent_state == 0 )
				{
					ltimer = -1 ;
				}
			}
			else// No switch, no function
			{ // Check for source with numeric rate
				if ( pvad->rate >= 4 )	// A time
				{
					if ( pvad->vsource )
					{
						play = 1 ;
					}
				}
			}
		}
		play |= curent_state ;

		if ( ( VoiceCheckFlag100mS & 2 ) == 0 )
		{
		 if ( pvad->rate == 3 )	// All
		 {
		 		uint32_t pos ;
				pos = 1 ;
				if ( pvad->func && ( functionTrue == 0 ) )
				{
					pos = 0 ;
				}
		 		if ( pos )
				{
					if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
					{
						pos = getFlightPhase() ;
					}
					else
					{
						pos = switchPosition( pvad->swtch ) ;
					}
					uint32_t state = pc->nvs_state ;
					play = 0 ;
					if ( state != pos )
					{
						if ( state > 0x80 )
						{
							if ( --state == 0x80 )
							{
								state = pos ;
								ltimer = 0 ;
								play = pos + 1 ;
							}
						}
						else
						{
							state = 0x83 ;
						}
						pc->nvs_state = state ;
					}
				}
				else
				{
					pc->nvs_state = 0x40 ;
				}
		 }
		 else
		 {
			if ( play == 1 )
			{
				if ( pc->nvs_state == 0 )
				{ // just turned ON
					if ( ( pvad->rate == 0 ) || ( pvad->rate == 2 ) )
					{ // ON
						if ( pvad->delay )
						{
							pc->nvs_delay = pvad->delay + 1 ;
						}
						ltimer = 0 ;
					}
				}
				else
				{ // just turned OFF
					if ( pvad->rate == 1 )
					{
						if ( pvad->func == 8 )	// |d|>val
						{
							if ( pvad->delay )
							{
								pc->nvs_delay = pvad->delay + 1 ;
								play = 0 ;
							}
						}
					}
				}
				pc->nvs_state = 1 ;
				if ( ( pvad->rate == 1 ) )
				{
					play = 0 ;
				}
				if ( pc->nvs_delay )
				{
					if ( --pc->nvs_delay )
					{
						play = 0 ;
					}
				}
			}
			else
			{
				if ( ( pvad->func == 8 ) && ( pc->nvs_delay ) )	// |d|>val
				{
					play = 0 ;
					if ( --pc->nvs_delay == 0 )
					{
						play = 1 ;
					}
				}
				else
				{
					pc->nvs_delay = 0 ;
					if ( pc->nvs_state == 1 )
					{
						if ( ( pvad->rate == 1 ) || ( pvad->rate == 2 ) )
						{
							ltimer = 0 ;
							play = 1 ;
							if ( pvad->rate == 2 )
							{
								play = 2 ;
							}
						}
					}
				}
				pc->nvs_state = 0 ;
			}
			if ( pvad->rate == 33 )
			{
				play = 0 ;
				ltimer = -1 ;
			}
		 }
		}
		else //( ( VoiceCheckFlag100mS & 2 ) != 0 )
		{
		 	uint32_t pos ;
			if ( pvad->func == 8 )	// |d|>val
			{
				pc->nvs_last_value = getValue( pvad->source - 1 ) ;
			}
			if ( pvad->rate == 3 )
			{
				if ( pvad->swtch == MAX_SKYDRSWITCH + 1 )
				{
					pos = getFlightPhase() ;
				}
				else
				{
					pos = switchPosition( pvad->swtch ) ;
				}
			}
			else
			{
				pos = play ;
			}
			pc->nvs_state = pos ;
			play = 0 ;
			if ( pvad->rate == 33 )	// ONCE
			{
	 			if ( i >= NUM_VOICE_ALARMS + NUM_EXTRA_VOICE_ALARMS )
				{	// Global alert
					if ( VoiceCheckFlag100mS & 4 )
					{
						play = 1 ;
					}
				}
				else
				{
					play = 1 ;
				}
			}
			ltimer = -1 ;
		}

		if ( pvad->mute )
		{
			if ( pvad->source > ( CHOUT_BASE + NUM_SKYCHNOUT ) )
			{ // Telemetry item
				if ( !telemItemValid( pvad->source - 1 - CHOUT_BASE - NUM_SKYCHNOUT ) )
				{
					play = 0 ;	// Mute it
				}
			}
		}

		if ( play )
		{
			if ( ltimer < 0 )
			{
				if ( pvad->rate >= 4 )	// A time or ONCE
				{
					ltimer = 0 ;
				}
			}
			if ( ltimer == 0 )
			{
				if ( pvad->vsource == 1 )
				{
					doVoiceAlarmSource( pvad ) ;
				}
				if ( pvad->fnameType == 0 )	// None
				{
					// Nothing!
				}
				else if ( ( pvad->fnameType == 1 ) || ( pvad->fnameType == 4 ) )	// Name
				{
					char name[10] ;
					char *p ;
					p = (char *)ncpystr( (uint8_t *)name, pvad->file.name, 8 ) ;
					if ( name[0] && ( name[0] != ' ' ) )
					{
						if ( play >= 2 )
						{
							while ( *(p-1) == ' ' )
							{
								p -= 1 ;
							}
							*(p-1) += ( play - 1 ) ;
						}
						if ( pvad->fnameType == 4 )
						{
							putNamedVoiceQueue( name, VLOC_SYSTEM ) ;
						}
						else
						{
							putUserVoice( name, 0 ) ;
						}
					}
				}
				else if ( pvad->fnameType == 2 )	// Number
				{
					uint16_t value = pvad->file.vfile ;
					if ( value > 507 )
					{
						value = calc_scaler( value-508, 0, 0 ) ;
					}
					else if ( value > 500 )
					{
						value = g_model.gvars[value-501].gvar ;
					}
					putVoiceQueue( ( value + ( play - 1 ) ) | VLOC_NUMUSER ) ;
				}
				else
				{ // Audio
					int16_t index ;
					index = pvad->file.vfile ;
					if ( index == 16 )
					{
						index = AU_HAPTIC4 ;
					}
					audio.event( index, 0, 1 ) ;
				}
				if ( pvad->vsource == 2 )
				{
					doVoiceAlarmSource( pvad ) ;
				}
        if ( pvad->haptic )
				{
					audioDefevent( (pvad->haptic > 1) ? ( ( pvad->haptic == 3 ) ? AU_HAPTIC3 : AU_HAPTIC2 ) : AU_HAPTIC1 ) ;
				}
				if ( ( pvad->rate < 4 ) || ( pvad->rate > 32 ) )	// Not a time
				{
					ltimer = -1 ;
				}
				else
				{
					ltimer = 1 ;
				}
			}
			else if ( ltimer > 0 )
			{
				ltimer += 1 ;
				if ( ltimer > ( (pvad->rate-2) * 10 ) )
				{
					ltimer = 0 ;
				}
			}
		}
		pvad += 1 ;
		pc->nvs_timer = ltimer ;
	}

	if ( MuteTimer )
	{
		MuteTimer -= 1 ;
	}
}

#define MUSIC_SWITCH_SAME 0
#define MUSIC_SWITCH_ON		1
#define MUSIC_SWITCH_OFF	2

extern uint16_t PlayListCount ;
extern uint16_t PlaylistIndex ;

uint32_t musicSwitch( int16_t mSwitch, uint8_t *state )
{
  if(mSwitch)
	{
		if ( mSwitch < -HSW_MAX )
		{
			mSwitch += 256 ;
		}
		if ( VoiceCheckFlag100mS & 2 )
		{
  	  *state = getSwitch00( mSwitch-(HSW_MAX) ) ;
			return MUSIC_SWITCH_SAME ;
		}

  	if(mSwitch>(HSW_MAX))	 // toggeled switch
		{
  	  uint8_t swPos = getSwitch00( mSwitch-(HSW_MAX) ) ;
			if ( swPos != *state )
			{
				*state = swPos ;
				if ( swPos )	// Now on
				{
					return MUSIC_SWITCH_ON ;
				}
			}
		}
		else
		{
			// normal switch
  	  uint8_t swPos = getSwitch00( mSwitch ) ;
			if ( swPos != *state )
			{
				*state = swPos ;
				if ( swPos )	// Now on
				{
					return MUSIC_SWITCH_ON ;
				}
				else
				{ // now off
					return MUSIC_SWITCH_OFF ;
				}
			}
		}
	}
	return MUSIC_SWITCH_SAME ;
}

void processMusicSwitches()
{
  int16_t mSwitch ;
	uint32_t state ;
	mSwitch = g_model.musicData.musicStartSwitch ;
	state = musicSwitch( mSwitch, &MusicSwitches.LastMusicStartSwitchState ) ;

	if ( state == MUSIC_SWITCH_ON )	// Now on
	{
		if ( ( MusicPlaying == MUSIC_PLAYING ) || ( MusicPlaying == MUSIC_PAUSED ) )
		{
			MusicPlaying = MUSIC_STOPPING ;
		}
		else if ( MusicPlaying == MUSIC_STOPPED )
		{
			MusicPlaying = MUSIC_STARTING ;
		}
	}
	else if ( state == MUSIC_SWITCH_OFF )
	{
		if ( ( MusicPlaying == MUSIC_PLAYING ) || ( MusicPlaying == MUSIC_PAUSED ) )
		{
			MusicPlaying = MUSIC_STOPPING ;
		}
	}

	mSwitch = g_model.musicData.musicPauseSwitch ;
	state = musicSwitch( mSwitch, &MusicSwitches.LastMusicPauseSwitchState ) ;
	if ( state == MUSIC_SWITCH_ON )	// Now on
	{
		if ( MusicPlaying == MUSIC_PLAYING )
		{
			MusicPlaying = MUSIC_PAUSING ;
		}
		else if ( MusicPlaying == MUSIC_PAUSED )
		{
			MusicPlaying = MUSIC_RESUMING ;
		}
		else if ( MusicPlaying == MUSIC_PAUSING )
		{
			MusicPlaying = MUSIC_PLAYING ;
		}
	}
	else if ( state == MUSIC_SWITCH_OFF )
	{
		if ( MusicPlaying == MUSIC_PAUSED )
		{
			MusicPlaying = MUSIC_RESUMING ;
		}
		else if ( MusicPlaying == MUSIC_PAUSING )
		{
			MusicPlaying = MUSIC_PLAYING ;
		}
	}

	mSwitch = g_model.musicData.musicPrevSwitch ;
	state = musicSwitch( mSwitch, &MusicSwitches.LastMusicPrevSwitchState ) ;
	if ( g_eeGeneral.musicType )
	{
		if ( MusicPlaying == MUSIC_PLAYING )
		{
			if ( state == MUSIC_SWITCH_ON )	// Now on
			{
				MusicPrevNext = MUSIC_NP_PREV ;
			}
		}
	}

	mSwitch = g_model.musicData.musicNextSwitch ;
	state = musicSwitch( mSwitch, &MusicSwitches.LastMusicNextSwitchState ) ;
	if ( g_eeGeneral.musicType )
	{
		if ( MusicPlaying == MUSIC_PLAYING )
		{
			if ( state == MUSIC_SWITCH_ON )	// Now on
			{
				MusicPrevNext = MUSIC_NP_NEXT ;
			}
		}
	}
}

// every 20mS
void processSwitchTimer( uint32_t i )
{
  SKYCSwData &cs = g_model.customSw[i];
//  uint8_t cstate = CS_STATE(cs.func);

//  if(cstate == CS_TIMER)
//	{
		int16_t y ;
		y = CsTimer[i] ;
		if ( y == 0 )
		{
			int8_t z ;
			z = cs.v1 ;
			if ( z >= 0 )
			{
				z = -z-1 ;
				y = z * 50 ;
			}
			else
			{
				y = z * 5 ;
			}
		}
		else if ( y < 0 )
		{
			if ( ++y == 0 )
			{
				int8_t z ;
				z = cs.v2 ;
				if ( z >= 0 )
				{
					z += 1 ;
					y = z * 50 - 1 ;
				}
				else
				{
					y = -(z*5)-1 ;
				}
			}
		}
		else  // if ( CsTimer[i] > 0 )
		{
			y -= 1 ;
		}

		int8_t x = getAndSwitch( cs ) ;
		if ( x )
		{
		  if (getSwitch00( x) == 0 )
			{
				Last_switch[i] = 0 ;
				if ( cs.func == CS_NTIME )
				{
					int8_t z ;
					z = cs.v1 ;
					if ( z >= 0 )
					{
						z = -z-1 ;
						y = z * 50 ;
					}
					else
					{
						y = z * 5 ;
					}
				}
				else
				{
					y = -1 ;
				}
			}
			else
			{
				Last_switch[i] = 2 ;
			}
		}
		CsTimer[i] = y ;
//	}
}

//// Every 20mS
//static void processLatchFflop()
//{
//	uint32_t i ;
//	for ( i = 0 ; i < NUM_SKYCSW ; i += 1 )
//	{
// 	  SKYCSwData &cs = g_model.customSw[i];
//		if ( cs.func == CS_FLIP )
//		{
//		  if (getSwitch00( cs.v1) )
//			{
//				if ( ( Last_switch[i] & 2 ) == 0 )
//				{
//					 Clock it!
//			    if (getSwitch00( cs.v2) )
//					{
//						Last_switch[i] = 3 ;
//					}
//					else
//					{
//						Last_switch[i] = 2 ;
//					}
//				}
//			}
//			else
//			{
//				Last_switch[i] &= ~2 ;
//			}
//		}
//	}
//}

// Every 20mS
void processSwitches()
{
	uint32_t cs_index ;
	for ( cs_index = 0 ; cs_index < NUM_SKYCSW ; cs_index += 1 )
	{
  	SKYCSwData &cs = g_model.customSw[cs_index] ;
  	uint8_t ret_value = false ;

  	if( cs.func )
		{
  		int8_t a = cs.v1 ;
  		int8_t b = cs.v2 ;
  		int16_t x = 0 ;
  		int16_t y = 0 ;
  		uint8_t s = CS_STATE( cs.func ) ;

  		if(s == CS_VOFS)
  		{
  		  x = getValue(cs.v1u-1);
    		if ( ( ( cs.v1u > CHOUT_BASE+NUM_SKYCHNOUT) && ( cs.v1u < EXTRA_POTS_START ) ) || (cs.v1u >= EXTRA_POTS_START + 8) )
				{
  		    y = convertTelemConstant( cs.v1u-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
				}
  		  else
  		  	y = calc100toRESX(cs.v2);
  		}
  		else if(s == CS_VCOMP)
  		{
 		    x = getValue(cs.v1u-1);
 		    y = getValue(cs.v2u-1);
  		}

  		switch (cs.func)
			{
	  		case (CS_VPOS):
  		    ret_value = (x>y);
  	    break;
  			case (CS_VNEG):
  		    ret_value = (x<y) ;
  	    break;
  			case (CS_APOS):
	  	    ret_value = (abs(x)>y) ;
  		  break;
	  		case (CS_ANEG):
  		    ret_value = (abs(x)<y) ;
  	    break;
				case CS_VEQUAL :
  		    ret_value = (x == y) ;
  	    break;
				case CS_EXEQUAL:
					if ( isAgvar( cs.v1 ) )
					{
						x *= 10 ;
						y *= 10 ;
					}
  		  	ret_value = abs(x-y) < 32 ;
  			break;

				case CS_VXEQUAL:
					if ( isAgvar( cs.v1 ) || isAgvar( cs.v2 ) )
					{
						x *= 10 ;
						y *= 10 ;
					}
  			  ret_value = abs(x-y) < 32 ;
  			break;

  			case (CS_AND):
  			case (CS_OR):
  			case (CS_XOR):
  			{
  			  bool res1 = getSwitch(a,0,0) ;
  			  bool res2 = getSwitch(b,0,0) ;
  			  if ( cs.func == CS_AND )
  			  {
  			    ret_value = res1 && res2 ;
  			  }
  			  else if ( cs.func == CS_OR )
  			  {
  			    ret_value = res1 || res2 ;
  			  }
  			  else  // CS_XOR
  			  {
  			    ret_value = res1 ^ res2 ;
  			  }
  			}
  			break;

	  		case (CS_EQUAL):
  		    ret_value = (x==y);
  	    break;
  			case (CS_NEQUAL):
  		    ret_value = (x!=y);
  	    break;
  			case (CS_GREATER):
  		    ret_value = (x>y);
  		   break;
	  		case (CS_LESS):
  		    ret_value = (x<y);
  	    break;
	  		case (CS_NTIME):
					processSwitchTimer( cs_index ) ;
					ret_value = CsTimer[cs_index] >= 0 ;
  			break ;
				case (CS_TIME):
				{
					processSwitchTimer( cs_index ) ;
  			  ret_value = CsTimer[cs_index] >= 0 ;
					int8_t x = getAndSwitch( cs ) ;
					if ( x )
					{
					  if (getSwitch( x, 0, 0 ) )
						{
							if ( ( Last_switch[cs_index] & 2 ) == 0 )
							{ // Triggering
								ret_value = 1 ;
							}
						}
					}
				}
  			break ;
  			case (CS_MONO):
  			case (CS_RMONO):
				{
					if ( VoiceCheckFlag100mS & 2 )
					{
						// Resetting, retrigger any monostables
						Last_switch[cs_index] &= ~2 ;
					}
					int8_t andSwOn = 1 ;
					if ( ( cs.func == CS_RMONO ) )
					{
						andSwOn = getAndSwitch( cs ) ;
						if ( andSwOn )
						{
							andSwOn = getSwitch00( andSwOn) ;
						}
						else
						{
							andSwOn = 1 ;
						}
					}

				  if (getSwitch00( cs.v1) )
					{
						if ( ( Last_switch[cs_index] & 2 ) == 0 )
						{
							// Trigger monostable
							uint8_t trigger = 1 ;
							if ( ( cs.func == CS_RMONO ) )
							{
								if ( ! andSwOn )
								{
									trigger = 0 ;
								}
							}
							if ( trigger )
							{
								Last_switch[cs_index] = 3 ;
								int16_t x ;
								x = cs.v2 * 5 ;
								if ( x < 0 )
								{
									x = -x ;
								}
								else
								{
									x += 5 ;
									x *= 10 ;
								}
								CsTimer[cs_index] = x ;
							}
						}
					}
					else
					{
						Last_switch[cs_index] &= ~2 ;
					}
					int16_t y ;
					y = CsTimer[cs_index] ;
					if ( Now_switch[cs_index] < 2 )	// not delayed
					{
						if ( y )
						{
							if ( ( cs.func == CS_RMONO ) )
							{
								if ( ! andSwOn )
								{
									y = 1 ;
								}
							}
							if ( --y == 0 )
							{
								Last_switch[cs_index] &= ~1 ;
							}
							CsTimer[cs_index] = y ;
						}
					}
 			  	ret_value = CsTimer[cs_index] > 0 ;
				}
  			break ;

				case (CS_LATCH) :
		  		if (getSwitch00( cs.v1) )
					{
						Last_switch[cs_index] = 1 ;
					}
					else
					{
					  if (getSwitch00( cs.v2) )
						{
							Last_switch[cs_index] = 0 ;
						}
					}
  			  ret_value = Last_switch[cs_index] & 1 ;
  			break ;

				case (CS_FLIP) :
		  		if (getSwitch00( cs.v1) )
					{
						if ( ( Last_switch[cs_index] & 2 ) == 0 )
						{
							// Clock it!
					    if (getSwitch00( cs.v2) )
							{
								Last_switch[cs_index] = 3 ;
							}
							else
							{
								Last_switch[cs_index] = 2 ;
							}
						}
					}
					else
					{
						Last_switch[cs_index] &= ~2 ;
					}
  			  ret_value = Last_switch[cs_index] & 1 ;
  			break ;
  			case (CS_BIT_AND) :
				{
  			  x = getValue(cs.v1u-1);
					y = cs.v2u ;
					y |= cs.bitAndV3 << 8 ;
  			  ret_value = ( x & y ) != 0 ;
				}
  			break ;
  			default:
  		    ret_value = false;
 		    break;
  		}

			if ( ret_value )
			{
				int8_t x = getAndSwitch( cs ) ;
				if ( x )
				{
  		    ret_value = getSwitch( x, 0, 0 ) ;
				}
			}
			if ( ( cs.func < CS_LATCH ) || ( cs.func > CS_RMONO ) )
			{
				Last_switch[cs_index] = ret_value ;
			}
			if ( Now_switch[cs_index] == 0 )	// was off
			{
				if ( ret_value )
				{
					if ( g_model.switchDelay[cs_index] )
					{
						ret_value = g_model.switchDelay[cs_index] * 10 ;
					}
				}
			}
			else
			{
				if ( Now_switch[cs_index] > 1 )	// delayed
				{
					if ( ret_value )
					{
						uint8_t temp = Now_switch[cs_index] - 2 ;
						if ( temp )
						{
							ret_value = temp ;
						}
					}
				}
			}
			Now_switch[cs_index] = ret_value ;
		}
		else // no function
		{
			if ( VoiceCheckFlag100mS & 2 )
			{
				Now_switch[cs_index] = 0 ;
			}
		}
	}
}

#ifdef ARUNI
inline bool qSixPosCalibrating()
{ // six position switch is being calibrated
  return ((SixPosCaptured & 0x80) == 0);
}
#endif

uint32_t MixerRate ;
uint32_t MixerCount ;

uint8_t AlarmTimers[NUM_SKYCHNOUT+EXTRA_SKYCHANNELS] ;

uint8_t EncoderI2cData[2] ;
uint16_t I2CencCounter ;

void mainSequence( uint32_t no_menu )
{
	CalcScaleNest = 0 ;

  uint16_t t0 = getTmr2MHz();
	CPU_UINT numSafety = NUM_SKYCHNOUT - g_model.numVoice ;
	if ( g_eeGeneral.filterInput == 1 )
	{
		getADC_osmp() ;
	}
	else if ( g_eeGeneral.filterInput == 2 )
	{
		getADC_filt() ;
	}
	else
	{
		getADC_single() ;
	}

	perMain( no_menu ) ;		// Allow menu processing
	if(heartbeat == 0x3)
	{
    wdt_reset();
    heartbeat = 0;
  }

	checkTrainerSource() ;
	if ( ( g_model.com2Function == COM2_FUNC_SBUSTRAIN ) || ( g_model.com2Function == COM2_FUNC_SBUS57600 ) || ( CurrentTrainerSource == TRAINER_SBUS ) )
	{
		TrainerProfile *tProf = &g_eeGeneral.trainerProfile[g_model.trainerProfile] ;
		if ( tProf->channel[0].source != TRAINER_JACK )
		{
			processSbusInput() ;
		}
	}

extern uint8_t TrainerMode ;
	if ( TrainerMode == 1 )
	{
		TrainerProfile *tProf = &g_eeGeneral.trainerProfile[g_model.trainerProfile] ;
		if ( tProf->channel[0].source == TRAINER_J_SBUS )
		{
			processSbusInput() ;
		}
	}

	if ( Tenms )
	{

		Tenms = 0 ;
		eePoll() ;
		handleUsbConnection() ;

		if ( ++OneSecTimer >= 100 )
		{
			OneSecTimer -= 100 ;

			if ( StickScrollTimer )
			{
				StickScrollTimer -= 1 ;
			}
			MixerRate = MixerCount ;
			MixerCount = 0 ;
			uint32_t x ;
static uint32_t saveIdleCount ;
//			x = ( IdleCount - saveIdleCount ) * 10000 ;
			x = ( IdleCount - saveIdleCount ) ;
//			x /= 2000000 ;
			x /= 200 ;
			saveIdleCount = IdleCount ;
			if ( x > 9999 )
			{
				x = 9999 ;
			}
			IdlePercent = x ;


extern uint32_t TotalExecTime ;

			BasicExecTime = TotalExecTime / 2000 ;
			TotalExecTime = 0 ;

		}
		sdPoll10mS() ;

		rtc_gettime( &Time ) ;
	}

	processAdjusters() ;

	t0 = getTmr2MHz() - t0;
  if ( t0 > g_timeMain ) g_timeMain = t0 ;
  if ( AlarmCheckFlag > 1 )		// Every 1 second
  {
    AlarmCheckFlag = 0 ;
    // Check for alarms here
    // Including Altitude limit

    if (frskyUsrStreaming)
    {
      int16_t limit = g_model.FrSkyAltAlarm ;
      int16_t altitude ;
      if ( limit )
      {
        if (limit == 2)  // 400
        {
          limit = 400 ;	//ft
        }
        else
        {
          limit = 122 ;	//m
        }
				altitude = FrskyHubData[FR_ALT_BARO] + AltOffset ;
				altitude /= 10 ;
				if (g_model.FrSkyUsrProto == 0)  // Hub
				{
      		if ( g_model.FrSkyImperial )
					{
        		altitude = m_to_ft( altitude ) ;
					}
				}
        if ( altitude > limit )
        {
          audioDefevent(AU_WARNING2) ;
        }
      }
    }


    // this var prevents and alarm sounding if an earlier alarm is already sounding
    // firing two alarms at once is pointless and sounds rubbish!
    // this also means channel A alarms always over ride same level alarms on channel B
    // up to debate if this is correct!
    //				bool AlarmRaisedAlready = false;

    if (frskyStreaming)
		{
//      enum AlarmLevel level[4] ;
      // RED ALERTS
//      if( (level[0]=FRSKY_alarmRaised(0,0)) == alarm_red) FRSKY_alarmPlay(0,0);
//      else if( (level[1]=FRSKY_alarmRaised(0,1)) == alarm_red) FRSKY_alarmPlay(0,1);
//      else	if( (level[2]=FRSKY_alarmRaised(1,0)) == alarm_red) FRSKY_alarmPlay(1,0);
//      else if( (level[3]=FRSKY_alarmRaised(1,1)) == alarm_red) FRSKY_alarmPlay(1,1);
//      // ORANGE ALERTS
//      else	if( level[0] == alarm_orange) FRSKY_alarmPlay(0,0);
//      else if( level[1] == alarm_orange) FRSKY_alarmPlay(0,1);
//      else	if( level[2] == alarm_orange) FRSKY_alarmPlay(1,0);
//      else if( level[3] == alarm_orange) FRSKY_alarmPlay(1,1);
//      // YELLOW ALERTS
//      else	if( level[0] == alarm_yellow) FRSKY_alarmPlay(0,0);
//      else if( level[1] == alarm_yellow) FRSKY_alarmPlay(0,1);
//      else	if( level[2] == alarm_yellow) FRSKY_alarmPlay(1,0);
//      else if( level[3] == alarm_yellow) FRSKY_alarmPlay(1,1);

			// Check for current alarm
			if ( g_model.currentSource )
			{
				if ( g_model.frskyAlarms.alarmData[0].frskyAlarmLimit )
				{
					if ( ( FrskyHubData[FR_AMP_MAH] >> 6 ) >= g_model.frskyAlarms.alarmData[0].frskyAlarmLimit )
					{
						putSystemVoice( SV_CAP_WARN, V_CAPACITY ) ;
					}
					uint32_t value ;
					value = g_model.frskyAlarms.alarmData[0].frskyAlarmLimit ;
					value <<= 6 ;
					value = 100 - ( FrskyHubData[FR_AMP_MAH] * 100 / value ) ;
					FrskyHubData[FR_FUEL] = value ;
				}
			}
    }

		// Now for the Safety/alarm switch alarms
		{
			uint8_t i ;
			static uint8_t periodCounter ;

			periodCounter += 0x11 ;
			periodCounter &= 0xF7 ;
			if ( periodCounter > 0x5F )
			{
				periodCounter &= 0x0F ;
			}

			for ( i = 0 ; i < numSafety ; i += 1 )
			{
    		SKYSafetySwData *sd = &g_model.safetySw[i] ;
				if (sd->opt.ss.mode == 1)
				{
					if ( AlarmTimers[i] == 0 )
					{
						if(getSwitch00( sd->opt.ss.swtch))
						{
							audio.event( /*((g_eeGeneral.speakerMode & 1) == 0) ? 1 :*/ sd->opt.ss.val ) ;
							AlarmTimers[i] = 40 ;
						}
					}
				}
				if (sd->opt.ss.mode == 2)
				{
					if ( sd->opt.ss.swtch > MAX_SKYDRSWITCH )
					{
						switch ( sd->opt.ss.swtch - MAX_SKYDRSWITCH -1 )
						{
							case 0 :
								if ( ( periodCounter & 3 ) == 0 )
								{
									voice_telem_item( sd->opt.ss.val ) ;
								}
							break ;
							case 1 :
								if ( ( periodCounter & 0xF0 ) == 0 )
								{
									voice_telem_item( sd->opt.ss.val ) ;
								}
							break ;
							case 2 :
								if ( ( periodCounter & 7 ) == 2 )
								{
									voice_telem_item( sd->opt.ss.val ) ;
								}
							break ;
						}
					}
					else if ( ( periodCounter & 3 ) == 0 )		// Every 4 seconds
					{
						if(getSwitch00( sd->opt.ss.swtch))
						{
							putVoiceQueue( ( sd->opt.ss.val + 128 ) | VLOC_NUMUSER ) ;
						}
					}
				}
			}
		}
  }


	// New switch voices
	// New entries, Switch, (on/off/both), voice file index

	if ( CheckFlag20mS )
	{
		CheckFlag20mS = 0 ;
//		processLatchFflop() ;
		processSwitches() ;
	}

//	if ( CheckFlag50mS )
//	{
//		CheckFlag50mS = 0 ;
//		// Process all switches for delay etc.
////		processTimer() ;
//	}

	if ( VoiceCheckFlag100mS )	// Every 100mS
  {
		uint32_t i ;
		static uint32_t timer ;
		static uint32_t delayTimer = 0 ;

		if ( VoiceCheckFlag100mS & 1 )
		{
    	for ( i = 0 ; i < HUBDATALENGTH ; i += 1 )
			{
				if (TelemetryDataValid[i] )
				{
					TelemetryDataValid[i] -= 1 ;
				}
			}

			TrimInUse[0] <<= 1 ;
			TrimInUse[1] <<= 1 ;
			TrimInUse[2] <<= 1 ;
			TrimInUse[3] <<= 1 ;
//#ifdef TELEMETRY_LOST
//			if ( TelemetryStatus && ( frskyStreaming == 0 ) )
//			{ // We have lost telemetry
//				putSystemVoice( SV_NO_TELEM, V_NOTELEM ) ;
//			}
//			TelemetryStatus = frskyStreaming ;
//#endif
			timer += 1 ;
			if ( delayTimer )
			{
				delayTimer -= 1 ;
			}

			uint8_t redAlert = 0 ;
			static uint8_t redCounter ;
			static uint8_t orangeCounter ;
			uint8_t rssiValue = FrskyHubData[FR_RXRSI_COPY] ;

			if ( frskyStreaming )
			{
				int8_t offset ;
				if ( g_model.enRssiRed == 0 )
				{
					offset = rssiOffsetValue( 1 ) ;

					if ( rssiValue && rssiValue < g_model.rssiRed + offset )
					{
						// Alarm
						redAlert = 1 ;
						orangeCounter += 1 ;
						if ( ++redCounter > 3 )
						{
							if ( delayTimer == 0 )
							{
								putSystemVoice( SV_RSSICRIT, V_RSSI_CRITICAL ) ;
								delayTimer = 40 ;	// 4 seconds
							}
							redCounter = 0 ;
						}
					}
					else
					{
						redCounter = 0 ;
					}
				}
				if ( ( redAlert == 0 ) && ( g_model.enRssiOrange == 0 ) )
				{
					offset = rssiOffsetValue( 0 ) ;
					if ( rssiValue && rssiValue < g_model.rssiOrange + offset )
					{
						// Alarm
						if ( ++orangeCounter > 3 )
						{
							if ( delayTimer == 0 )
							{
								putSystemVoice( SV_RSSI_LOW, V_RSSI_WARN ) ;
								delayTimer = 40 ;	// 4 seconds
							}
							orangeCounter = 0 ;
						}
					}
					else
					{
						orangeCounter = 0 ;
					}
				}
			}

			for ( i = 0 ; i < numSafety ; i += 1 )
			{
				if ( AlarmTimers[i] )
				{
					AlarmTimers[i] -= 1 ;
				}
    		SKYSafetySwData *sd = &g_model.safetySw[i] ;
				if (sd->opt.ss.mode == 2)
				{
					if ( sd->opt.ss.swtch <= MAX_SKYDRSWITCH )
					{
						if ( AlarmTimers[i] == 0 )
						{
							if(getSwitch00( sd->opt.ss.swtch))
							{
								putVoiceQueue( ( sd->opt.ss.val + 128 ) | VLOC_NUMUSER ) ;
								AlarmTimers[i] = 40 ;		// 4 seconds
							}
						}
			    }
				}
			}
			for ( i = 0 ; i < NUM_SCALERS ; i += 1 )
			{
				if ( g_model.eScalers[i].dest )
				{
					calc_scaler( i, 0, 0 ) ;
				}
			}
		}

		for ( i = numSafety ; i < NUM_SKYCHNOUT+NUM_VOICE ; i += 1 )
		{
			uint8_t curent_state ;
			uint8_t mode ;
			uint8_t value ;
    	SKYSafetySwData *sd = &g_model.safetySw[i];
    	if ( i >= NUM_SKYCHNOUT )
			{
				sd = (SKYSafetySwData*) &g_model.voiceSwitches[i-NUM_SKYCHNOUT];
			}

			mode = sd->opt.vs.vmode ;
			value = sd->opt.vs.vval ;
			if ( mode <= 5 )
			{
				if ( value > 250 )
				{
					value = g_model.gvars[value-248].gvar ; //Gvars 3-7
				}
			}

			if ( sd->opt.vs.vswtch )		// Configured
			{
				curent_state = getSwitch00( sd->opt.vs.vswtch ) ;
				if ( ( VoiceCheckFlag100mS & 2 ) == 0 )
				{
					if ( ( mode == 0 ) || ( mode == 2 ) )
					{ // ON
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							putVoiceQueue( ( sd->opt.vs.vval ) | VLOC_NUMUSER  ) ;
						}
					}
					if ( ( mode == 1 ) || ( mode == 2 ) )
					{ // OFF
						if ( ( Vs_state[i] == 1 ) && !curent_state )
						{
							uint8_t x ;
							x = sd->opt.vs.vval ;
							if ( mode == 2 )
							{
								x += 1 ;
							}
							putVoiceQueue( x | VLOC_NUMUSER  ) ;
						}
					}
					if ( mode > 5 )
					{
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							voice_telem_item( sd->opt.vs.vval ) ;
						}
					}
					else if ( mode > 2 )
					{ // 15, 30 or 60 secs
						if ( curent_state )
						{
							uint16_t mask ;
							mask = 150 ;
							if ( mode == 4 ) mask = 300 ;
							if ( mode == 5 ) mask = 600 ;
							if ( timer % mask == 0 )
							{
								putVoiceQueue( sd->opt.vs.vval | VLOC_NUMUSER  ) ;
							}
						}
					}
				}
				Vs_state[i] = curent_state ;
			}
		}

		{
			// Check CVLT and CTOT every 100 mS
    	if (frskyUsrStreaming)
		  {
//				uint16_t total_volts = 0 ;
				uint16_t total_1volts = 0 ;
				uint16_t total_2volts = 0 ;
				CPU_UINT audio_sounded = 0 ;
				uint16_t low_cell = 440 ;		// 4.4V
				for (uint8_t k=0 ; k<12 ; k++)
				{
					uint32_t index = k < 6 ? TEL_ITEM_CELL1 : TEL_ITEM_CELL7 - 6 ;
					index += k ;
					if ( telemItemValid( index ) )
					{
						if ( k < 6 )
						{
							total_1volts += FrskyHubData[FR_CELL1+k] ;
						}
						else
						{
							total_2volts += FrskyHubData[FR_CELL1+k] ;
						}
						if ( FrskyHubData[FR_CELL1+k] < low_cell )
						{
							low_cell = FrskyHubData[FR_CELL1+k] ;
						}
						if ( AlarmCheckFlag > 1 )
						{
							if ( audio_sounded == 0 )
							{
		  	  		  if ( FrskyHubData[FR_CELL1+k] < g_model.frSkyVoltThreshold * 2 )
								{
		  	  		    audioDefevent(AU_WARNING3);
									audio_sounded = 1 ;
								}
				  		}
						}
	  			}
					// Now we have total volts available
					if ( total_1volts + total_2volts )
					{
						FrskyHubData[FR_CELLS_TOT] = (total_1volts + total_2volts + 4 ) / 10 ; 	// Some rounding up
						TelemetryDataValid[FR_CELLS_TOT] = 40 + g_model.telemetryTimeout ;

						if ( total_1volts )
						{
							FrskyHubData[FR_CELLS_TOTAL1] = (total_1volts + 4)  / 10 ;
							TelemetryDataValid[FR_CELLS_TOTAL1] = 40 + g_model.telemetryTimeout ;
					  }
						if ( total_2volts )
						{
							FrskyHubData[FR_CELLS_TOTAL2] = (total_2volts + 4)  / 10 ;
							TelemetryDataValid[FR_CELLS_TOTAL2] = 40 + g_model.telemetryTimeout ;
						}
						if ( low_cell < 440 )
						{
							FrskyHubData[FR_CELL_MIN] = low_cell ;
						}
					}
				}
			}
		}

		// Now test for the voice alarms
		processVoiceAlarms() ;
		processMusicSwitches() ;

		VoiceCheckFlag100mS = 0 ;

		// Vario
		{

			static uint8_t varioRepeatRate = 0 ;
			static uint8_t sounded = 0 ;

			if ( g_model.varioData.varioSource ) // Vario enabled
			{
				if ( getSwitch00( g_model.varioData.swtch ) )
				{
					uint8_t new_rate = 0 ;
					if ( varioRepeatRate )
					{
						varioRepeatRate -= 1 ;
					}
					if ( varioRepeatRate == 0 )
					{
						sounded = 0 ;
					}
					int16_t vspd ;
					if ( g_model.varioData.varioSource == 1 )
					{
						vspd = FrskyHubData[FR_VSPD] ;

						if ( g_model.varioData.param > 1 )
						{
							vspd /= g_model.varioData.param ;
						}
					}
					else if ( g_model.varioData.varioSource == 2 )
					{
						vspd = FrskyHubData[FR_A2_COPY] - 128 ;
						if ( ( vspd < 3 ) && ( vspd > -3 ) )
						{
							vspd = 0 ;
						}
						vspd *= g_model.varioData.param ;
					}
					else
					{
						// A Scaler
						vspd = calc_scaler( g_model.varioData.varioSource-3, 0, 0 ) ;
						if ( g_model.varioData.param > 1 )
						{
							vspd /= g_model.varioData.param ;
						}
					}
					if ( vspd )
					{
						if ( vspd < 0 )
						{
							vspd = -vspd ;
							if (!g_model.varioData.sinkTones )
							{
								if ( vspd > 25 )		// OpenXsensor
								{
									if ( sounded != 2 )
									{
										sounded = 2 ;
										varioRepeatRate = 0 ;
  	  	     		    audio.event( AU_VARIO_DOWN, vspd/25 ) ;
									}
								}
							}
						}
						else
						{
							if ( vspd > 25 )			// OpenXsensor
							{
								if ( sounded != 1 )
								{
									sounded = 1 ;
									varioRepeatRate = 0 ;
		  	        	audio.event( AU_VARIO_UP, vspd/25 ) ;
								}
							}
						}
						if ( vspd < 75 )
						{
							new_rate = 8 ;
						}
						else if ( vspd < 100 )
						{
							new_rate = 7 ;
						}
						else if ( vspd < 125 )
						{
							new_rate = 6 ;
						}
						else if ( vspd < 150 )
						{
							new_rate = 5 ;
						}
						else if ( vspd < 175 )
						{
							new_rate = 4 ;
						}
						else if ( vspd < 200 )
						{
							new_rate = 3 ;
						}
						else
						{
							new_rate = 2 ;
						}
					}
					else
					{
						if (g_model.varioData.sinkTones )
						{
							if ( sounded == 0 )
							{
								new_rate = 20 ;
								sounded = 3 ;
								varioRepeatRate = 0 ;
  	    	  		audio.event( AU_VARIO_UP, 0 ) ;
							}
						}
					}
					if ( varioRepeatRate == 0 )
					{
						varioRepeatRate = new_rate ;
					}
				}
			}
		}
	}

	if ( DsmCheckFlag )
	{
		DsmCheckFlag = 0 ;
		static uint8_t criticalCounter = 0 ;
		static uint8_t warningCounter = 0 ;

		// Check DSM telemetry for fades/frame losses here
		uint32_t fades ;
		fades = DsmABLRFH[0] + DsmABLRFH[1] + DsmABLRFH[2] + DsmABLRFH[3] ;
		uint32_t critical = 0 ;

		if ( g_model.dsmLinkData.levelCritical )
		{
			switch ( g_model.dsmLinkData.sourceCritical )
			{
				case 0 :	// fades
					if ( ( fades - LastDsmfades ) >= g_model.dsmLinkData.levelCritical )
					{
						critical = 1 ;
					}
				break ;
				case 1 :	// losses
					if ( ( DsmABLRFH[4] - LastDsmFH[0] ) >= g_model.dsmLinkData.levelCritical )
					{
						critical = 1 ;
					}
				break ;
				case 2 :	// holdss
					if ( ( DsmABLRFH[5] - LastDsmFH[1] ) >= g_model.dsmLinkData.levelCritical )
					{
						critical = 1 ;
					}
				break ;
			}
		}
		if ( critical == 0 )
		{
			if ( g_model.dsmLinkData.levelWarn )
			{
				uint32_t warning = 0 ;
				switch ( g_model.dsmLinkData.sourceWarn )
				{
					case 0 :	// fades
						if ( ( fades - LastDsmfades ) >= g_model.dsmLinkData.levelWarn )
						{
							warning = 1 ;
						}
					break ;
					case 1 :	// losses
						if ( ( DsmABLRFH[4] - LastDsmFH[0] ) >= g_model.dsmLinkData.levelWarn )
						{
							warning = 1 ;
						}
					break ;
					case 2 :	// holdss
						if ( ( DsmABLRFH[5] - LastDsmFH[1] ) >= g_model.dsmLinkData.levelWarn )
						{
							warning = 1 ;
						}
					break ;
				}
				if ( warning )
				{
					if ( warningCounter == 0 )
					{
						putSystemVoice( SV_RSSI_LOW, V_RSSI_WARN ) ;
						warningCounter = 11 ;
					}
				}
			}
		}
		else
		{
			if ( criticalCounter == 0 )
			{
				putSystemVoice( SV_RSSICRIT, V_RSSI_CRITICAL ) ;
				criticalCounter = 11 ;
			}
		}
		LastDsmfades = fades ;
		LastDsmFH[0] = DsmABLRFH[4] ;
		LastDsmFH[1] = DsmABLRFH[5] ;
		if ( criticalCounter )
		{
			criticalCounter -= 1 ;
		}
		if ( warningCounter )
		{
			warningCounter -= 1 ;
		}
	}
}

uint32_t check_power_or_usb()
{
	if ( check_soft_power() == POWER_OFF )		// power now off
	{
		return 1 ;
	}
	return 0 ;
}

void check_backlight()
{
	int8_t sw = g_model.mlightSw ;
	if ( !sw )
	{
		sw = g_eeGeneral.lightSw ;
	}
  if(getSwitch00(sw) || g_LightOffCounter)
	{
		BACKLIGHT_ON ;
	}
  else
	{
    BACKLIGHT_OFF ;
	}
}

uint16_t stickMoveValue()
{
    uint16_t sum = 0 ;
		uint8_t i ;
    for( i=0; i<4; i++)
		{
      sum += anaIn(i) ;
		}
	return sum ;
}

static uint32_t hasStickMoved( uint16_t value )
{
  if(abs(( (int16_t)value-(int16_t)stickMoveValue()))>160)
		return 1 ;
	return 0 ;
}

void doSplash()
{
	uint32_t j ;
	if( !g_eeGeneral.disableSplashScreen )
  {
   	check_backlight() ;
    lcd_clear();
		refreshDisplay();
    lcdSetRefVolt(g_eeGeneral.contrast);
  	clearKeyEvents();

#if defined(PCBX10)
		for ( uint32_t i = 0 ; i < 25 ; i += 1 )
		{
			getADC_filt();
		}
#else
		for ( uint32_t i = 0 ; i < 10 ; i += 1 )
		{
			getADC_filt();
		}
#endif
		getADC_filt();
  	uint16_t inacSum = stickMoveValue() ;

  	uint16_t tgtime = get_tmr10ms() + SPLASH_TIMEOUT;
  	uint16_t scrtime = get_tmr10ms() ;

		j = 62 ;
  	while(tgtime > get_tmr10ms())
  	{
			if ( scrtime < get_tmr10ms() )
			{
				scrtime += 4 ;
				uint8_t p ;
				uint8_t x ;
				uint8_t y ;
				uint8_t z ;
				lcd_clear();
// #if defined(PCBX12D) || defined(PCBX10)
//   	 		lcd_img(0 + X12OFFSET, 0, &splashdata[4],0,0, 0x03E0 );
// 				if(!g_eeGeneral.hideNameOnSplash)
// 					lcd_putsnAttColour( 0*FW + X12OFFSET, 7*FH, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName), 0, 0x03E0 ) ;
// #else
//   	 		lcd_img(0 + X12OFFSET, 0, &splashdata[4],0,0 );
// 				if(!g_eeGeneral.hideNameOnSplash)
// 					lcd_putsnAtt( 0*FW + X12OFFSET, 7*FH, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName),0);
// #endif

				if ( j )
				{
#if defined(PCBX12D) || defined(PCBX10)
					plotType = PLOT_BLACK ;
#else
					plotType = PLOT_WHITE ;
#endif
					p = 0 ;
					x = 126 ;
					z = 64 ;
					for ( y = 0 ; y < j ; y += 2 )
					{
						lcd_vline( y + X12OFFSET, p, z ) ;
						lcd_vline( 127-y + X12OFFSET, p, z ) ;
						lcd_rect( y+1 + X12OFFSET, p, x, z ) ;
						p += 1 ;
						z -= 2 ;
						x -= 4 ;
					}
					j -= 2 ;
					plotType = PLOT_XOR ;
				}

  			refreshDisplay();
			}

#ifdef SIMU
        if (!main_thread_running) return;
        sleep(1/*ms*/);
#else
        getADC_filt();
#endif

//			uint8_t xxx ;
			if ( ( /*xxx =*/ keyDown() ) )
			{
				return ;  //wait for key release
			}
			if ( hasStickMoved( inacSum ) )
			{
		   return ;  //wait for key release
			}

			if ( check_power_or_usb() )
			{
				return ;		// Usb on or power off
			}
			check_backlight() ;
  	  wdt_reset();
  	}
	}
}


//global helper vars
bool    checkIncDec_Ret;
struct t_p1 P1values ;
uint8_t LongMenuTimer ;
uint8_t StepSize = 20 ;

int16_t checkIncDec16( int16_t val, int16_t i_min, int16_t i_max, uint8_t i_flags)
{
  int16_t newval = val;
  uint8_t kpl=KEY_RIGHT, kmi=KEY_LEFT, kother = -1;
	uint8_t editAllowed = 1 ;

	if ( g_eeGeneral.forceMenuEdit && (s_editMode == 0) && ( i_flags & NO_MENU_ONLY_EDIT) == 0 )
	{
		editAllowed = 0 ;
	}

		uint8_t event = Tevent ;
//  if(event & _MSK_KEY_DBL){
//    uint8_t hlp=kpl;
//    kpl=kmi;
//    kmi=hlp;
//    event=EVT_KEY_FIRST(EVT_KEY_MASK & event);
//  }
  if(event==EVT_KEY_FIRST(kpl) || event== EVT_KEY_REPT(kpl) || (s_editMode && (event==EVT_KEY_FIRST(KEY_UP) || event== EVT_KEY_REPT(KEY_UP))) )
	{
		if ( editAllowed )
		{
#ifdef PCBXLITE
			if ((GPIOE->IDR & 0x0100) == 0 )
#else
			if ( menuPressed() )
#endif
			{
    		newval += StepSize ;
			}
			else
			{
    		newval += 1 ;
			}
			audioDefevent(AU_KEYPAD_UP);
    	kother=kmi;
		}

  }else if(event==EVT_KEY_FIRST(kmi) || event== EVT_KEY_REPT(kmi) || (s_editMode && (event==EVT_KEY_FIRST(KEY_DOWN) || event== EVT_KEY_REPT(KEY_DOWN))) )
	{
		if ( editAllowed )
		{
#ifdef PCBXLITE
			if ((GPIOE->IDR & 0x0100) == 0 )
#else
			if ( menuPressed() )
#endif
			{
    		newval -= StepSize ;
			}
			else
			{
    		newval -= 1 ;
			}
			audioDefevent(AU_KEYPAD_DOWN);
    	kother=kpl;
		}

  }
  if((kother != (uint8_t)-1) && keyState((EnumKeys)kother)){
    newval=-val;
    killEvents(kmi);
    killEvents(kpl);
  }
  if(i_min==0 && i_max==1 )
	{
		if ( (event==EVT_KEY_FIRST(KEY_MENU) || event==EVT_KEY_BREAK(BTN_RE)) )
		{
      s_editMode = false;
      newval=!val;
     	killEvents(EVT_KEY_FIRST(KEY_MENU)) ; // Allow Dbl for BTN_RE
			if ( event==EVT_KEY_BREAK(BTN_RE) )
			{
				RotaryState = ROTARY_MENU_UD ;
			}
			event = 0 ;
		}
		else
		{
			newval &= 1 ;
		}
	}
  if ( i_flags & INCDEC_SWITCH )
	{
		if ( s_editMode )
		{
    	int8_t swtch = getMovedSwitch();
    	if (swtch)
			{
	#if defined(PCBSKY) || defined(PCB9XT)
				swtch = switchUnMap( swtch ) ;
	#endif
    	  newval = swtch ;
    	}
		}
  }

  //change values based on P1
  newval -= P1values.p1valdiff;
	if ( RotaryState == ROTARY_VALUE )
	{
		newval += ( menuPressed() || encoderPressed() ) ? Rotary_diff * 20 : Rotary_diff ;
//#ifdef PCBX12D
//		Rotary_diff = 0 ;
//#endif
	}
  if(newval>i_max)
  {
    newval = i_max;
    killEvents(event);
    audioDefevent(AU_KEYPAD_UP);
  }
  else if(newval < i_min)
  {
    newval = i_min;
    killEvents(event);
    audioDefevent(AU_KEYPAD_DOWN);
  }
  if(newval != val)
	{
		if ( menuPressed() )
		{
			LongMenuTimer = 255 ;
		}
    if(newval==0)
		{
   	  pauseEvents(event);
			if (newval>val)
			{
				audioDefevent(AU_KEYPAD_UP);
			}
			else
			{
				audioDefevent(AU_KEYPAD_DOWN);
			}
    }
    eeDirty(i_flags & (EE_GENERAL|EE_MODEL));
    checkIncDec_Ret = true;
  }
  else {
    checkIncDec_Ret = false;
  }
	StepSize = 20 ;
  return newval;
}

int8_t checkIncDec( int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags)
{
  return checkIncDec16(i_val,i_min,i_max,i_flags);
}

int8_t checkIncDecSwitch( int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags)
{
	i_val = switchUnMap( i_val ) ;
  return switchMap( checkIncDec16(i_val,i_min,i_max,i_flags) ) ;
}

int8_t checkIncDec_hm( int8_t i_val, int8_t i_min, int8_t i_max)
{
  return checkIncDec(i_val,i_min,i_max,EE_MODEL);
}

int8_t checkIncDec_hm0( int8_t i_val, int8_t i_max)
{
  return checkIncDec(i_val,0,i_max,EE_MODEL);
}

int8_t checkIncDec_hg( int8_t i_val, int8_t i_min, int8_t i_max)
{
  return checkIncDec(i_val,i_min,i_max,EE_GENERAL);
}

int8_t checkIncDec_hg0( int8_t i_val, int8_t i_max)
{
  return checkIncDec(i_val,0,i_max,EE_GENERAL);
}

#if defined(USB_JOYSTICK) || defined(PCBSKY)
#ifndef SIMU
#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
extern USB_OTG_CORE_HANDLE USB_OTG_dev;
#endif
#ifdef PCBSKY
extern uint8_t HIDDJoystickDriver_Change( uint8_t *data ) ;
#define HID_IN_PACKET 9
#endif

/*
  Prepare and send new USB data packet

  The format of HID_Buffer is defined by
  USB endpoint description can be found in
  file usb_hid_joystick.c, variable HID_JOYSTICK_ReportDesc
*/

#ifdef USB_JOYSTICK
void usbJoystickUpdate(void)
{
  static uint8_t HID_Buffer[HID_IN_PACKET];

  //buttons
  HID_Buffer[0] = 0; //buttons
  for (int i = 0; i < 8; ++i) {
    if ( g_chans512[i+8] > 0 ) {
      HID_Buffer[0] |= (1 << i);
    }
  }

  //analog values
  for (int i = 0; i < 8; ++i) {
    int16_t value = g_chans512[i] / 8;
    if ( value > 127 ) value = 127;
    else if ( value < -127 ) value = -127;
    HID_Buffer[i+1] = static_cast<int8_t>(value);
  }

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
  USBD_HID_SendReport (&USB_OTG_dev, HID_Buffer, HID_IN_PACKET );
#endif
#ifdef PCBSKY
	HIDDJoystickDriver_Change( HID_Buffer ) ;
#endif
}
#endif
#endif
#endif


const static uint8_t rate[8] = { 0, 0, 100, 40, 16, 7, 3, 1 } ;
uint32_t calcStickScroll( uint32_t index )
{
	uint32_t direction ;
	int32_t value ;

	if ( ( g_eeGeneral.stickMode & 1 ) == 0 )
	{
		index ^= 3 ;
	}

	value = phyStick[index] ;
	value /= 8 ;
	direction = value > 0 ? 0x80 : 0 ;
	if ( value < 0 )
	{
		value = -value ;			// (abs)
	}
	if ( value > 7 )
	{
		value = 7 ;
	}
	value = rate[(uint8_t)value] ;
	if ( value )
	{
		StickScrollTimer = STICK_SCROLL_TIMEOUT ;		// Seconds
	}
	return value | direction ;
}

#if defined(PCBX12D) || defined(PCBX10)
extern uint8_t LastEvent ;
#endif
static void	processAdjusters()
{
static uint8_t GvAdjLastSw[NUM_GVAR_ADJUST + EXTRA_GVAR_ADJUST][2] ;
	for ( CPU_UINT i = 0 ; i < NUM_GVAR_ADJUST + EXTRA_GVAR_ADJUST ; i += 1 )
	{
		GvarAdjust *pgvaradj ;
		pgvaradj = ( i >= NUM_GVAR_ADJUST ) ? &g_model.egvarAdjuster[i - NUM_GVAR_ADJUST] : &g_model.gvarAdjuster[i] ;
		uint32_t idx = pgvaradj->gvarIndex ;

		int8_t sw0 = pgvaradj->swtch ;
		int8_t sw1 = 0 ;
		uint32_t switchedON = 0 ;
		int32_t value = g_model.gvars[idx].gvar ;
		if ( sw0 )
		{
			sw0 = getSwitch00(sw0) ;
			if ( !GvAdjLastSw[i][0] && sw0 )
			{
    		switchedON = 1 ;
			}
			GvAdjLastSw[i][0] = sw0 ;
		}
		if ( ( pgvaradj->function > 3 ) && ( pgvaradj->function < 7 ) )
		{
			sw1 = pgvaradj->switch_value ;
			if ( sw1 )
			{
				sw1 = getSwitch00(sw1) ;
				if ( !GvAdjLastSw[i][1] && sw1 )
				{
    			switchedON |= 2 ;
				}
				GvAdjLastSw[i][1] = sw1 ;
			}
		}

		switch ( pgvaradj->function )
		{
			case 1 :	// Add
				if ( switchedON & 1 )
				{
     			value += pgvaradj->switch_value ;
				}
			break ;

			case 2 :
				if ( switchedON & 1 )
				{
     			value = pgvaradj->switch_value ;
				}
			break ;

			case 3 :
				if ( switchedON & 1 )
				{
					if ( pgvaradj->switch_value == 5 )	// REN
					{
						value = RotaryControl ;	// Adjusted elsewhere
					}
					else
					{
						value = getGvarSourceValue( pgvaradj->switch_value ) ;
					}
				}
			break ;

			case 4 :
				if ( switchedON & 1 )
				{
     			value += 1 ;
				}
				if ( switchedON & 2 )
				{
     			value -= 1 ;
				}
			break ;

			case 5 :
				if ( switchedON & 1 )
				{
     			value += 1 ;
				}
				if ( switchedON & 2 )
				{
     			value = 0 ;
				}
			break ;

			case 6 :
				if ( switchedON & 1 )
				{
     			value -= 1 ;
				}
				if ( switchedON & 2 )
				{
     			value = 0 ;
				}
			break ;

			case 7 :
				if ( switchedON & 1 )
				{
     			value += 1 ;
					if ( value > pgvaradj->switch_value )
					{
						value = pgvaradj->switch_value ;
					}
				}
			break ;

			case 8 :
				if ( switchedON & 1 )
				{
     			value -= 1 ;
					if ( value < pgvaradj->switch_value )
					{
						value = pgvaradj->switch_value ;
					}
				}
			break ;

		}
  	if(value > 125)
		{
			value = 125 ;
		}
  	if(value < -125 )
		{
			value = -125 ;
		}
		g_model.gvars[idx].gvar = value ;
	}
}

#ifdef PCBX9D
uint8_t AnaEncSw = 0 ;
#endif

#ifdef PCBX9D
void valueprocessAnalogEncoder( uint32_t x )
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
	AnaEncSw = y & 4 ;
	y &= 3 ;

  uint32_t dummy ;

	dummy = y & 1 ;
	if ( y & 2 )
	{
		dummy |= 4 ;
	}
	if ( dummy != ( Rotary_position & 0x05 ) )
	{
		int32_t increment ;
		if ( ( Rotary_position & 0x01 ) ^ ( ( dummy & 0x04) >> 2 ) )
		{
			increment = -1 ;
		}
		else
		{
			increment = 1 ;
		}
		increment += Rotary_count ;
		if ( y == 3 )
		{
			if ( g_eeGeneral.rotaryDivisor == 1)
			{	// Rest position + div by 4
				if ( increment > 0 )
				{
					increment += 2 ;
					increment &= 0xFFFFFFFC ;
				}
				else
				{
					if ( ( increment & 3 ) != 3 )
					{
						increment &= 0xFFFFFFFC ;
					}
				}
			}
		}
		Rotary_count = increment ;
		Rotary_position = ( Rotary_position & ~0x45 ) | dummy ;
	}
}
#endif

uint8_t GvarSource[4] ;

int8_t getGvarSourceValue( uint8_t src )
{
	int16_t value ;

	if ( src >= EXTRA_POTS_START )
	{
		value = calibratedStick[src-EXTRA_POTS_START+7] / 8 ;
	}
	else if ( src <= 4 )
	{
		value = getTrimValue( CurrentPhase, src - 1 ) ;
		TrimInUse[src-1] |= 1 ;
		GvarSource[src-1] = 1 ;
	}
	else if ( src == 5 )	// REN
	{
		value = 0 ;
	}
	else if ( src <= 9 )	// Stick
	{
		value = calibratedStick[ src-5 - 1 ] / 8 ;
	}
#if defined(PCBSKY) || defined(PCB9XT)
	else if ( src <= 12 )	// Pot
#endif
#ifdef PCBX9D
	else if ( src <= 13 )	// Pot
#endif
#if defined(PCBX12D) || defined(PCBX10)
	else if ( src <= 13 )	// Pot
#endif
	{
		value = calibratedStick[ ( src-6)] / 8 ;
	}
#if defined(PCBSKY) || defined(PCB9XT)
	else if ( src <= 36 )	// Chans
#endif
#ifdef PCBX9D
	else if ( src <= 37 )	// Pot
#endif
#if defined(PCBX12D) || defined(PCBX10)
	else if ( src <= 37 )	// Pot
#endif
	{
#if defined(PCBSKY) || defined(PCB9XT)
		value = ex_chans[src-13] * 100 / 1024 ;
#endif
#ifdef PCBX9D
		value = ex_chans[src-14] * 100 / 1024 ;
#endif
#if defined(PCBX12D) || defined(PCBX10)
		value = ex_chans[src-14] * 100 / 1024 ;
#endif
	}
#if defined(PCBSKY) || defined(PCB9XT)
	else if ( src <= 44 )	// Scalers
#endif
#ifdef PCBX9D
	else if ( src <= 45 )	// Scalers
#endif
#if defined(PCBX12D) || defined(PCBX10)
	else if ( src <= 45 )	// Scalers
#endif
	{
#if defined(PCBSKY) || defined(PCB9XT)
		value = calc_scaler( src-37, 0, 0 ) ;
#endif
#ifdef PCBX9D
		value = calc_scaler( src-38, 0, 0 ) ;
#endif
#if defined(PCBX12D) || defined(PCBX10)
		value = calc_scaler( src-38, 0, 0 ) ;
#endif
	}
#if defined(PCBSKY) || defined(PCB9XT)
	else if ( src <= 68 )	// Scalers
#endif
#ifdef PCBX9D
	else if ( src <= 69 )	// Scalers
#endif
#if defined(PCBX12D) || defined(PCBX10)
	else if ( src <= 69 )	// Scalers
#endif
	{ // Outputs
		int32_t x ;
#if defined(PCBSKY) || defined(PCB9XT)
		x = g_chans512[src-45] ;
#endif
#ifdef PCBX9D
		x = g_chans512[src-46] ;
#endif
#if defined(PCBX12D) || defined(PCBX10)
		x = g_chans512[src-46] ;
#endif
		x *= 100 ;
		value = x / 1024 ;
	}
	else
	{
		value = 0 ;
	}
	if ( value > 125 )
	{
		value = 125 ;
	}
	if ( value < -125 )
	{
		value = -125 ;
	}
 	return value ;
}


#if defined(PCBX12D) || defined(PCBX10)
uint16_t TestTime ;
uint16_t TestNote ;

uint16_t ClearTime ;
uint16_t MenuTime ;
uint16_t Xcounter ;
#endif

#ifdef  LUA
extern char lua_warning_info[] ;
#endif

uint8_t ScriptActive ;


#ifdef PCBX9D
uint16_t MixerRunAtTime ;
#endif

void perMain( uint32_t no_menu )
{
  static uint16_t lastTMR;
	uint16_t t10ms ;

	t10ms = get_tmr10ms() ;
  tick10ms = ((uint16_t)(t10ms - lastTMR)) != 0 ;
  lastTMR = t10ms ;

	{
		MixerCount += 1 ;
		uint16_t t1 = getTmr2MHz() ;
#ifdef PCBX9D
		MixerRunAtTime = t1 ;
#endif
		perOutPhase(g_chans512, 0);
		t1 = getTmr2MHz() - t1 ;
		g_timeMixer = t1 ;
	}

	if(tick5ms)
	{
		check_frsky( 1 ) ;
		tick5ms = 0 ;

		if (!tick10ms)
		{
			// Run background script here
			basicTask( 0, SCRIPT_BACKGROUND ) ;
		}
	}

	if(!tick10ms) return ; //make sure the rest happen only every 10ms.

#ifdef SERIAL_HOST
	Host10ms = 1 ;
#endif
	if ( ppmInValid )
	{
		if ( --ppmInValid == 0 )
		{
			// Announce trainer lost
			putSystemVoice( SV_TRN_LOST, 0 ) ;
		}
	}

#ifdef PCBSKY
//extern void usbMassStorage( void ) ;
//	usbMassStorage() ;
#endif

#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX7) || defined (PCBX9LITE)
#ifdef BLUETOOTH
	if ( BtRxTimer )
	{
		BtRxTimer -= 1 ;
	}
#endif
#endif
	if ( SbusTimer )
	{
		SbusTimer -= 1 ;
	}

	heartbeat |= HEART_TIMER10ms;

#ifdef PCB9XT
	processAnalogSwitches() ;
#endif // PCB9XT
#if defined(PCBX12D) || defined(PCBX10)
		uint16_t t1 ;
		t1 = getTmr2MHz();
		TestTime = t1 - TestNote ;
		TestNote = t1 ;
#endif

#if defined(PCBX12D) || defined(PCBX10)
	uint8_t evt = 0 ;
	if ( ( lastTMR & 1 ) == 0 )
	{
		evt=getEvent();
  	evt = checkTrim(evt);
	}
#else
	uint8_t evt=getEvent();
  evt = checkTrim(evt);
#endif

#if defined(PCBX12D) || defined(PCBX10)
	if ( t10ms & 1 )
	{
		ledRed() ;
	}
	else
	{
		ledBlue() ;
	}
	if ( evt )
	{
		LastEvent = evt ;
	}
#endif

		if ( ( evt == 0 ) || ( evt == EVT_KEY_REPT(KEY_MENU) ) )
		{
			uint8_t timer = LongMenuTimer ;
			if ( menuPressed() )
			{
				if ( timer < 255 )
				{
					timer += 1 ;
				}
			}
			else
			{
				timer = 0 ;
			}
			if ( timer == 200 )
			{
				evt = EVT_TOGGLE_GVAR ;
				timer = 255 ;
			}
			LongMenuTimer = timer ;
		}


#if defined(PCBX12D) || defined(PCBX10)
	if ( ( lastTMR & 1 ) == 0 )
#endif
	{
		int32_t x ;
		if ( g_eeGeneral.rotaryDivisor == 1)
		{
			x = Rotary_count >> 2 ;
		}
		else if ( g_eeGeneral.rotaryDivisor == 2)
		{
			x = Rotary_count >> 1 ;
		}
		else
		{
			x = Rotary_count ;
		}
		Rotary_diff = x - LastRotaryValue ;
		LastRotaryValue = x ;
	}
#if defined(PCBX12D) || defined(PCBX10)
	else
	{
		Rotary_diff = 0 ;
	}
#endif

	{
		uint16_t a = 0 ;
		uint16_t b = 0 ;

		uint16_t lightoffctr ;
		lightoffctr = g_LightOffCounter ;

		if(lightoffctr) lightoffctr -= 1 ;
		if( evt | Rotary_diff )
		{
			a = g_eeGeneral.lightAutoOff*500 ; // on keypress turn the light on 5*100
			InacCounter = 0 ;
		}
		if(InactivityMonitor) b = g_eeGeneral.lightOnStickMove*500 ;
		if(a>lightoffctr) lightoffctr = a ;
		if(b>lightoffctr) lightoffctr = b ;
		g_LightOffCounter = lightoffctr ;
	}
	check_backlight() ;
// Handle volume
	uint8_t requiredVolume ;
	requiredVolume = g_eeGeneral.volume ;

	uint8_t option = g_menuStack[g_menuStackPtr] == menuProc0 ;
	if ( option && ( PopupData.PopupActive == 0 ) )
	{
		if ( Rotary_diff )
		{
			int16_t x = RotaryControl ;
			x += Rotary_diff ;
			if ( x > 125 )
			{
				RotaryControl = 125 ;
			}
			else if ( x < -125 )
			{
				RotaryControl = -125 ;
			}
			else
			{
				RotaryControl = x ;
			}

			// GVARS adjust
			for( uint8_t i = 0 ; i < MAX_GVARS ; i += 1 )
			{
				if ( g_model.gvars[i].gvsource == 5 )	// REN
				{
					if ( getSwitch( g_model.gvswitch[i], 1, 0 ) )
					{
						int16_t value = g_model.gvars[i].gvar + Rotary_diff ;
						g_model.gvars[i].gvar = limit( (int16_t)-125, value, (int16_t)125 ) ;
					}
			  }
			}
			Rotary_diff = 0 ;
		}
	}


	if ( g_eeGeneral.disablePotScroll || option )
	{
		if ( g_model.anaVolume )	// Only check if on main screen
		{
			static uint16_t oldVolValue ;
			uint16_t x ;
			uint16_t divisor ;
#if defined(PCBSKY) || defined(PCB9XT)
			if ( g_model.anaVolume < 4 )
#endif
#ifdef PCBX9D
#ifdef PCBX9LITE
		if ( g_model.anaVolume < 2 )
#else
#if defined(PCBX7) || defined (PCBXLITE)
		if ( g_model.anaVolume < 3 )
#else // PCBX7
		if ( g_model.anaVolume < 5 )
#endif // PCBX7
#endif // PCBX9LITE
#endif
#if defined(PCBX12D) || defined(PCBX10)
			if ( g_model.anaVolume < 5 )
#endif // PCBX12D
			{
				x = calibratedStick[g_model.anaVolume+3] + 1024 ;
				divisor = 2044 ;
			}
			else
			{
				x = g_model.gvars[g_model.anaVolume-1].gvar + 125 ;
				divisor = 249 ;
			}
			if ( abs( oldVolValue - x ) > 4 ) // (divisor/125  ) )
			{
				oldVolValue = x ;
			}
			else
			{
				x = oldVolValue ;
			}
			requiredVolume = x * (NUM_VOL_LEVELS-1) / divisor ;
		}
	}
	if ( HoldVolume )
	{
		requiredVolume = HoldVolume ;
	}
	if ( requiredVolume != CurrentVolume )
	{
		setVolume( requiredVolume ) ;
	}

#ifdef PCBX9D
  static uint8_t usbStarted = 0 ;
  if ( !usbStarted && usbPlugged() )
	{
    usbStarted = 1 ;
  }
#endif

	if ( g_eeGeneral.stickScroll )
	{
	 	if ( StickScrollTimer )
		{
			static uint8_t repeater ;
			uint32_t direction ;
			int32_t value ;

			if ( repeater < 128 )
			{
				repeater += 1 ;
			}
			value = calcStickScroll( 2 ) ;
			direction = value & 0x80 ;
			value &= 0x7F ;
			if ( value )
			{
		 		if ( StickScrollAllowed & 2 )
				{
					if ( repeater > value )
					{
						repeater = 0 ;
						if ( direction )
						{
							putEvent(EVT_KEY_FIRST(KEY_UP));
						}
						else
						{
							putEvent(EVT_KEY_FIRST(KEY_DOWN));
						}
					}
				}
			}
			else
			{
				value = calcStickScroll( 3 ) ;
				direction = value & 0x80 ;
				value &= 0x7F ;
				if ( value )
				{
			 		if ( StickScrollAllowed & 1 )
					{
						if ( repeater > value )
						{
							repeater = 0 ;
							if ( direction )
							{
								putEvent(EVT_KEY_FIRST(KEY_RIGHT));
							}
							else
							{
								putEvent(EVT_KEY_FIRST(KEY_LEFT));
							}
						}
					}
				}
			}
		}
	}
	else
	{
		StickScrollTimer = 0 ;		// Seconds
	}
	StickScrollAllowed = 3 ;

	GvarSource[0] = 0 ;
	GvarSource[1] = 0 ;
	GvarSource[2] = 0 ;
	GvarSource[3] = 0 ;
	for( uint8_t i = 0 ; i < MAX_GVARS ; i += 1 )
	{
		// ToDo, test for trim inputs here
		if ( g_model.gvars[i].gvsource )
		{
			int16_t value ;
			uint8_t src = g_model.gvars[i].gvsource ;
			if ( g_model.gvswitch[i] )
			{
				if ( !getSwitch00( g_model.gvswitch[i] ) )
				{
					continue ;
				}
			}

			if ( src == 5 )	// REN
			{
				value = g_model.gvars[i].gvar ;	// Adjusted elsewhere
			}
			else
			{
				value = getGvarSourceValue( src ) ;
			}
			g_model.gvars[i].gvar = limit( (int16_t)-125, value, (int16_t)125 ) ;
		}
	}

	check_frsky( 0 ) ;

// Here, if waiting for EEPROM response, don't action menus

#ifdef BT_WITH_ENCODER
	btEncTx() ;
#endif

	if ( no_menu == 0 )
	{
		static uint8_t alertKey ;
#if defined(PCBX12D) || defined(PCBX10)
//	 if ( ( lastTMR & 1 ) == 0 )
//	 {
//		uint16_t t1 ;
//		t1 = getTmr2MHz();
		waitLcdClearDdone() ;
#else
		if ( ScriptActive == 0 )
		{
	    lcd_clear() ;
		}
#endif

#if defined(PCBX12D) || defined(PCBX10)
//		t1 = getTmr2MHz() - t1 ;
//		if ( ++Xcounter > 50 )
//		{
//			ClearTime = t1 ;
//			Xcounter = 0 ;
//		}
//	 }
#endif

		if ( AlertMessage )
		{
			almess( AlertMessage, AlertType ) ;
#ifdef LUA
			if ( lua_warning_info[0] )
			{
				uint8_t i = 0 ;
				uint8_t j = 4*FH ;
				uint8_t c ;
				char *p = lua_warning_info ;
				while ( ( c = *p ) )
				{
					i = lcd_putc( i , j, c ) ;
					if ( i > 20*FW )
					{
						i = 0 ;
						j += FH ;
						if ( j > 7*FH )
						{
							break ;
						}
					}
					p += 1 ;
				}
  		  refreshDisplay() ;
			  wdt_reset() ;		//
			}
#endif
#ifdef BASIC
			if ( BasicErrorText[0] )
			{
				uint8_t i = 0 ;
				uint8_t j = 4*FH ;
				uint8_t c ;
				char *p = (char *)BasicErrorText ;
				while ( ( c = *p ) )
				{
					i = lcd_putc( i , j, c ) ;
					if ( i > 20*FW )
					{
						i = 0 ;
						j += FH ;
						if ( j > 7*FH )
						{
							break ;
						}
					}
					p += 1 ;
				}
  		  refreshDisplay() ;
			  wdt_reset() ;		//
			}
#endif

			uint8_t key = keyDown() ;
			killEvents( evt ) ;
			evt = 0 ;
			if ( alertKey )
			{
				if ( alertKey == 1 )
				{
					if( key == 0 )
					{
						alertKey = 2 ;
					}
				}
				else if ( alertKey == 2 )
				{
					if( key )
					{
						alertKey = 3 ;
					}
				}
				else
				{
					if( key == 0 )
					{
						AlertMessage = 0 ;
#ifdef LUA
						if ( lua_warning_info[0] )
						{
							lua_warning_info[0] ='\0' ;
						}
#endif
#ifdef BASIC
						if ( BasicErrorText[0] )
						{
							BasicErrorText[0] ='\0' ;
						}
#endif
					}
				}
			}
			else if ( key )
			{
				alertKey = 1 ;
			}
			else
			{
				alertKey = 2 ;
			}
		}
		else
		{
			alertKey = 0 ;

			if ( EnterMenu )
			{
				evt = EnterMenu ;
				EnterMenu = 0 ;
				audioDefevent(AU_MENUS);
			}
	 		StepSize = 20 ;
	 		Tevent = evt ;

#ifdef PCBX9D
			{
extern uint8_t ImageDisplay ;
extern uint8_t ImageX ;
extern uint8_t ImageY ;
				ImageX = 130 ;
				ImageY = 32 ;
				ImageDisplay = 1 ;
			}
#endif
#if defined(PCBX12D) || defined(PCBX10)
			{
extern uint8_t ImageDisplay ;
extern uint8_t ImageX ;
extern uint8_t ImageY ;
				ImageX = 130 ;
				ImageY = 32 ;
				ImageDisplay = 1 ;
			}
#endif

#ifdef WHERE_TRACK
	notePosition('a') ;
#endif

#if defined(LUA) || defined(BASIC)
  // if Lua standalone, run it and don't clear the screen (Lua will do it)
  					// else if Lua telemetry view, run it and don't clear the screen
  // else clear screen and show normal menus
//

//#ifdef PAGE_NAVIGATION
	int8_t evtAsRotary = 0 ;
	if ( evt == 0 )
	{
extern int32_t Rotary_diff ;
		if ( Rotary_diff > 0 )
		{
			evt = EVT_KEY_FIRST(KEY_DOWN) ;
			evtAsRotary = 1 ;
		}
		else if ( Rotary_diff < 0 )
		{
			evt = EVT_KEY_FIRST(KEY_UP) ;
			evtAsRotary = 1 ;
		}
	}
//#endif
		uint32_t refreshNeeded = 0 ;
		uint32_t telemetryScriptRunning = 0 ;
#ifdef LUA
		refreshNeeded = luaTask( evt, RUN_STNDAL_SCRIPT, true ) ;
  	if ( !refreshNeeded )
		{
    	refreshNeeded = luaTask( evt, RUN_TELEM_FG_SCRIPT, true ) ;
  	}
  	if ( refreshNeeded )
		{
			if (evtAsRotary)
			{
				Rotary_diff = 0 ;
			}
		}
#else
extern uint32_t TotalExecTime ;
		uint16_t execTime = getTmr2MHz() ;
		refreshNeeded = basicTask( evt, SCRIPT_LCD_OK	| SCRIPT_STANDALONE ) ;
		execTime = (uint16_t)(getTmr2MHz() - execTime) ;
		TotalExecTime += execTime ;
		if ( refreshNeeded == 3 )
		{
			refreshNeeded = 0 ;
			if ( sd_card_ready() )
			{
				evt = 0 ;
			}
			// standalone finished so:
			basicLoadModelScripts() ;
		}
		else
		{
	  	if ( !refreshNeeded )
			{
    		refreshNeeded = basicTask( PopupData.PopupActive ? 0 : evt, SCRIPT_TELEMETRY ) ;
				if ( refreshNeeded )
				{
					telemetryScriptRunning = 1 ;
				}
//				if ( refreshNeeded == 3 )
//				{
//					refreshNeeded = 0 ;
//					evt = 0 ;
//				}
	  	}
		}

//		refreshNeeded = basicTask( evt, RUN_STNDAL_SCRIPT, true ) ;
//  	if ( !refreshNeeded )
//		{
//    	refreshNeeded = basicTask( evt, RUN_TELEM_FG_SCRIPT, true ) ;
//  	}
		if ( refreshNeeded )
		{
			ScriptActive = 1 ;
//			lcd_puts_Pleft( 5*FH, (char *)BasicErrorText ) ;
			if (evtAsRotary)
			{
				Rotary_diff = 0 ;
			}

			if ( PopupData.PopupActive )
			{
//				Tevent = evt ;
				actionMainPopup( evt ) ;
			}
			else
			{
				if ( telemetryScriptRunning )
				{
					if ( ( evt==EVT_KEY_LONG(KEY_MENU)) || ( evt==EVT_KEY_LONG(BTN_RE) ) )
					{
						PopupData.PopupActive = 3 ;
						PopupData.PopupIdx = 0 ;
      			killEvents(evt) ;
						evt = 0 ;
						Tevent = 0 ;
					}
				}
			}
		}
#endif
  	else
#endif
		{
			ScriptActive = 0 ;
#if defined(LUA) || defined(BASIC)
			if (evtAsRotary)
			{
				evt = 0 ;
			}
#endif

#if defined(PCBX12D) || defined(PCBX10)
			t1 = getTmr2MHz();
			updatePicture() ;
#endif

			g_menuStack[g_menuStackPtr](evt);
			refreshNeeded = 4 ;
#if defined(PCBX12D) || defined(PCBX10)
			t1 = getTmr2MHz() - t1 ;
			if ( Xcounter == 0 )
			{
				MenuTime = t1 ;
			}
#endif
		}


	#if defined(PCBX9D) || defined(PCBSKY) || defined(PCB9XT)
			if ( ( refreshNeeded == 2 ) || ( ( refreshNeeded == 4 ) && ( ( lastTMR & 3 ) == 0 ) ) )
	#endif
#if defined(PCBX12D) || defined(PCBX10)
			if ( refreshNeeded != 1 )
	#endif
			{
#if defined(PCBX12D) || defined(PCBX10)
				displayStatusLine(ScriptActive) ;
				ScriptActive = 0 ;
	#endif
				uint16_t t1 = getTmr2MHz() ;
  		  refreshDisplay() ;
#if defined(PCBX12D) || defined(PCBX10)
				lcd_clearBackground() ;	// Start clearing other frame
	#endif
				t1 = getTmr2MHz() - t1 ;
				g_timeRfsh = t1 ;
			}
		}
	}


  switch( get_tmr10ms() & 0x1f )
	{ //alle 10ms*32

    case 2:
//        //check v-bat
////        Calculation By Mike Blandford
////        Resistor divide on battery voltage is 5K1 and 2K7 giving a fraction of 2.7/7.8
////        If battery voltage = 10V then A2D voltage = 3.462V
////        11 bit A2D count is 1417 (3.462/5*2048).
////        1417*18/256 = 99 (actually 99.6) to represent 9.9 volts.
////        Erring on the side of low is probably best.

        int32_t ab = anaIn(12);

        ab = ( ab + ab*(g_eeGeneral.vBatCalib)/128 ) * 4191 ;
//        ab = (uint16_t) ab / (g_eeGeneral.disableBG ? 240 : BandGap ) ;  // ab might be more than 32767
#ifdef PCBSKY
        ab /= 55296  ;
        g_vbat100mV = ( (ab + g_vbat100mV + 1) >> 1 ) + 3 ;  // Filter it a bit => more stable display
								// Also add on 0.3V for voltage drop across input diode
#endif
#ifdef PCBX9D
 #if defined(PCBX9LITE)
        ab /= 71089  ;
 #else
  #if defined(PCBXLITE)
        ab /= 64626  ;
  #else
        ab /= 57165  ;
  #endif
 #endif
        g_vbat100mV = ( (ab + g_vbat100mV + 1) >> 1 ) ;  // Filter it a bit => more stable display
#endif
#ifdef PCB9XT
        ab /= 64535  ;
        g_vbat100mV = ( (ab + g_vbat100mV + 1) >> 1 ) ;  // Filter it a bit => more stable display
#endif
#if defined(PCBX12D) || defined(PCBX10)
        ab /= 68631  ;
        g_vbat100mV = ( (ab + g_vbat100mV + 1) >> 1 ) ;  // Filter it a bit => more stable display
#endif

        static uint8_t s_batCheck;
        s_batCheck+=16;
        if(s_batCheck==0)
				{
					if( (g_vbat100mV<g_eeGeneral.vBatWarn) && (g_vbat100mV>49) )
					{
						voiceSystemNameNumberAudio( SV_TXBATLOW, V_BATTERY_LOW, AU_TX_BATTERY_LOW ) ;
            if (g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
					}
#ifdef PCBSKY
 #ifndef ARUNI
					else if ( ( g_eeGeneral.mAh_alarm ) && ( ( MAh_used + Current_used/3600 ) / 500 >= g_eeGeneral.mAh_alarm ) )
					{
						if ( ( g_eeGeneral.ar9xBoard == 0 ) && ( g_eeGeneral.extraPotsSource[0] != 2 ) && ( g_eeGeneral.extraPotsSource[1] != 2 ) )
						{
							voiceSystemNameNumberAudio( SV_TXBATLOW, V_BATTERY_LOW, AU_TX_BATTERY_LOW ) ;
						}
					}
 #endif
#endif
        }
    break ;

  }
  InactivityMonitor = 0; //reset this flag

	AUDIO_HEARTBEAT();  // the queue processing

}

extern uint8_t UserTimer1 ;

void interrupt5ms()
{
	static uint32_t pre_scale ;		// Used to get 10 Hz counter

	sound_5ms() ;
	tick5ms = 1 ;
	if ( ++pre_scale >= 2 )
	{
		PowerOnTime += 1 ;
		Tenms |= 1 ;			// 10 mS has passed
		pre_scale = 0 ;
		if ( UserTimer1 )
		{
			UserTimer1 -= 1 ;
		}

  	per10ms();
		if (--AlarmTimer == 0 )
		{
			AlarmTimer = 100 ;		// Restart timer
			AlarmCheckFlag += 1 ;	// Flag time to check alarms
		}
		if ( --CheckTimer == 0 )
		{
			CheckTimer = 2 ;
			__disable_irq() ;
			CheckFlag20mS = 1 ;
			__enable_irq() ;
		}

		if (--VoiceTimer == 0 )
		{
			VoiceTimer = 10 ;		// Restart timer
//			CheckFlag50mS = 1 ;
			__disable_irq() ;
			VoiceCheckFlag100mS |= 1 ;	// Flag time to check alarms
			__enable_irq() ;
		}

		if (--DsmCheckTimer == 0 )
		{
			DsmCheckTimer = 50 ;		// Restart timer
			DsmCheckFlag |= 1 ;	// Flag time to check alarms
		}
	}
}




// For SKY board
// ADC channels are assigned to:
// AD1  stick_RH
// AD2  stick_LH
// AD3  PI#T_TRIM
// AD4  battery
// AD5  HOV_PIT
// AD9  stick_LV
// AD13 HOV_THR
// AD14 stick_RV
// AD15 Chip temperature
// Peripheral ID 29 (0x20000000)
// Note ADC sequencing won't work as it only operates on channels 0-7
//      and we need 9, 13 and 14 as well
// ALSO: Errata says only one channel converted for each trigger

// Needed implementation (11 bit result)
// SINGLE - 1 read then >> 1
// OVERSAMPLE - 4 reads - average = sum then >> 3
// FILTERED - 1 read but filter processing

// Filtering algorithm
// o/p = half previous o/p + temp1
// temp1 = average temp1 and temp0
// temp0 = average new reading and temp0

uint16_t LastAnaIn[4] ; //[NUMBER_ANALOG+NUM_EXTRA_ANALOG] ;

#ifndef SIMU
uint16_t anaIn(uint8_t chan)
{

#ifdef PCB9XT
  volatile uint16_t *p = &S_anaFilt[chan] ;
	if ( ( chan >= 7 ) && ( chan <= 10 ) )
	{
		uint32_t x = 7 ;
		if ( g_eeGeneral.extraPotsSource[chan-7] )
		{
			x += g_eeGeneral.extraPotsSource[chan-7] - 1 ;
		}
		p = &S_anaFilt[x] ;
	}
#endif
#ifdef PCBSKY
  volatile uint16_t *p = &S_anaFilt[chan] ;
	if ( ( chan >= 7 ) && ( chan <= 8 ) )
	{
		uint32_t x = 7 ;
		if ( g_eeGeneral.extraPotsSource[chan-7] )
		{
			x += g_eeGeneral.extraPotsSource[chan-7] - 1 ;
		}
		p = &S_anaFilt[x] ;
	}
#endif
#ifdef PCBX9D
  volatile uint16_t *p = &S_anaFilt[chan] ;
#endif
#if defined(PCBX12D) || defined(PCBX10)
  volatile uint16_t *p = &S_anaFilt[chan] ;
#endif
  uint16_t temp = *p ;
  int16_t t1 ;
	if ( chan < 4 )	// A stick
	{
		if ( g_eeGeneral.stickReverse & ( 1 << chan ) )
		{
			temp = 2048 - temp ;
		}
		t1 = temp - LastAnaIn[chan] ;
		if ( ( t1 < 2 ) && ( t1 > -2 ) )
		{
			temp = LastAnaIn[chan] ;
		}
		else
		{
			LastAnaIn[chan] = temp ;
		}
	}
  return temp ;
}
#endif

uint32_t getAnalogIndex( uint32_t index )
{
	uint32_t z = index ;
	if ( index == 8 )
	{
		if ( g_eeGeneral.ar9xBoard == 1 )
		{
			if ( g_eeGeneral.extraPotsSource[0] == 1 )
			{
				z = 9 ;
			}
		}
		else
		{
			z = 9 ;
		}
	}
	return z ;
}


#ifdef ARUNI
static uint8_t qSixPosDelayFiltering(uint8_t x, uint16_t y)
{
  if (qSixPosCalibrating() || g_eeGeneral.sixPosDelayFilter) {
    uint8_t sixpos = (( g_eeGeneral.analogMapping & MASK_6POS ) >> 2) + 3;
    if (x == sixpos) {
      uint8_t v7 = (y >> 4);  // 11b to 7b truncation
      uint8_t diff;
      if (v7 > SixPosValue) {
        diff = v7 - SixPosValue;
      } else {
        diff = SixPosValue - v7;
      }
      if (diff > 1) {         // sixpos value changed
        SixPosValue = v7;     // save truncated 7b analog value
        SixPosDelay = 25;     // 25x10ms=250ms delay filtering
      }                       // ala retriggerable one-shot
      return 1;
    }
  }
  return 0;
}
#endif

void getADC_single()
{
	register uint32_t x ;
	uint16_t temp ;
	uint32_t numAnalog = ANALOG_DATA_SIZE ;

	read_adc() ;

	for( x = 0 ; x < numAnalog ; x += 1 )
	{
		temp = AnalogData[x] >> 1;
#ifdef ARUNI
    if (qSixPosDelayFiltering(x, temp))
      continue;
#endif
		S_anaFilt[x] = temp ;
	}
}

#if defined(PCBSKY) || defined(PCB9XT)
#define OSMP_SAMPLES	4
#define OSMP_TOTAL		16384
#define OSMP_SHIFT		3
#endif
#ifdef PCBX9D
#define OSMP_SAMPLES	4
#define OSMP_TOTAL		16384
#define OSMP_SHIFT		3
//#define OSMP_SAMPLES	8
//#define OSMP_TOTAL		32768
//#define OSMP_SHIFT		4
#endif
#if defined(PCBX12D) || defined(PCBX10)
#define OSMP_SAMPLES	4
#define OSMP_TOTAL		16384
#define OSMP_SHIFT		3
#endif

void getADC_osmp()
{
	register uint32_t x ;
	register uint32_t y ;
	uint32_t numAnalog = ANALOG_DATA_SIZE ;

	uint16_t temp[ANALOG_DATA_SIZE] ;
	static uint16_t next_ana[ANALOG_DATA_SIZE] ;

#ifdef ARUNI
  uint32_t osmp_samples = OSMP_SAMPLES ;
  uint32_t osmp_shift = OSMP_SHIFT ;
	if ( g_eeGeneral.filterInput == 2 )
	{
    osmp_samples *= 2 ;
    osmp_shift += 1 ;
  }
#endif

	for( x = 0 ; x < numAnalog ; x += 1 )
	{
		temp[x] = 0 ;
	}
#ifdef ARUNI
	for( y = 0 ; y < osmp_samples ; y += 1 )
#else
	for( y = 0 ; y < OSMP_SAMPLES ; y += 1 )
#endif
	{
		read_adc() ;

		for( x = 0 ; x < numAnalog ; x += 1 )
		{
			temp[x] += AnalogData[x] ;
		}
	}
	for( x = 0 ; x < ANALOG_DATA_SIZE ; x += 1 )
	{
#ifdef ARUNI
		uint16_t y = temp[x] >> osmp_shift ;
    if (qSixPosDelayFiltering(x, y))
      continue;
#else
		uint16_t y = temp[x] >> OSMP_SHIFT ;
#endif
		uint16_t z = S_anaFilt[x] ;
		uint16_t w = next_ana[x] ;

		int16_t diff = abs( (int16_t) y - z ) ;

		next_ana[x] = y ;
		if ( diff < 10 )
		{
			if ( y > z )
			{
				if ( w > z )
				{
					y = z + 1 ;
				}
				else
				{
					y = z ;
				}
			}
			else if ( y < z )
			{
				if ( w < z )
				{
					y = z - 1 ;
				}
				else
				{
					y = z ;
				}
			}
		}
		S_anaFilt[x] = y ;
	}
}

void getADC_filt()
{
	register uint32_t x ;
	static uint16_t t_ana[2][ANALOG_DATA_SIZE] ;
	uint16_t temp ;
	uint32_t numAnalog = ANALOG_DATA_SIZE ;


	read_adc() ;
	for( x = 0 ; x < numAnalog ; x += 1 )
	{
    uint16_t y = AnalogData[x];
#ifdef ARUNI
    if (qSixPosDelayFiltering(x, y >> 1))
      continue;
#endif
		temp = S_anaFilt[x] ;
		temp = temp/2 + (t_ana[1][x] >> 2 ) ;
		S_anaFilt[x] = temp ;
		t_ana[1][x] = ( t_ana[1][x] + t_ana[0][x] ) >> 1 ;
		t_ana[0][x] = ( t_ana[0][x] + y ) >> 1 ;
	}
}

uint32_t getFlightPhase()
{
	uint32_t i ;
  for ( i = 0 ; i < MAX_MODES ; i += 1 )
	{
    PhaseData *phase = &g_model.phaseData[i];
    if ( phase->swtch )
		{
    	if ( getSwitch00( phase->swtch ) )
			{
    		if ( phase->swtch2 )
				{
					if ( getSwitch00( phase->swtch2 ) )
					{
						return i + 1 ;
					}
				}
				else
				{
					return i + 1 ;
				}
    	}
		}
		else
		{
    	if ( phase->swtch2 && getSwitch00( phase->swtch2 ) )
			{
    	  return i + 1 ;
    	}
		}
  }
  return 0 ;
}

int16_t getRawTrimValue( uint8_t phase, uint8_t idx )
{
	if ( phase )
	{
		return g_model.phaseData[phase-1].trim[idx] ;
	}
	else
	{
		return g_model.trim[idx] ;
	}
}

uint32_t getTrimFlightPhase( uint8_t phase, uint8_t idx )
{
  for ( uint32_t i=0 ; i<MAX_MODES ; i += 1 )
	{
    if (phase == 0) return 0;
    int16_t trim = getRawTrimValue( phase, idx ) ;
    if ( trim <= TRIM_EXTENDED_MAX )
		{
			return phase ;
		}
    uint32_t result = trim-TRIM_EXTENDED_MAX-1 ;
    if (result >= phase)
		{
			result += 1 ;
		}
    phase = result;
  }
  return 0;
}


int16_t getTrimValue( uint8_t phase, uint8_t idx )
{
  return getRawTrimValue( getTrimFlightPhase( phase, idx ), idx ) ;
}

void setTrimValue(uint8_t phase, uint8_t idx, int16_t trim)
{
	if ( phase )
	{
		phase = getTrimFlightPhase( phase, idx ) ;
	}
	if ( phase )
	{
    if(trim < -125 || trim > 125)
//    if(trim < -500 || trim > 500)
		{
			trim = ( trim > 0 ) ? 125 : -125 ;
//			trim = ( trim > 0 ) ? 500 : -500 ; For later addition
		}
  	g_model.phaseData[phase-1].trim[idx] = trim ;
	}
	else
	{
    if(trim < -125 || trim > 125)
		{
			trim = ( trim > 0 ) ? 125 : -125 ;
		}
		g_model.trim[idx] = trim ;
	}
  STORE_MODELVARS_TRIM ;
}

//uint8_t TrimBits ;

static uint8_t checkTrim(uint8_t event)
{
  int8_t  k = (event & EVT_KEY_MASK) - TRM_BASE;
  int8_t  s = g_model.trimInc;

		if ( s == 4 )
		{
			s = 8 ;			  // 1=>1  2=>2  3=>4  4=>8
		}
		else
		{
			if ( s == 3 )
			{
				s = 4 ;			  // 1=>1  2=>2  3=>4  4=>8
			}
		}

  if( (k>=0) && (k<8) )
	{
		if ( !IS_KEY_BREAK(event)) // && (event & _MSK_KEY_REPT))
  	{
//			TrimBits |= (1 << k ) ;

  	  //LH_DWN LH_UP LV_DWN LV_UP RV_DWN RV_UP RH_DWN RH_UP
  	  uint8_t idx = k/2;

	// SORT idx for stickmode if FIX_MODE on
			idx = stickScramble[g_eeGeneral.stickMode*4+idx] ;
			uint8_t ct = g_eeGeneral.crosstrim + ( g_eeGeneral.xcrosstrim << 1 ) ;
			if ( ct )
			{
				idx = 3 - idx ;
			}
        if ( ct == 2 ) // Vintage style crosstrim
        {
          if (idx == 0 || idx == 3)       // swap back LH/RH trims
            idx = 3 - idx;
        }
			if ( TrimInUse[idx] )
			{
				uint32_t phaseNo = getTrimFlightPhase( CurrentPhase, idx ) ;
  	  	int16_t tm = getTrimValue( phaseNo, idx ) ;
  	  	int8_t  v = (s==0) ? (abs(tm)/4)+1 : s;
  	  	bool thrChan = (2 == idx) ;
				bool thro = (thrChan && (g_model.thrTrim));
  	  	if(thro) v = 2; // if throttle trim and trim trottle then step=2

				if ( GvarSource[idx] )
				{
					v = 1 ;
				}

  	  	if(thrChan && throttleReversed()) v = -v;  // throttle reversed = trim reversed
  	  	int16_t x = (k&1) ? tm + v : tm - v;   // positive = k&1

  	  	if(((x==0)  ||  ((x>=0) != (tm>=0))) && (!thro) && (tm!=0))
				{
					setTrimValue( phaseNo, idx, 0 ) ;
  	  	  killEvents(event);
  	  	  audioDefevent(AU_TRIM_MIDDLE);
  	  	}
				else if(x>-125 && x<125)
				{
					setTrimValue( phaseNo, idx, x ) ;
					if(x <= 125 && x >= -125)
					{
						audio.event(AU_TRIM_MOVE,(abs(x)/4)+60);
					}
  	  	}
  	  	else
  	  	{
					setTrimValue( phaseNo, idx, (x>0) ? 125 : -125 ) ;
					if(x <= 125 && x >= -125)
					{
						audio.event(AU_TRIM_MOVE,(-abs(x)/4)+60);
					}
  	  	}
			}
  	  return 0;
  	}
//		else
//		{
//			TrimBits &= ~(1 << k ) ;
//		}
	}
  return event;
}

char LastItem[8] ;

void setLastIdx( char *s, uint8_t idx )
{
	uint8_t length ;
	length = (uint8_t) *s++ ;

	ncpystr( (uint8_t *)LastItem, (uint8_t *)s+length*idx, length ) ;
}

void setLastTelemIdx( uint8_t idx )
{
	uint8_t *s ;
	uint32_t x ;
	if ( ( idx >= 69 ) && ( idx <= 74 ) ) // A Custom sensor
	{
		x = idx - 69 ;
		x *= 4 ;
		s = &g_model.customTelemetryNames[x] ;
		if ( *s && (*s != ' ' ) )
		{
			ncpystr( (uint8_t *)LastItem, s, 4 ) ;
			return ;
		}
	}
	if ( ( idx >= 38 ) && ( idx <= 45 ) )	// A Scaler
	{
		x = idx - 38 ;
		s = g_model.Scalers[x].name ;
		if ( *s && (*s != ' ' ) )
		{
			ncpystr( (uint8_t *)LastItem, s, 4 ) ;
			return ;
		}
	}
	setLastIdx( (char *) PSTR(STR_TELEM_ITEMS), idx ) ;
}

void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att)
{
	uint8_t chanLimit = NUM_SKYXCHNRAW ;
	uint8_t mix = att & MIX_SOURCE ;
	LastItem[0] = '\0' ;
	if ( mix )
	{
		chanLimit += MAX_GVARS + 1 + 1 ;
		att &= ~MIX_SOURCE ;
	}
  if(idx==0)
		ncpystr( (uint8_t *)LastItem, (uint8_t *) XPSTR("----"), 4 ) ;
  else if(idx<=4)
	{
		const char *ptr = "" ;
		if ( g_model.useCustomStickNames )
		{
			ptr = ( char *)g_eeGeneral.customStickNames+4*(idx-1) ;
		}
		if ( *ptr && (*ptr != ' ' ) )
		{
			ncpystr( (uint8_t *)LastItem, (uint8_t *)ptr, 4 ) ;
		}
		else
		{
			setLastIdx( (char *) PSTR(STR_STICK_NAMES), idx-1 ) ;
		}
	}
  else if(idx<=chanLimit)
		setLastIdx( (char *) PSTR(STR_CHANS_GV), idx-5 ) ;
	else if(idx < EXTRA_POTS_START)
	{
		if ( mix )
		{
			idx += TEL_ITEM_SC1-(chanLimit-NUM_SKYXCHNRAW) ;
			if ( idx - NUM_SKYXCHNRAW > TEL_ITEM_SC1 + NUM_SCALERS )
			{
				uint8_t *ptr ;
				idx -= TEL_ITEM_SC1 + NUM_SCALERS - 8 + NUM_SKYXCHNRAW ;
#if EXTRA_SKYCHANNELS
				if ( idx > 16 )
				{
					if ( idx <= 16 + 8  )
					{
						idx += 8 ;
						ptr = cpystr( (uint8_t *)LastItem, (uint8_t *)"CH" ) ;
						*ptr++ = (idx / 10) + '0' ;
						*ptr = (idx % 10) + '0' ;
					}
					else
					{
						ptr = ncpystr( (uint8_t *)LastItem, (uint8_t *)&PSTR(STR_GV_SOURCE)[1+3*(idx-24)], 3 ) ;
					}
				}
				else
#endif
				{
					ptr = cpystr( (uint8_t *)LastItem, (uint8_t *)"PPM1" ) ;
					if ( idx == 9 )
					{
						*(ptr-1) = '9' ;
					}
					else
					{
						*ptr = '0' + idx - 10 ;
					}
				}
				*(ptr+1) = '\0' ;
				lcd_putsAtt(x,y,LastItem,att);
				return ;
			}
		}
		setLastTelemIdx( idx-NUM_SKYXCHNRAW ) ;
//		setLastIdx( (char *) PSTR(STR_TELEM_ITEMS), idx-NUM_SKYXCHNRAW ) ;
	}
	else
	{
		if(idx < EXTRA_POTS_START + 8)
		{
			setLastIdx( (char *) PSTR(STR_CHANS_EXTRA), idx-EXTRA_POTS_START ) ;
		}
		else
		{
			// Extra telemetry items
			setLastTelemIdx( idx-NUM_SKYXCHNRAW-8 ) ;
//			setLastIdx( (char *) PSTR(STR_TELEM_ITEMS), idx-NUM_SKYXCHNRAW-8 ) ;

		}
	}
	lcd_putsAtt(x,y,LastItem,att);
}

void putsChn(uint8_t x,uint8_t y,uint8_t idx1,uint8_t att)
{
	if ( idx1 == 0 )
	{
    lcd_putsnAtt(x,y,XPSTR("--- "),4,att);
	}
	else
	{
		uint8_t x1 ;
		x1 = x + 4*FW-2 ;
		if ( idx1 < 10 )
		{
			x1 -= FWNUM ;
		}
  	lcd_outdezAtt(x1,y,idx1,att);
    lcd_putsnAtt(x,y,PSTR(STR_CH),2,att);
	}
}

#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
uint8_t MaxSwitchIndex = MAX_SKYDRSWITCH ;		// For ON and OFF
#endif


#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
uint8_t switchMapTable[100] ;
uint8_t switchUnMapTable[100] ;

// So, I think I map SA0/1/2 to ELE 3-pos, SC0/1/2 to ID0/1/2, SD0/1/2 to AIL 3-pos,
// SE0/1/2 to RUD 3-pos, SG0/1/2 to GEA 3-pos, SB0/1/2 to THR 3-pos,
// SF0/2 to THR/RUD and SH0/2 to GEA/TRN.

void createSwitchMapping()
{
	uint8_t *p = switchMapTable ;

	*p++ = 0 ;
	*p++ = HSW_SA0 ;
	*p++ = HSW_SA1 ;
	*p++ = HSW_SA2 ;

	*p++ = HSW_SB0 ;
	*p++ = HSW_SB1 ;
	*p++ = HSW_SB2 ;

	*p++ = HSW_SC0 ;
#ifdef PCBXLITE
	if (g_eeGeneral.ailsource)
	{
#endif
	*p++ = HSW_SC1 ;
#ifdef PCBXLITE
	}
#endif
	*p++ = HSW_SC2 ;

#ifndef PCBX9LITE
	*p++ = HSW_SD0 ;
 #ifdef PCBXLITE
	if (g_eeGeneral.rudsource)
	{
 #endif
	*p++ = HSW_SD1 ;
 #ifdef PCBXLITE
	}
 #endif
	*p++ = HSW_SD2 ;
#endif // nX3

#ifndef PCBX9LITE
#ifndef PCBX7
#ifndef PCBXLITE
	*p++ = HSW_SE0 ;
	*p++ = HSW_SE1 ;
	*p++ = HSW_SE2 ;
#endif
#endif
#endif // nX3

#ifndef PCBXLITE
	*p++ = HSW_SF2 ;
#endif

#ifndef PCBX9LITE
#ifndef PCBX7
#ifndef PCBXLITE
	*p++ = HSW_SG0 ;
	*p++ = HSW_SG1 ;
	*p++ = HSW_SG2 ;
#endif
#endif
#endif // nX3

#ifndef PCBXLITE
	*p++ = HSW_SH2 ;
#endif

	if ( g_eeGeneral.switchMapping & USE_PB1 )
	{
		*p++ = HSW_Pb1 ;
	}
	if ( g_eeGeneral.switchMapping & USE_PB2 )
	{
		*p++ = HSW_Pb2 ;
	}

#ifndef PCBX12D
 #ifndef PCBX10
	if ( g_eeGeneral.analogMapping & MASK_6POS )
 #endif
#endif
	{
		*p++ = HSW_Ele6pos0 ;
		*p++ = HSW_Ele6pos1 ;
		*p++ = HSW_Ele6pos2 ;
		*p++ = HSW_Ele6pos3 ;
		*p++ = HSW_Ele6pos4 ;
		*p++ = HSW_Ele6pos5 ;
	}

#if defined(PCBX12D) || defined(PCBX10)
	*p++ = HSW_Pb1 ;
	*p++ = HSW_Pb2 ;
	*p++ = HSW_Pb3 ;
	*p++ = HSW_Pb4 ;
#endif
	*p++ = HSW_Ttrmup ;
	*p++ = HSW_Ttrmdn ;
	*p++ = HSW_Rtrmup ;
	*p++ = HSW_Rtrmdn ;
	*p++ = HSW_Atrmup ;
	*p++ = HSW_Atrmdn ;
	*p++ = HSW_Etrmup ;
	*p++ = HSW_Etrmdn ;

	for ( uint32_t i = 10 ; i <=33 ; i += 1  )
	{
		*p++ = i ;	// Custom switches
	}
	*p = MAX_SKYDRSWITCH ;
	MaxSwitchIndex = p - switchMapTable ;
	*++p = MAX_SKYDRSWITCH+1 ;
	*++p = MAX_SKYDRSWITCH+2 ;

	for ( uint32_t i = 0 ; i <= (uint32_t)MaxSwitchIndex+2 ; i += 1  )
	{
		switchUnMapTable[switchMapTable[i]] = i ;
	}
}

#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
void create6posTable()
{
	uint32_t i ;

	for ( i = 0 ; i < 5 ; i += 1 )
	{
		uint32_t j ;
		j = (g_eeGeneral.SixPositionCalibration[i+1] + g_eeGeneral.SixPositionCalibration[i]) / 2 ;
		if ( g_eeGeneral.SixPositionCalibration[5] >  g_eeGeneral.SixPositionCalibration[0] )
		{
			j = 4095 - j ;
		}
		SixPositionTable[i] = j ;
	}
}
#endif

#endif

int8_t switchUnMap( int8_t x )
{
	uint8_t sign = 0 ;
	if ( x < 0 )
	{
		sign = 1 ;
		x = -x ;
	}
	x = switchUnMapTable[x] ;
	if ( sign )
	{
		x = -x ;
	}
	return x ;
}

int8_t switchMap( int8_t x )
{
	uint8_t sign = 0 ;
	if ( x < 0 )
	{
		sign = 1 ;
		x = -x ;
	}
	x = switchMapTable[x] ;
	if ( sign )
	{
		x = -x ;
	}
	return x ;
}


void putsMomentDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att)
{
  int16_t tm = idx1 ;
#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
	if ( tm < -HSW_MAX )
	{
		tm += 256 ;
	}
#endif
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
  if(abs(tm)>(HSW_MAX))	 //momentary on-off
#endif
#if defined(PCBSKY) || defined(PCB9XT)
  if(abs(tm)>(HSW_MAX))	 //momentary on-off
#endif
	{
  	lcd_putcAtt(x+3*FW,  y,'m',att);
		if ( tm > 0 )
		{
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
			tm -= HSW_MAX ;
#endif
#if defined(PCBSKY) || defined(PCB9XT)
			tm -= HSW_MAX ;
#endif
		}
	}
  putsDrSwitches( x-1*FW, y, tm, att ) ;
}

void putSwitchName(uint8_t x, uint8_t y, int8_t z, uint8_t att); // @menus.cpp

void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att)//, bool nc)
{
	if ( idx1 == 0 )
	{
    lcd_putsAtt(x+FW,y,XPSTR("---"),att);return;
	}
	else if ( idx1 == MAX_SKYDRSWITCH )
	{
    lcd_putsAtt(x+FW,y,PSTR(STR_ON),att);return;
	}
	else if ( idx1 == -MAX_SKYDRSWITCH )
	{
    lcd_putsAtt(x+FW,y,PSTR(STR_OFF),att);return;
	}
	else if ( idx1 == MAX_SKYDRSWITCH + 1 )
	{
    lcd_putsAtt(x+FW,y,XPSTR("Fmd"),att) ;
		return  ;;
	}

	if ( idx1 < 0 )
	{
  	lcd_putcAtt(x,y, '!',att);
	}
	int8_t z ;
	z = idx1 ;
	if ( z < 0 )
	{
		z = -idx1 ;
	}
	if ( ( z <= HSW_Ttrmup ) && ( z >= HSW_Etrmdn ) )
	{
		z -= HSW_Etrmdn ;
	  lcd_putsAttIdx(x+FW,y,XPSTR("\003EtdEtuAtdAtuRtdRtuTtuTtd"),z,att) ;
		return ;
	}
	if ( ( z <= HSW_FM6 ) && ( z >= HSW_FM0 ) )
	{
		z -= HSW_FM0 ;
  	lcd_putsAttIdx( x+FW, y, XPSTR("\003FM0FM1FM2FM3FM4FM5FM6"), z, att ) ;
		return ;
	}
	z -= 1 ;
#if defined(PCBSKY) || defined(PCB9XT)
//		z *= 3 ;
	if ( z > MAX_SKYDRSWITCH )
	{
		z -= HSW_OFFSET - 1 ;
	}
  putSwitchName(x+FW,y,z,att) ;
#endif

#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
//		z *= 3 ;
	if ( z > MAX_SKYDRSWITCH )
	{
		z -= HSW_OFFSET - 1 ;
	}
  putSwitchName(x+FW,y,z,att) ;
#endif
}

//Type 1-trigA, 2-trigB, 0 best for display
void putsTmrMode(uint8_t x, uint8_t y, uint8_t attr, uint8_t timer, uint8_t type )
{
  int8_t tm = g_model.timer[timer].tmrModeA ;
	if ( type < 2 )		// 0 or 1
	{
	  if(tm<TMR_VAROFS) {
        lcd_putsnAtt(  x, y, PSTR(STR_TRIGA_OPTS)+3*abs(tm),3,attr);
  	}
		else
		{
  		tm -= TMR_VAROFS - 7 ;
      lcd_putsAttIdx(  x, y, get_curve_string(), tm, attr ) ;
#if defined(PCBSKY) || defined(PCB9XT)
			if ( tm < 9 + 7 )	// Allow for 7 offset above
#endif
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
			if ( tm < 9 )
#endif
			{
				x -= FW ;
			}
  		lcd_putcAtt(x+3*FW,  y,'%',attr);
		}
	}
	if ( ( type == 2 ) || ( ( type == 0 ) && ( tm == 1 ) ) )
	{
		putsMomentDrSwitches( x, y, g_model.timer[timer].tmrModeB, attr ) ;
	}
}

const char *get_switches_string()
{
  return PSTR(SWITCHES_STR)+1	;
}

int16_t getValue(uint8_t i)
{
  if(i<7) return calibratedStick[i];//-512..512
	if ( i >= EXTRA_POTS_START-1 )
	{
		if ( i >= EXTRA_POTS_START-1+8 )
		{
			return get_telemetry_value( i-CHOUT_BASE-NUM_SKYCHNOUT ) ;
		}
  	if( i >= EXTRA_PPM_START )
		{
			if ( i < EXTRA_PPM_START + NUM_EXTRA_PPM )
			{
				return g_ppmIns[ i + NUM_PPM - EXTRA_PPM_START ] ;
			}
			else
			{
				return ex_chans[i+NUM_SKYCHNOUT-EXTRA_CHANNELS_START] ;
			}
		}
		return calibratedStick[i-EXTRA_POTS_START+8] ;
	}
  if(i<PPM_BASE) return 0 ;
	else if(i<CHOUT_BASE)
	{
		int16_t x ;
		x = g_ppmIns[i-PPM_BASE] ;
		if(i<PPM_BASE+4)
		{
			x -= g_eeGeneral.trainerProfile[g_model.trainerProfile].channel[i-PPM_BASE].calib ;
		}
		return x*2;
	}
	else if(i<CHOUT_BASE+NUM_SKYCHNOUT) return ex_chans[i-CHOUT_BASE];
  else if(i<CHOUT_BASE+NUM_SKYCHNOUT+NUM_TELEM_ITEMS)
	{
		return get_telemetry_value( i-CHOUT_BASE-NUM_SKYCHNOUT ) ;
	}
  return 0 ;
}


bool getSwitch00( int8_t swtch )
{
	return getSwitch( swtch, 0, 0 ) ;
}

bool getSwitch(int8_t swtch, bool nc, uint8_t level)
{
  bool ret_value ;
  uint8_t cs_index ;
  uint8_t aswitch ;

	aswitch = abs(swtch) ;
 	SwitchStack[level] = aswitch ;

	cs_index = aswitch-(MAX_SKYDRSWITCH-NUM_SKYCSW);

	{
		int32_t index ;
		for ( index = level - 1 ; index >= 0 ; index -= 1 )
		{
			if ( SwitchStack[index] == aswitch )
			{ // Recursion on this switch taking place
    		ret_value = Last_switch[cs_index] & 1 ;
		    return swtch>0 ? ret_value : !ret_value ;
			}
		}
	}
  if ( level > SW_STACK_SIZE - 1 )
  {
    ret_value = Last_switch[cs_index] & 1 ;
    return swtch>0 ? ret_value : !ret_value ;
  }

	if ( swtch == 0 )
	{
    return nc ;
	}
	else if ( swtch == MAX_SKYDRSWITCH )
	{
    return true ;
	}
	else if ( swtch == -MAX_SKYDRSWITCH )
	{
    return false ;
	}

#if defined(PCBSKY) || defined(PCB9XT)
	if ( abs(swtch) > MAX_SKYDRSWITCH )
	{
		uint8_t value = hwKeyState( abs(swtch) ) ;
		if ( swtch > 0 )
		{
			return value ;
		}
		else
		{
			return ! value ;
		}
	}
#endif
#if defined(PCBX9D) || defined(PCBX12D)  || defined(PCBX10)
	if ( abs(swtch) > MAX_SKYDRSWITCH )
	{
		uint8_t value = hwKeyState( abs(swtch) ) ;
		if ( swtch > 0 )
		{
			return value ;
		}
		else
		{
			return ! value ;
		}
	}
#endif

  uint8_t dir = swtch>0;
  if(abs(swtch)<(MAX_SKYDRSWITCH-NUM_SKYCSW))
	{
    if(!dir) return ! keyState((enum EnumKeys)(SW_BASE-swtch-1));
    return            keyState((enum EnumKeys)(SW_BASE+swtch-1));
  }

  //use putsChnRaw
  //input -> 1..4 -> sticks,  5..8 pots
  //MAX,FULL - disregard
  //ppm

	ret_value = Now_switch[cs_index] & 1 ;

	return swtch>0 ? ret_value : !ret_value ;

}

#ifndef WIDE_SCREEN
uint8_t speaker[] = {
4,8,0,
0x1C,0x1C,0x3E,0x7F
} ;
#endif

#if defined(PCBX12D) || defined(PCBX10)
void putsDblSizeName( uint16_t x, uint16_t y )
#else
void putsDblSizeName( uint8_t y )
#endif
{
	for(uint8_t i=0;i<sizeof(g_model.name);i++)
	{
#if defined(PCBX12D) || defined(PCBX10)
		lcd_putcAttColour( x + FW*2+i*2*FW-i-2, y, g_model.name[i],DBLSIZE, 0x001F, LcdBackground );
	}
#else
#ifdef WIDE_SCREEN
		lcd_putcAtt(FW*2+i*2*FW-i-2, y, g_model.name[i],DBLSIZE);
	}
#else
		lcd_putcAtt(FW*2-4+i*(2*FW-4), y, g_model.name[i],DBLSIZE|CONDENSED);
	}
	putsTime( 105, 0, Time.hour*60+Time.minute, 0, 0 ) ;
	lcd_img( 91, 8, speaker, 0, 0 ) ;
	lcd_hbar( 96, 9, 23, 6, (CurrentVolume*100+16)/23 ) ;
#endif
#endif
}


#if defined(PCBSKY) || defined(PCB9XT)
static uint16_t switches_states = 0 ;
static uint8_t trainer_state = 0 ;
#endif
#ifdef PCBX9D
#ifdef REV9E
uint32_t switches_states = 0 ;
uint8_t extSwitches_states = 0 ;
#else
uint32_t switches_states = 0 ;
#endif

#endif
#if defined(PCBX12D) || defined(PCBX10)
uint32_t switches_states = 0 ;
#endif

int8_t getMovedSwitch()
{
	uint8_t skipping = 0 ;
  int8_t result = 0;

	static uint16_t s_last_time = 0;

	uint16_t time = get_tmr10ms() ;
  if ( (uint16_t)(time - s_last_time) > 10)
	{
		skipping = 1 ;
	}
  s_last_time = time ;

#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
#ifdef REV9E
#else // REV9E
#if defined(PCBX7) || defined (PCBXLITE) || defined (PCBX9LITE)
#else // PCBX7 || PCBXLITE
  for (uint8_t i=0 ; i<8 ; i += 1 )
	{
    uint16_t mask = (0x03 << (i*2)) ;
    uint8_t prev = (switches_states & mask) >> (i*2) ;
		uint8_t next = switchPosition( i ) ;

    if (prev != next)
		{
      switches_states = (switches_states & (~mask)) | (next << (i*2));
      if (i<5)
        result = 1+(3*i)+next;
      else if (i==5)
			{
        result = -(1+(3*5)) ;
				if (next!=0) result = -result ;
			}
      else if (i==6)
        result = 1+(3*5)+1+next;
      else
			{
        result = -(1+(3*5)+1+3) ;
				if (next!=0) result = -result ;
			}
    }
  }

#endif // PCBX7
#endif // REV9E
#endif

  if ( skipping )
    result = 0 ;

  return result ;
}

void checkQuickSelect()
{
  uint8_t i = keyDown(); //check for keystate
  uint8_t j;

	if ( ( i & 6 ) == 6 )
	{
		SystemOptions |= SYS_OPT_MUTE ;
 		while ( keyDown() )
		{
			wdt_reset() ;
			lcd_puts_Pleft( FH, XPSTR("Mute Activated") ) ;
			refreshDisplay() ;
		}
		return ;
	}

	for(j=1; j<8; j++)
      if(i & (1<<j)) break;
  j--;

  if(j<6)
	{
#if defined(PCBSKY) || defined(PCB9XT)
    if(!eeModelExists(j))
#endif
#ifdef PCBX9D
    if(!eeModelExists(j))
#endif
			return ;
    if( g_eeGeneral.currModel != j )
		{
#if defined(PCBSKY) || defined(PCB9XT)
	    ee32LoadModel(g_eeGeneral.currModel = j);
			protocolsToModules() ;
#endif
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
#if defined(PCBX12D) || defined(PCBX10)
void eeLoadModel(uint8_t id) ;
#endif
	    eeLoadModel(g_eeGeneral.currModel = j);
			protocolsToModules() ;
#endif
#ifdef LUA
			luaLoadModelScripts() ;
#endif
#ifdef BASIC
			basicLoadModelScripts() ;
#endif
	    STORE_GENERALVARS;
		}
    lcd_clear();
    lcd_putsAtt(64-7*FW,0*FH,PSTR(STR_LOADING),DBLSIZE);

#if defined(PCBX12D) || defined(PCBX10)
		putsDblSizeName( 0, 3*FH ) ;
#else
		putsDblSizeName( 3*FH ) ;
#endif
    refreshDisplay();
    clearKeyEvents(); // wait for user to release key
  }
}

void alertMessages( const char * s, const char * t )
{
  lcd_clear();
  lcd_putsAtt(64-5*FW,0*FH,PSTR(STR_ALERT),DBLSIZE);
  lcd_puts_P(0,4*FH,s);
  lcd_puts_P(0,5*FH,t);
  lcd_puts_P(0,6*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;
}


void alert(const char * s, bool defaults)
{
	if ( Main_running )
	{
#define MESS_TYPE		1

		AlertType = ALERT_TYPE ;
		AlertMessage = s ;
		return ;
	}
	almess( s, ALERT_TYPE ) ;

	lcdSetRefVolt(defaults ? 0x22 : g_eeGeneral.contrast);
	voiceSystemNameNumberAudio( SV_ERROR, V_ERROR, AU_WARNING2 ) ;
  clearKeyEvents();
  while(1)
  {
#ifdef SIMU
    if (!main_thread_running) return;
    sleep(1/*ms*/);
#endif

    if(keyDown())
    {
	    clearKeyEvents();
      return;  //wait for key release
    }
    wdt_reset();

		if ( check_power_or_usb() )
		{
			 return ;		// Usb on or power off
    }
		if(getSwitch00(g_eeGeneral.lightSw) || getSwitch00(g_model.mlightSw) || g_eeGeneral.lightAutoOff || defaults)
      {BACKLIGHT_ON;}
    else
      {BACKLIGHT_OFF;}
  }
}

void message(const char * s)
{
	almess( s, MESS_TYPE ) ;
}


#ifdef PCBX9D
void check6pos()
{
	if ( g_eeGeneral.analogMapping & MASK_6POS )
	{
		if ( g_eeGeneral.SixPositionCalibration[0] == 0 )
		{
			if ( g_eeGeneral.SixPositionCalibration[1] == 0 )
			{
				// 6POS enabled but not calibrated
    		alert(XPSTR("6 Pos switch enabled\037 But NOT calibrated"));
			}
		}
	}
}
#endif

uint8_t checkThrottlePosition()
{
  uint8_t thrchn=(2-(g_eeGeneral.stickMode&1));//stickMode=0123 -> thr=2121
	int16_t v = scaleAnalog( anaIn(thrchn), thrchn ) ;
	if ( g_model.throttleIdle )
	{
		if ( abs( v ) < THRCHK_DEADBAND )
		{
			return 1 ;
		}
	}
	else
	{
  	if(v <= -RESX + THRCHK_DEADBAND )
  	{
  		return 1 ;
  	}
	}
	return 0 ;
}

void checkMultiPower()
{
	uint32_t warning = 0 ;
	if ( g_model.Module[0].protocol == PROTO_MULTI )
	{
		if ( (g_model.Module[0].channels>>7) & 0x01 )
		{
			warning = 1 ;
		}
	}
	if ( g_model.Module[1].protocol == PROTO_MULTI )
	{
		if ( (g_model.Module[1].channels>>7) & 0x01 )
		{
			warning = 1 ;
		}
	}
	if ( warning )
	{
  	alert(XPSTR("Multi on LOW power"));
		voiceSystemNameNumberAudio( SV_WARNING, V_ERROR, AU_WARNING1 ) ;
	}
}


void checkTHR()
{
  if(g_eeGeneral.disableThrottleWarning) return;
  if(g_model.disableThrottleCheck) return;

#ifndef SIMU
  getADC_single();   // if thr is down - do not display warning at all
#endif

	if ( checkThrottlePosition() )
	{
		return ;
	}

  // first - display warning

	lcd_clear();
#if defined(PCBX12D) || defined(PCBX10)
  lcd_img( 1 + X12OFFSET, 0, HandImage,0,0, LCD_RED ) ;
#else
  lcd_img( 1, 0, HandImage,0,0 ) ;
#endif
  lcd_putsAtt(36 + X12OFFSET,0*FH,XPSTR("THROTTLE"),DBLSIZE|CONDENSED);
  lcd_putsAtt(36 + X12OFFSET,2*FH,PSTR(STR_WARNING),DBLSIZE|CONDENSED);
	lcd_puts_P(0 + X12OFFSET,5*FH,  PSTR(STR_THR_NOT_IDLE) ) ;
	lcd_puts_P(0 + X12OFFSET,6*FH,  PSTR(STR_RST_THROTTLE) ) ;
	lcd_puts_P(0 + X12OFFSET,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;
  refreshDisplay();
  clearKeyEvents();
	putSystemVoice( SV_TH_WARN, V_THR_WARN ) ;

	//loop until throttle stick is low
  while (1)
  {
#if defined(PCBX12D) || defined(PCBX10)
		PictureDrawn = 0 ;
#endif

#ifdef SIMU
      if (!main_thread_running) return;
      sleep(1/*ms*/);
#else
      getADC_single();
#endif
			check_backlight() ;

			if ( checkThrottlePosition() )
			{
				return ;
			}
      if( keyDown() )
      {
			  clearKeyEvents() ;
        return;
      }
      wdt_reset();
			CoTickDelay(1) ;					// 2mS for now

		if ( check_power_or_usb() ) return ;		// Usb on or power off

  }
}

int32_t readControl( uint8_t channel )
{
	int32_t value ;
	value = scaleAnalog( anaIn(channel), channel ) ;
	value *= 100 ;
	value /= 1024 ;
	return value ;
}

void checkCustom()
{
	CustomCheckData *pdata ;
	pdata = &g_model.customCheck ;
	int32_t value ;
	uint32_t timer ;
	uint8_t idx = pdata->source - 1 ;
	if ( pdata->source == 0 )
	{
		return ;
	}
	if ( idx < 4 )
	{
		idx = stickScramble[g_eeGeneral.stickMode*4+idx] ;
	}



#ifndef SIMU
  getADC_osmp() ;
#endif

	value = readControl( idx ) ;
	if ( ( value >= pdata->min ) && ( value <= pdata->max ) )
	{
		return ;
	}

  clearKeyEvents();

  timer = 0 ;

	putSystemVoice( SV_CUSTOM_WARN, V_CUSTOM_WARN ) ;

  while (1)
  {
#ifdef SIMU
    if (!main_thread_running) return;
    sleep(1/*ms*/);
#else
    getADC_single();
#endif
		check_backlight() ;

		value = readControl( idx ) ;
		if ( ( value >= pdata->min ) && ( value <= pdata->max ) )
		{
			if ( ++timer > 99 )
			{
				return ;
			}
		}
		else
		{
			timer = 0 ;
		}
	  alertMessages( XPSTR("Custom Check"), XPSTR("Set Control") ) ;
		putsChnRaw( 9*FW, 2*FH, unmapPots( pdata->source ), 0 ) ;
		lcd_outdezAtt( 5*FW, 3*FH, pdata->min, 0) ;
		lcd_outdezAtt( 11*FW, 3*FH, value, 0) ;
		lcd_outdezAtt( 17*FW, 3*FH, pdata->max, 0) ;
		refreshDisplay();

    if( keyDown() )
    {
			clearKeyEvents() ;
      return;
    }
    wdt_reset();
		CoTickDelay(5) ;					// 10mS for now

		if ( check_power_or_usb() ) return ;		// Usb on or power off
  }
}


uint16_t oneSwitchText( uint8_t swtch, uint16_t states )
{
	uint8_t index = swtch - 1 ;
	uint8_t attr = 0 ;
	return (attr << 8 ) | index ;
}


void putWarnSwitch( uint8_t x, uint8_t idx )
{
	if ( ( idx == 2 ) || ( idx == 1 ) )
	{
		idx = oneSwitchText( idx+1, getCurrentSwitchStates() ) ;
	}
  putSwitchName( x, 5*FH, idx, 0) ;
}

uint16_t getCurrentSwitchStates()
{
  uint16_t i = 0 ;
	getMovedSwitch() ;
	i = switches_states & 1 ;
	if ( switches_states & 2 )			i |= 0x0100 ;
	if ( switches_states & 4 )			i |= 0x0002 ;
	if ( switches_states & 8 )			i |= 0x0200 ;
	if ( switches_states & 0x10 ) 	i |= 0x0004 ;
	if ( switches_states & 0x20 ) 	i |= 0x0400 ;
	if ( switches_states & 0x40 ) 	i |= 0x0008 ;
	if ( switches_states & 0x80 ) 	i |= 0x0800 ;
	if ( switches_states & 0x100 )  i |= 0x0010 ;
	if ( switches_states & 0x400 )  i |= 0x0020 ;
	if ( switches_states & 0x1000 ) i |= 0x0040 ;
	if ( switches_states & 0x2000 ) i |= 0x1000 ;
	if ( switches_states & 0x4000 ) i |= 0x0080 ;
	if ( switches_states & 0x8000 ) i |= 0x2000 ;

	return i ;
}

void checkSwitches()
{
#if defined(PCBX12D) || defined(PCBX10)
	uint32_t warningStates ;
#else
	uint16_t warningStates ;
#endif

#ifdef PCB9XT
		read_adc() ; // needed for 3/6 pos ELE switch
		processAnalogSwitches() ;
		processAnalogSwitches() ;		// Make sure the values are processed at startup.
#endif // PCB9XT

	warningStates = g_model.modelswitchWarningStates ;
#if defined(PCBX12D) || defined(PCBX10)
	warningStates |= (uint32_t)(g_model.xmodelswitchWarningStates & 0x07) << 16 ;
#endif

	if( warningStates & 1 ) return ; // if warning is on
	warningStates >>= 1 ;

#if defined(PCBSKY) || defined(PCB9XT)
	uint8_t x = warningStates & SWP_IL5;
  if(x==SWP_IL1 || x==SWP_IL2 || x==SWP_IL3 || x==SWP_IL4 || x==SWP_IL5) //illegal states for ID0/1/2
  {
    warningStates &= ~SWP_IL5; // turn all off, make sure only one is on
    warningStates |=  SWP_ID0B;
		g_model.modelswitchWarningStates = (warningStates << 1) ;
  }
#endif

#if defined(PCBSKY) || defined(PCB9XT)
	uint8_t first = 1 ;
#endif
	//loop until all switches are reset

 	warningStates &= ~g_model.modelswitchWarningDisables ;
	while (1)
  {
#if defined(PCBSKY) || defined(PCB9XT)
		read_adc() ; // needed for 3/6 pos ELE switch
#ifdef PCB9XT
		processAnalogSwitches() ;
#endif // PCB9XT

#if defined(PCBX12D) || defined(PCBX10)
    uint32_t i = getCurrentSwitchStates() ;
		i &= ~g_model.modelswitchWarningDisables ;
		if ( g_model.modelswitchWarningDisables & 0xC000 ) // 6-pos
		{
			i &= ~0x00010000 ;
		}
#else
    uint16_t i = getCurrentSwitchStates() ;
		i &= ~g_model.modelswitchWarningDisables ;
#endif

		if ( first )
		{
 			clearKeyEvents();
			first = 0 ;
			if( i != warningStates )
			{
				putSystemVoice( SV_SW_WARN, V_SW_WARN ) ;
			}
		}

    if( (i==warningStates) || (keyDown())) // check state against settings
    {
        return;  //wait for key release
    }

        //show the difference between i and switch?
        //show just the offending switches.
        //first row - THR, GEA, AIL, ELE, ID0/1/2
        uint16_t x = i ^ warningStates ;

  			lcd_clear();
		    lcd_img( 1, 0, HandImage,0,0 ) ;
			  lcd_putsAtt(36,0*FH,PSTR(STR_SWITCH),DBLSIZE|CONDENSED);
  			lcd_putsAtt(36,2*FH,PSTR(STR_WARNING),DBLSIZE|CONDENSED);
  			lcd_puts_P(0,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;

#ifdef ARUNI
        if(x & THR_WARN_MASK)
            putWarnSwitch(2 + 0*FW, 0 );
        if(x & RUD_WARN_MASK)
            putWarnSwitch(2 + 3*FW + FW/2, 1 );
        if(x & ELE_WARN_MASK)
            putWarnSwitch(2 + 7*FW, 2 );

        if(x & IDX_WARN_MASK)
        {
            if(i & SWP_ID0B)
                putWarnSwitch(2 + 10*FW + FW/2, 3 );
            if(i & SWP_ID1B)
                putWarnSwitch(2 + 10*FW + FW/2, 4 );
            if(i & SWP_ID2B)
                putWarnSwitch(2 + 10*FW + FW/2, 5 );
        }

        if(x & AIL_WARN_MASK)
            putWarnSwitch(2 + 14*FW, 6 );
        if(x & GEA_WARN_MASK)
            putWarnSwitch(2 + 17*FW + FW/2, 7 );
#else
        if(x & SWP_THRB)
            putWarnSwitch(2 + 0*FW, 0 );
        if(x & 0x0202)
            putWarnSwitch(2 + 3*FW + FW/2, 1 );
        if(x & 0x0C04)
            putWarnSwitch(2 + 7*FW, 2 );

        if(x & SWP_IL5)
        {
            if(i & SWP_ID0B)
                putWarnSwitch(2 + 10*FW + FW/2, 3 );
            if(i & SWP_ID1B)
                putWarnSwitch(2 + 10*FW + FW/2, 4 );
            if(i & SWP_ID2B)
                putWarnSwitch(2 + 10*FW + FW/2, 5 );
        }

        if(x & 0x1040)
            putWarnSwitch(2 + 14*FW, 6 );
        if(x & 0x2080)
            putWarnSwitch(2 + 17*FW + FW/2, 7 );
#endif


#endif
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
// To Do
#ifndef SIMU
		  getADC_single() ;	// For 6-pos switch
#endif
			getMovedSwitch() ;	// loads switches_states

#if defined(PCBX12D) || defined(PCBX10)
    	uint32_t ss = switches_states ;
#else
    	uint16_t ss = switches_states ;
#endif
#ifdef PCBT12
			if ( ss & 0x8000 )
			{
				ss &= ~0xF000 ;
				ss |= 0x2000 ;
			}
#endif
			ss &= ~g_model.modelswitchWarningDisables ;
			if ( g_model.modelswitchWarningDisables & 0xC000 ) // 6-pos
			{
				ss &= ~0x00010000 ;
			}

#ifdef PCBX7
 #ifdef PCBT12
			if ( ( ss & 0x3CFF ) == warningStates )	// Miss E and G, inc H
 #else
			if ( ( ss & 0x0CFF ) == warningStates )	// Miss E and G
 #endif
#else
 #ifdef PCBXLITE
			if ( ( ss & 0x00FF ) == (warningStates & 0x00FF ) )
 #else
  #ifdef PCBX9LITE
			if ( ( ss & 0x0C3F ) == (warningStates & 0x0C3F) )	// Miss D, E and G
  #else // X3
#if defined(PCBX12D) || defined(PCBX10)
			if ( ( ss & 0x0001FFFF ) == warningStates )
   #else
			if ( ( ss & 0x3FFF ) == warningStates )
   #endif
  #endif // X3
 #endif
#endif
			{
				return ;
			}
#if defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
// To Do
		if ( keyDown() )
		{
	    clearKeyEvents();
			return ;
		}
#endif
		lcd_clear();
#if defined(PCBX12D) || defined(PCBX10)
    lcd_img( 1 + X12OFFSET, 0, HandImage,0,0, LCD_RED ) ;
#else
    lcd_img( 1, 0, HandImage,0,0 ) ;
#endif
	  lcd_putsAtt(32 + X12OFFSET,0*FH,XPSTR("SWITCH"),DBLSIZE);
	  lcd_putsAtt(32 + X12OFFSET,2*FH,XPSTR("WARNING"),DBLSIZE);
		lcd_puts_P(0 + X12OFFSET,7*FH,  PSTR(STR_PRESS_KEY_SKIP) ) ;

		for ( uint8_t i = 0 ; i < 7 ; i += 1 )
		{
#ifdef PCBX7
#ifdef PCBT12
			if ( i == 4 )
#else
			if ( ( i == 4 ) || ( i == 6 ) )
#endif
			{
				continue ;	// Skip E and G
			}
#endif
#ifdef PCBX9LITE
			if ( ( i == 3 ) || ( i == 4 ) || ( i == 6 ) )
			{
				continue ;	// Skip D, E and G
			}
#endif
#ifdef PCBXLITE
			if ( i >= 4 )
			{
				continue ;	// Skip E and G
			}
#endif
 		  uint16_t mask = ( 0x03 << (i*2) ) ;
 		  uint8_t attr = ((warningStates & mask) == (ss & mask)) ? 0 : INVERS ;
			if ( ~g_model.modelswitchWarningDisables & mask )
			{
#ifdef PCBT12
				uint32_t j ;
				j = i ;
				if ( j > 4 )
				{
					j += 1 ;
				}
  		  lcd_putcAtt( 3*FW+i*(2*FW+2) + X12OFFSET, 5*FH, 'A'+j, attr ) ;
#else
  		  lcd_putcAtt( 3*FW+i*(2*FW+2) + X12OFFSET, 5*FH, 'A'+i, attr ) ;
#endif
				lcd_putcAtt( 4*FW+i*(2*FW+2) + X12OFFSET, 5*FH, PSTR(HW_SWITCHARROW_STR)[(warningStates & mask) >> (i*2)], attr ) ;
			}
		}
#if defined(PCBX12D) || defined(PCBX10)
		if ( ~g_model.modelswitchWarningDisables & 0xC000 )
		{
		  uint8_t attr = ((warningStates & 0x1C000) == (ss & 0x1C000)) ? 0 : INVERS ;
  		lcd_putcAtt( 3*FW+7*(2*FW+2) + X12OFFSET, 5*FH, '6', attr ) ;
			lcd_putcAtt( 4*FW+7*(2*FW+2) + X12OFFSET, 5*FH, ((warningStates & 0x0001C000) >> 14) + '0', attr ) ;
		}

//		lcd_outhex4( 190, 4*FH, ss >> 16 ) ;
//		lcd_outhex4( 216, 4*FH, ss ) ;
//		lcd_outhex4( 190, 5*FH, warningStates >> 16 ) ;
//		lcd_outhex4( 216, 5*FH, warningStates ) ;
//		lcd_outhex4( 190, 6*FH, switches_states >> 16 ) ;
//		lcd_outhex4( 216, 6*FH, switches_states ) ;

//		lcd_outhex4( 190, 7*FH, g_model.modelswitchWarningDisables ) ;
//		lcd_outhex4( 216, 7*FH, switchPosition( HSW_Ele6pos0 ) ) ;

//		lcd_outhex4( 0, 0*FH, hwKeyState( HSW_Ele6pos0 ) ) ;
//		lcd_outhex4( 0, 1*FH, hwKeyState( HSW_Ele6pos1 ) ) ;
//		lcd_outhex4( 0, 2*FH, hwKeyState( HSW_Ele6pos2 ) ) ;
//		lcd_outhex4( 0, 3*FH, hwKeyState( HSW_Ele6pos3 ) ) ;
//		lcd_outhex4( 0, 4*FH, hwKeyState( HSW_Ele6pos4 ) ) ;
//		lcd_outhex4( 0, 5*FH, hwKeyState( HSW_Ele6pos5 ) ) ;




#endif

#endif
    refreshDisplay() ;


    wdt_reset();

		if ( check_power_or_usb() ) return ;		// Usb on or power off

		check_backlight() ;

#ifdef SIMU
    if (!main_thread_running) return;
    sleep(1/*ms*/);
#endif
		CoTickDelay(5) ;	// 10mS
  }
}

#ifdef ARUNI
void displayChangeWarning(const char *s)
{
  lcd_img( 1, 0, HandImage,0,0 ) ;
  lcd_putsAtt(36,1*FH,PSTR(STR_WARNING),DBLSIZE|CONDENSED);
	lcd_puts_P(0,4*FH,  s ) ;
	lcd_puts_P((strlen(s)+1)*FW, 4*FH, PSTR(STR_CHANGE_MAY) ) ;
	lcd_puts_P(0,5*FH,  PSTR(STR_BRICK_RADIO) ) ;
	lcd_puts_P(0,7*FH,  PSTR(STR_MENU_PROCEED) ) ;
}
#endif

#ifdef SLAVE_RESET
void panicDebugMenu()
{
	popMenu(true) ; //return to uppermost, beeps itself
	pushMenu(menuProcPanic) ;
}
#endif


MenuFuncP lastPopMenu()
{
  return  g_menuStack[g_menuStackPtr+1];
}

extern union t_sharedMemory SharedMemory ;

void leavingMenu()
{
	if ( SharedMemory.TextControl.TextFileOpen )
	{
		f_close( &SharedMemory.TextControl.TextFile ) ;
		SharedMemory.TextControl.TextFileOpen = 0 ;
	}
}

void popMenu(bool uppermost)
{
	leavingMenu() ;
  if(g_menuStackPtr>0 || uppermost)
	{
    g_menuStackPtr = uppermost ? 0 : g_menuStackPtr-1;
 		EnterMenu = EVT_ENTRY_UP ;
  }else{
    alert(PSTR(STR_MSTACK_UFLOW));
  }
}

void chainMenu(MenuFuncP newMenu)
{
	leavingMenu() ;
  g_menuStack[g_menuStackPtr] = newMenu;
	EnterMenu = EVT_ENTRY ;
}

void pushMenu(MenuFuncP newMenu)
{
	leavingMenu() ;
  if(g_menuStackPtr >= DIM(g_menuStack)-1)
  {
    alert(PSTR(STR_MSTACK_OFLOW));
    return;
  }
	EnterMenu = EVT_ENTRY ;
  g_menuStack[++g_menuStackPtr] = newMenu ;
}

uint8_t *ncpystr( uint8_t *dest, uint8_t *source, uint8_t count )
{
  while ( (*dest++ = *source++) )
	{
		if ( --count == 0 )
		{
			*dest++ = '\0' ;
			break ;
		}
	}
  return dest - 1 ;
}

uint8_t *cpystr( uint8_t *dest, uint8_t *source )
{
  while ( (*dest++ = *source++) )
    ;
  return dest - 1 ;
}

int8_t REG100_100(int8_t x)
{
	return REG( x, -100, 100 ) ;
}

int8_t REG(int8_t x, int8_t min, int8_t max)
{
  int8_t result = x;
  if (x >= 126 || x <= -126) {
    x = (uint8_t)x - 126;
    result = g_model.gvars[x].gvar ;
    if (result < min) {
      g_model.gvars[x].gvar = result = min;
    }
    if (result > max) {
      g_model.gvars[x].gvar = result = max;
    }
  }
  return result;
}

uint8_t IS_EXPO_THROTTLE( uint8_t x )
{
	if ( g_model.thrExpo )
	{
		return IS_THROTTLE( x ) ;
	}
	return 0 ;
}

void checkXyCurve()
{
	if ( g_model.curvexy[9] == 0 )
	{
		uint32_t i ;
		int8_t j = -100 ;
		for ( i = 9 ; i < 18 ; j += 25, i += 1 )
		{
			g_model.curvexy[i] = j ;
		}
	}
	if ( g_model.curve2xy[9] == 0 )
	{
		uint32_t i ;
		int8_t j = -100 ;
		for ( i = 9 ; i < 18 ; j += 25, i += 1 )
		{
			g_model.curve2xy[i] = j ;
		}
	}
}

void protocolsToModules()
{
	if ( g_model.modelVersion < 4 )
	{

		int8_t temp ;
#ifdef PCBSKY
		g_model.Module[1].protocol = g_model.protocol ;
		g_model.Module[1].country = g_model.country ;
		g_model.Module[1].ppmOpenDrain = g_model.ppmOpenDrain ;
		g_model.Module[1].pulsePol = g_model.pulsePol ;
		if ( g_model.Module[1].protocol == PROTO_PPM )
		{
			temp = (g_model.ppmNCH) * 2 ;
			if ( temp > 8 )
			{
				temp -= 13 ;
			}
		}
		else
		{
			temp = g_model.ppmNCH ;
		}
		g_model.Module[1].channels = temp ;
		g_model.Module[1].startChannel = g_model.startChannel ;
		g_model.Module[1].sub_protocol = g_model.sub_protocol ;
		g_model.Module[1].pxxRxNum = g_model.pxxRxNum ;
		g_model.Module[1].ppmDelay = g_model.ppmDelay ;
		g_model.Module[1].ppmFrameLength = g_model.ppmFrameLength ;
		g_model.Module[1].option_protocol = g_model.option_protocol ;
		g_model.Module[1].failsafeMode = g_model.failsafeMode[0] ;
		for ( temp = 0 ; temp < 16 ; temp += 1 )
		{
			uint32_t a = temp ;
			uint32_t b = temp / 8 ;
			if ( b )
			{
				a -= 8 ;
			}
			g_model.Module[1].failsafe[temp] = g_model.accessFailsafe[b][a] ;
		}

		g_model.Module[0].protocol = g_model.xprotocol ;
		g_model.Module[0].country = g_model.xcountry ;
		g_model.Module[0].pulsePol = g_model.xpulsePol ;
		if ( g_model.Module[0].protocol == PROTO_PPM )
		{
			temp = (g_model.ppm2NCH) * 2 ;
			if ( temp > 8 )
			{
				temp -= 13 ;
			}
		}
		else
		{
			temp = g_model.xppmNCH ;
		}
		g_model.Module[0].channels = temp ;
		g_model.Module[0].startChannel = g_model.xstartChannel ;
		temp = g_model.startPPM2channel ;
		if ( temp )
		{
			temp -= 1 ;
		}
		g_model.Module[0].startChannel = temp ;
		g_model.Module[0].sub_protocol = g_model.xsub_protocol ;
		g_model.Module[0].pxxRxNum = g_model.xPxxRxNum ;
		g_model.Module[0].ppmDelay = g_model.xppmDelay ;
		g_model.Module[0].ppmFrameLength = g_model.xppmFrameLength ;
		g_model.Module[0].option_protocol = g_model.xoption_protocol ;
		g_model.Module[0].failsafeMode = g_model.failsafeMode[1] ;
#endif
#if defined(PCB9XT) || defined(PCBX9D) || defined(PCBX12D) || defined(PCBX10)
		g_model.Module[0].protocol = g_model.protocol ;
		g_model.Module[0].country = g_model.country ;
		g_model.Module[0].pulsePol = g_model.pulsePol ;
		temp = g_model.ppmNCH ;
		if ( g_model.Module[0].protocol == PROTO_PPM )
		{
			temp *= 2 ;
			if ( temp > 8 )
			{
				temp -= 13 ;
			}
		}
		g_model.Module[0].channels = temp ;
		g_model.Module[0].startChannel = g_model.startChannel ;
		g_model.Module[0].sub_protocol = g_model.sub_protocol ;
		g_model.Module[0].pxxRxNum = g_model.pxxRxNum ;
		g_model.Module[0].ppmDelay = g_model.ppmDelay ;
		g_model.Module[0].ppmFrameLength = g_model.ppmFrameLength ;
		g_model.Module[0].option_protocol = g_model.option_protocol ;
		g_model.Module[0].failsafeMode = g_model.failsafeMode[0] ;
		for ( temp = 0 ; temp < 16 ; temp += 1 )
		{
			uint32_t a = temp ;
			uint32_t b = temp / 8 ;
			if ( b )
			{
				a -= 8 ;
			}
			g_model.Module[0].failsafe[temp] = g_model.accessFailsafe[b][a] ;
		}

		g_model.Module[1].protocol = g_model.xprotocol ;
		g_model.Module[1].country = g_model.xcountry ;
		g_model.Module[1].pulsePol = g_model.xpulsePol ;
		temp = g_model.xppmNCH ;
		if ( g_model.Module[1].protocol == PROTO_PPM )
		{
			temp *= 2 ;
			if ( temp > 8 )
			{
				temp -= 13 ;
			}
		}
		g_model.Module[1].channels = temp ;
		g_model.Module[1].startChannel = g_model.xstartChannel ;
		g_model.Module[1].sub_protocol = g_model.xsub_protocol ;
		g_model.Module[1].pxxRxNum = g_model.xPxxRxNum ;
		g_model.Module[1].ppmDelay = g_model.xppmDelay ;
		g_model.Module[1].ppmFrameLength = g_model.xppmFrameLength ;
		g_model.Module[1].option_protocol = g_model.xoption_protocol ;
		g_model.Module[1].failsafeMode = g_model.failsafeMode[1] ;
		for ( temp = 0 ; temp < 16 ; temp += 1 )
		{
			uint32_t a = temp ;
			uint32_t b = temp / 8 ;
			if ( b )
			{
				a -= 8 ;
			}
			g_model.Module[1].failsafe[temp] = g_model.accessFailsafe[b][a] ;
		}
#endif
		g_model.modelVersion = 4 ;
		STORE_MODELVARS ;
	}
}

/*** EOF ***/
