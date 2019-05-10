/****************************************************************************
*  Copyright (c) 2013 by Michael Blandford. All rights reserved.
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
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
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
* Other Authors:
 * - Andre Bernet
 * - Bertrand Songis
 * - Bryan J. Rentoul (Gruvin)
 * - Cameron Weeks
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini
 * - Thomas Husterer
*
****************************************************************************/


#include <stdint.h>
#include <stdlib.h>

//#include "AT91SAM3S4.h"
//#ifndef SIMU
//#include "core_cm3.h"
//#endif

#include "ersky9x.h"
#include "myeeprom.h"
#include "logicio.h"
#include "drivers.h"
#include "pulses.h"
//#include "debug.h"

extern uint8_t BindRangeFlag[] ;
//extern uint8_t PxxExtra[] ;

void dsmBindResponse( uint8_t mode, int8_t channels )
{
	// Process mode here
	uint8_t dsm_mode_response ;
	{
		dsm_mode_response = mode & ( ORTX_USE_DSMX | ORTX_USE_11mS | ORTX_USE_11bit | ORTX_AUTO_MODE ) ;
		if ( g_model.Module[1].protocol != PROTO_MULTI )
		{
			if ( ( g_model.Module[1].channels != channels ) || ( g_model.dsmMode != ( dsm_mode_response | 0x80 ) ) )
			{
				g_model.Module[1].channels = channels ;
				g_model.dsmMode = dsm_mode_response | 0x80 ;
	  		STORE_MODELVARS ;
			}
		}
		else
		{
extern uint8_t MultiResponseData ;
		dsm_mode_response = channels ;
		if ( mode & 0x80 )
		{
			dsm_mode_response |= 0x80 ;
		}
		if ( mode & 0x10 )
		{
			dsm_mode_response |= 0x40 ;
		}
		MultiResponseData = dsm_mode_response ;
extern uint8_t MultiResponseFlag ;
			MultiResponseFlag = 1 ;
		}
	}
}


void setMultiSerialArray( uint8_t *data, uint32_t module )
{
	uint32_t i ;
	uint8_t packetType ;
	uint8_t protoByte ;
	uint8_t optionByte ;
	uint8_t subProtocol ;
	uint32_t outputbitsavailable = 0 ;
	uint32_t outputbits = 0 ;
	struct t_module *pmodule = &g_model.Module[module] ;
	uint8_t startChan = pmodule->startChannel ;
	subProtocol = pmodule->sub_protocol+1 ;
	packetType = ( ( subProtocol & 0x3F) > 31 ) ? 0x54 : 0x55 ;
  if (pmodule->failsafeMode != FAILSAFE_NOT_SET && pmodule->failsafeMode != FAILSAFE_RX )
	{
    if ( FailsafeCounter[module] )
		{
	    if ( FailsafeCounter[module]-- == 1 )
			{
				packetType += 2 ;	// Failsafe packet
			}
		}
	  if ( FailsafeCounter[module] == 0 )
		{
//			if ( pmodule->failsafeRepeat == 0 )
//			{
				FailsafeCounter[module] = 1000 ;
//			}
		}
	}
	*data++ = packetType ;
	protoByte = subProtocol & 0x5F;		// load sub_protocol and clear Bind & Range flags
	if (BindRangeFlag[module] & PXX_BIND)	protoByte |=BindBit ;		//set bind bit if bind menu is pressed
	if (BindRangeFlag[module] & PXX_RANGE_CHECK) protoByte |=RangeCheckBit ;		//set bind bit if bind menu is pressed
	*data++ = protoByte ;

	protoByte = pmodule->channels ;
	*data++ = ( protoByte/*g_model.ppmNCH*/ & 0xF0) | ( pmodule->pxxRxNum & 0x0F ) ;

	optionByte = pmodule->option_protocol ;
	if ( ( subProtocol & 0x3F ) == M_AFHD2SA + 1 )
	{
    optionByte |= 0x80 ;
	}
	*data++ = optionByte ;
	for ( i = 0 ; i < 16 ; i += 1 )
	{
		int16_t x ;
		uint32_t y = startChan + i ;
		x = y >= ( NUM_SKYCHNOUT+EXTRA_SKYCHANNELS ) ? 0 : g_chans512[y] ;
		if ( packetType & 2 )
		{
			if ( pmodule->failsafeMode == FAILSAFE_HOLD )
			{
				x = 2047 ;
			}
			else if ( pmodule->failsafeMode == FAILSAFE_NO_PULSES )
			{
				x = 0 ;
			}
			else
			{
				// Send failsafe value
				int32_t value ;
				value = ( startChan < 16 ) ? pmodule->failsafe[startChan] : 0 ;
				value = ( value *4193 ) >> 9 ;
				value += 1024 ;
				x = limit( (int16_t)1, (int16_t)value, (int16_t)2046 ) ;
				startChan += 1 ;
			}
		}
		else
		{
			x *= 4 ;
			x += x > 0 ? 4 : -4 ;
			x /= 5 ;
			x += 0x400 ;
		}
		if ( x < 0 )
		{
			x = 0 ;
		}
		if ( x > 2047 )
		{
			x = 2047 ;
		}
		outputbits |= (uint32_t)x << outputbitsavailable ;
		outputbitsavailable += 11 ;
		while ( outputbitsavailable >= 8 )
		{
			uint32_t j = outputbits ;
			*data++ = j ;
			outputbits >>= 8 ;
			outputbitsavailable -= 8 ;
		}
	}
}


void setDsmHeader( uint8_t *dsmDat, uint32_t module )
{
  if (dsmDat[0]&BadData)  //first time through, setup header
  {
  	switch(g_model.Module[module].sub_protocol)
  	{
  		case LPXDSM2:
  		  dsmDat[0]= 0x80;
  		break;
  		case DSM2only:
  		  dsmDat[0]=0x90;
  		break;
  		default:
  		  dsmDat[0]=0x98;  //dsmx, bind mode
  		break;
  	}
  }

#if defined(PCBSKY) || defined(PCB9XT)
	if((dsmDat[0]&BindBit)&&(!keyState(SW_Trainer)))  dsmDat[0]&=~BindBit;		//clear bind bit if trainer not pulled
#else
	if((dsmDat[0]&BindBit)&&(!keyState(SW_SH2)))  dsmDat[0]&=~BindBit;		//clear bind bit if trainer not pulled
#endif
  if ((!(dsmDat[0]&BindBit))&& (BindRangeFlag[module] & PXX_RANGE_CHECK)) dsmDat[0]|=RangeCheckBit;   //range check function
  else dsmDat[0]&=~RangeCheckBit;
}


const uint16_t CRC_Short[]=
{
   0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
   0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7 };

uint16_t CRCTable(uint8_t val)
{
	return CRC_Short[val&0x0F] ^ (0x1081 * (val>>4));
}



uint16_t scaleForPXX( uint8_t i )
{
	int16_t value ;

	value = ( i < 32 ) ? g_chans512[i] *3 / 4 + 1024 : 0 ;
	return limit( (int16_t)1, value, (int16_t)2046 ) ;
}


#ifdef XFIRE

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
unsigned char crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8(const uint8_t * ptr, uint32_t len)
{
  uint8_t crc = 0;
  for ( uint32_t i=0 ; i<len ; i += 1 )
	{
    crc = crc8tab[crc ^ *ptr++] ;
  }
  return crc;
}

#define CROSSFIRE_CH_CENTER         0x3E0
#define CROSSFIRE_CH_BITS           11
#define CROSSFIRE_CHANNELS_COUNT		16

#define MODULE_ADDRESS							0xEE
#define CHANNELS_ID									0x16

extern uint8_t Bit_pulses[] ;
extern uint16_t XfireLength ;
extern struct t_telemetryTx TelemetryTx ;

// Range for pulses (channels output) is [-1024:+1024]
uint8_t setupPulsesXfire()
{
	uint32_t startChan ;
 	uint8_t *buf = Bit_pulses ;
 	*buf++ = MODULE_ADDRESS ;

	if ( TelemetryTx.XfireTx.count )
	{
		uint32_t i ;
 		*buf++ = TelemetryTx.XfireTx.count + 2 ;
  	uint8_t *crc_start = buf ;
  	*buf++ = TelemetryTx.XfireTx.command ;
  	for ( i = 0 ; i < TelemetryTx.XfireTx.count ; i += 1 )
		{
			*buf++ = TelemetryTx.XfireTx.data[i] ;
			if ( i > 62 )
			{
				break ;
			}
		}
		i = buf - crc_start ;
		*buf++ = crc8( crc_start, i ) ;
		TelemetryTx.XfireTx.count = 0 ;
	}
	else
	{
		startChan = g_model.Module[1].startChannel ;
  	*buf++ = 24 ; // 1(ID) + 22 + 1(CRC)
  	uint8_t *crc_start = buf ;
  	*buf++ = CHANNELS_ID ;
  	uint32_t bits = 0 ;
  	uint32_t bitsavailable = 0 ;
  	for (uint32_t i=0 ; i < CROSSFIRE_CHANNELS_COUNT ; i += 1 )
		{
  	  uint32_t val = limit(0, CROSSFIRE_CH_CENTER + (((g_chans512[startChan+i]) * 4) / 5), 2*CROSSFIRE_CH_CENTER) ;
  	  bits |= val << bitsavailable ;
  	  bitsavailable += CROSSFIRE_CH_BITS ;
  	  while (bitsavailable >= 8)
			{
  	    *buf++ = bits ;
  	    bits >>= 8 ;
  	    bitsavailable -= 8 ;
  	  }
  	}
  	*buf++ = crc8( crc_start, 23) ;
	}
  return (XfireLength = (buf - Bit_pulses)) ;
}


#endif
