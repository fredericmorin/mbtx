 /*
 * Author - Mike Blandford
 *
 * Based on er9x by Erez Raviv <erezraviv@gmail.com>
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

// This version for ARM based ERSKY9X board

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ersky9x.h"
#include "myeeprom.h"
#include "lcd.h"
#include "drivers.h"
#include "logicio.h"

#ifndef PCBX12D
#ifndef PCBX10
#include "font.lbm"
#define font_5x8_x20_x7f (font)

#include "font_dblsize.lbm"
#define font_10x16_x20_x7f (font_dblsize)

#include "font12x8test.lbm"

#ifndef SMALL
const uint8_t font_se_extra[] = {
#include "font_se_05x07.lbm"
} ;
#endif
const uint8_t font_fr_extra[] = {
#include "font_fr_05x07.lbm"
} ;
const uint8_t font_de_extra[] = {
#include "font_de_05x07.lbm"
} ;
#ifndef SMALL
const uint8_t font_it_extra[] = {
#include "font_it_05x07.lbm"
} ;
const uint8_t font_pl_extra[] = {
#include "font_pl_05x07.lbm"
} ;
#endif
#endif

#ifndef SMALL
const uint8_t font_se_big_extra[] = {
#include "font_se_10x14.lbm"
} ;
#endif
const uint8_t font_fr_big_extra[] = {
#include "font_fr_10x14.lbm"
} ;
const uint8_t font_de_big_extra[] = {
#include "font_de_10x14.lbm"
} ;
#endif


const uint8_t *ExtraFont = NULL ;
const uint8_t *ExtraBigFont = NULL ;

// Local data
uint8_t Lcd_lastPos ;
uint8_t plotType = PLOT_XOR ;

uint8_t plotStack[4] ;
uint8_t plotStackIndex ;

void pushPlotType( uint8_t type )
{
	if ( plotStackIndex < 4 )
	{
		plotStack[plotStackIndex++] = plotType ;
	}
	plotType = type ;
}

void popPlotType()
{
	if ( plotStackIndex )
	{
		plotStackIndex -= 1 ;
	}
	plotType = plotStack[plotStackIndex] ;
}


#ifndef PCBX12D
#ifndef PCBX10
#ifdef GREY_SCALE
uint8_t DisplayBuf[DISPLAY_W*DISPLAY_H/8*4] ;
#else
uint8_t DisplayBuf[DISPLAY_W*DISPLAY_H/8] ;
#endif
#define DISPLAY_END (DisplayBuf+sizeof(DisplayBuf))
#endif
#endif

#if defined(PCBX12D) || defined(PCBX10)
extern uint32_t CurrentFrameBuffer ;

#define SDRAM_BANK_ADDR     ((uint32_t)0xD0000000)
//#define LCD_W	480
//#define LCD_H	272
#endif

#if defined(PCBX12D) || defined(PCBX10)
uint16_t LcdBackground = LCD_BACKGROUND ;
uint16_t LcdForeground = LCD_BLACK ;
uint16_t LcdCustomColour = LCD_BLACK ;
#endif

#if defined(PCBX7) || defined (PCBXLITE) || defined (PCBX9LITE)
#define X9D_OFFSET		0
#define DISPLAY_START (DisplayBuf + 0)
#else // PCBX7/LITE/X3
#ifdef PCBX9D
#define X9D_OFFSET		11
#define DISPLAY_START (DisplayBuf + X9D_OFFSET)
#else
#define DISPLAY_START (DisplayBuf + 0)
#endif
#endif // PCBX7

#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX7) || defined (PCBXLITE) || defined(PCBX9LITE)
uint8_t ExtDisplayBuf[DISPLAY_W*DISPLAY_H/8 + 2] ;
uint16_t ExtDisplayTime ;
uint8_t ExtDisplaySend ;
#endif

#ifndef PCBDUE
#ifdef PCBSKY
// Lookup table for prototype board
#ifdef REVA
const uint8_t Lcd_lookup[] =
{
0x00,0x01,0x80,0x81,0x40,0x41,0xC0,0xC1,0x20,0x21,0xA0,0xA1,0x60,0x61,0xE0,0xE1,
0x10,0x11,0x90,0x91,0x50,0x51,0xD0,0xD1,0x30,0x31,0xB0,0xB1,0x70,0x71,0xF0,0xF1,
0x08,0x09,0x88,0x89,0x48,0x49,0xC8,0xC9,0x28,0x29,0xA8,0xA9,0x68,0x69,0xE8,0xE9,
0x18,0x19,0x98,0x99,0x58,0x59,0xD8,0xD9,0x38,0x39,0xB8,0xB9,0x78,0x79,0xF8,0xF9,
0x04,0x05,0x84,0x85,0x44,0x45,0xC4,0xC5,0x24,0x25,0xA4,0xA5,0x64,0x65,0xE4,0xE5,
0x14,0x15,0x94,0x95,0x54,0x55,0xD4,0xD5,0x34,0x35,0xB4,0xB5,0x74,0x75,0xF4,0xF5,
0x0C,0x0D,0x8C,0x8D,0x4C,0x4D,0xCC,0xCD,0x2C,0x2D,0xAC,0xAD,0x6C,0x6D,0xEC,0xED,
0x1C,0x1D,0x9C,0x9D,0x5C,0x5D,0xDC,0xDD,0x3C,0x3D,0xBC,0xBD,0x7C,0x7D,0xFC,0xFD,
0x02,0x03,0x82,0x83,0x42,0x43,0xC2,0xC3,0x22,0x23,0xA2,0xA3,0x62,0x63,0xE2,0xE3,
0x12,0x13,0x92,0x93,0x52,0x53,0xD2,0xD3,0x32,0x33,0xB2,0xB3,0x72,0x73,0xF2,0xF3,
0x0A,0x0B,0x8A,0x8B,0x4A,0x4B,0xCA,0xCB,0x2A,0x2B,0xAA,0xAB,0x6A,0x6B,0xEA,0xEB,
0x1A,0x1B,0x9A,0x9B,0x5A,0x5B,0xDA,0xDB,0x3A,0x3B,0xBA,0xBB,0x7A,0x7B,0xFA,0xFB,
0x06,0x07,0x86,0x87,0x46,0x47,0xC6,0xC7,0x26,0x27,0xA6,0xA7,0x66,0x67,0xE6,0xE7,
0x16,0x17,0x96,0x97,0x56,0x57,0xD6,0xD7,0x36,0x37,0xB6,0xB7,0x76,0x77,0xF6,0xF7,
0x0E,0x0F,0x8E,0x8F,0x4E,0x4F,0xCE,0xCF,0x2E,0x2F,0xAE,0xAF,0x6E,0x6F,0xEE,0xEF,
0x1E,0x1F,0x9E,0x9F,0x5E,0x5F,0xDE,0xDF,0x3E,0x3F,0xBE,0xBF,0x7E,0x7F,0xFE,0xFF
} ;
#endif
#endif
#endif // DUE

#if defined(PCBX12D) || defined(PCBX10)
void putsTime(uint16_t x,uint16_t y,int16_t tme,uint8_t att,uint8_t att2, uint16_t colour, uint16_t background)
#else
void putsTime(uint8_t x,uint8_t y,int16_t tme,uint8_t att,uint8_t att2)
#endif
{
	div_t qr ;

	uint8_t z = FWNUM*6-2 ;
	if ( att&DBLSIZE )
	{
		if ( att&CONDENSED )
		{
			x += 3 ;
			z = FWNUM*5-2 ;
		}
	}
	if ( tme<0 )
	{
#if defined(PCBX12D) || defined(PCBX10)
		lcd_putcAttColour( x - ((att&DBLSIZE) ? z : FWNUM*3),    y, '-',att, colour, background ) ;
#else
		lcd_putcAtt( x - ((att&DBLSIZE) ? z : FWNUM*3),    y, '-',att);
#endif
		tme = -tme;
	}

#if defined(PCBX12D) || defined(PCBX10)
	lcd_putcAttColour( x, y, ':',att&att2, colour, background ) ;
#else
	lcd_putcAtt( x, y, ':',att&att2);
#endif
	qr = div( tme, 60 ) ;
	if ( att&DBLSIZE )
	{
		if ( att&CONDENSED )
		{
			x += 2 ;
		}
	}
#if defined(PCBX12D) || defined(PCBX10)
	lcd_2_digits( x, y, (uint16_t)qr.quot, att, colour, background ) ;
#else
	lcd_2_digits( x, y, (uint16_t)qr.quot, att ) ;
#endif
	if ( att&DBLSIZE )
	{
		if ( att&CONDENSED )
		{
			x += FWNUM*5-4 ;
		}
		else
		{
			x += FWNUM*6-4 ;
		}
	}
	else
	{
		x += FW*3-4 ;
	}
#if defined(PCBX12D) || defined(PCBX10)
	lcd_2_digits( x, y, (uint16_t)qr.rem, att2, colour, background ) ;
#else
	lcd_2_digits( x, y, (uint16_t)qr.rem, att2 ) ;
#endif
}

void putsVolts(uint8_t x,uint8_t y, uint8_t volts, uint8_t att)
{
	uint8_t option = att & NO_UNIT ;
	att &= ~NO_UNIT ;
	lcd_outdezAtt(x, y, volts, att|PREC1);
	if(!(option&NO_UNIT)) lcd_putcAtt(Lcd_lastPos, y, 'V', 0);
}


void putsVBat(uint8_t x,uint8_t y,uint8_t att)
{
#ifndef MINIMISE_CODE
  att |= g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0;
#endif
	putsVolts(x, y, g_vbat100mV, att);
}

#if defined(PCBX12D) || defined(PCBX10)

extern uint16_t Mimage[] ;
extern uint16_t *SdramImage ;
void lcd_picture( uint16_t i_x, uint16_t i_y )
{
//	uint16_t *p ;
//	uint32_t rows ;
	uint16_t *src = SdramImage ;

extern uint16_t Image_width ;
extern uint16_t Image_height ;

	DMAcopyImage( i_x, i_y, Image_width, Image_height, src ) ;

//	for ( rows = 0 ; rows < Image_height ; rows += 1 )
//	{
//		p = ( uint16_t *) CurrentFrameBuffer ;
//		p += i_x ;
//		p += i_y * LCD_W ;
//    memmove( p, src, Image_width*2 ) ;
//		src += Image_width ;
//		i_y += 1 ;
//	}
}

void lcd_bitmap( uint8_t i_x, uint8_t i_y, PROGMEM *bitmap, uint8_t w, uint8_t h, uint8_t mode, uint16_t colour, uint16_t background )
{
	uint32_t yb ;
	uint32_t x ;
	uint16_t *p ;
	uint8_t mask ;
  uint32_t hb   = (h + 7) / 8 ;
  uint32_t hlast ;

#ifdef INVERT_DISPLAY
	i_x = (LCD_W/2-1) - i_x ; // - (w - 1) ;
	i_y = (LCD_H/2-1) - i_y ; // - (h - 1) ;
//	i_x = (LCD_W/2-1) - i_x ;
//	i_y = (LCD_H/2-1) - i_y ;
#endif

  register bool inv = false ;
	if (mode & INVERS) inv = true ;
	if ( (mode & BLINK) && BLINK_ON_PHASE )
	{
		inv = !inv ;
	}
//	bool inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false ) ;
  for( yb = 0; yb < hb; yb++)
	{
		p = ( uint16_t *) CurrentFrameBuffer ;
		p += i_x*2 ;
		p += i_y * LCD_W * 2 ;
#ifdef INVERT_DISPLAY
		p -= yb * LCD_W * 2 * 8 ;
#else
		p += yb * LCD_W * 2 * 8 ;
#endif
    for(x=0; x < w; x++)
		{
      register uint8_t b = *bitmap++ ;
			if ( inv )
			{
				b = ~b ;
			}
			uint16_t *q = p ;

			hlast = ( h < 8 ) ? h : 9 ;
			for ( mask = 1 ; mask ; mask <<= 1 )
			{
				uint16_t value = (b & mask) ? colour : background ;
				*q = value ;
				*(q+1) = value ;
				*(q+LCD_W) = value ;
				*(q+LCD_W+1) = value ;
#ifdef INVERT_DISPLAY
				q -= LCD_W * 2 ;
#else
				q += LCD_W * 2 ;
#endif
				if ( --hlast == 0 )
				{
					break ;
				}
			}
#ifdef INVERT_DISPLAY
			p -= 2 ;
#else
			p += 2 ;
#endif
    }
		h -= 8 ;
  }
}

void lcd_img( uint8_t i_x, uint8_t i_y, PROGMEM *imgdat, uint8_t idx, uint8_t mode, uint16_t colour, uint16_t background )
{
  register const unsigned char *q = imgdat ;
  register uint8_t w    = *q++ ;
  register uint32_t hb   = *q++ ;
  register uint8_t sze1 = *q++ ;
//	uint32_t yb ;
//	uint32_t x ;

  q += idx * sze1 ;

	lcd_bitmap( i_x, i_y, q, w, hb, mode, colour, background ) ;
}

void lcdSetRefVolt( uint8_t val )
{

}

#else
void lcd_bitmap( uint8_t i_x, uint8_t i_y, PROGMEM *bitmap, uint8_t w, uint8_t h, uint8_t mode )
{
	uint32_t yb ;
	uint32_t x ;

  register bool inv = false ;
	if (mode & INVERS) inv = true ;
	if ( (mode & BLINK) && BLINK_ON_PHASE )
	{
		inv = !inv ;
	}
//	bool inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false ) ;
  for( yb = 0; yb < h; yb++)
	{
    register uint8_t *p = &DISPLAY_START[ (i_y / 8 + yb) * DISPLAY_W + i_x ];
#if PCBX9D
		if ( i_x > 211-X9D_OFFSET )
		{
			p -= DISPLAY_W ;
		}
#endif
    for(x=0; x < w; x++)
		{
      register uint8_t b = *bitmap++ ;
      *p++ = inv ? ~b : b;
    }
  }

}

void lcd_img( uint8_t i_x, uint8_t i_y, PROGMEM *imgdat, uint8_t idx, uint8_t mode )
{
  register const unsigned char *q = imgdat ;
  register uint8_t w    = *q++ ;
  register uint32_t hb   = (*q++ +7) / 8 ;
  register uint8_t sze1 = *q++ ;
//	uint32_t yb ;
//	uint32_t x ;

  q += idx * sze1 ;

	lcd_bitmap( i_x, i_y, q, w, hb, mode ) ;
}
#endif

uint8_t lcd_putc(uint8_t x,uint8_t y,const char c )
{
  return lcd_putcAtt(x,y,c,0);
}

#if defined(PCBX12D) || defined(PCBX10)
uint16_t lcd_putcAttDblColour(uint16_t x,uint16_t y,const char c,uint8_t mode, uint16_t colour, uint16_t background )
{
	if ( c < 31 )
	{
		x = c*FW ;
		if ( (mode & CONDENSED) )
		{
			x = c*8 ;
		}
		else
		{
			x += x ;
		}
		return x ;
	}
	lcdDrawCharBitmapDoubleDma( x*2, y*2, c, mode, colour, background ) ;
	if (mode & CONDENSED)
	{
		x += 8 ;
	}
	else
	{
		x += FW*2 ;
	}
	return x ;


//	uint16_t *p ;
//	uint16_t *r ;
//  uint8_t *q ;
//	uint8_t mask ;
//	int32_t i ;

//	r = ( uint16_t *) CurrentFrameBuffer ;
//	r += x*2 ;
//	r += y * LCD_W * 2 ;
//	x += FW ;
//	uint32_t doNormal = 1 ;
//	unsigned char c_mapped = c ;
//  register bool inv = false ;
//	if (mode & INVERS) inv = true ;
//	if ( (mode & BLINK) && BLINK_ON_PHASE )
//	{
//		inv = !inv ;
//	}
////  register bool   inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);

//	if ( (mode & CONDENSED) )
//	{
//		doNormal = 0 ;

//		if ( doNormal == 0 )
//		{
//			if ( (c!=0x2E)) x+=8-FW; //check for decimal point
//		/* each letter consists of 8 top bytes followed by
//	 	* five bottom by 8 bottom bytes (16 bytes per
//	 	* char) */
//			if( c < 0xC0 )
//			{
//				c_mapped = c - 0x20 ;
//				q = (uint8_t *) &font_12x8[c_mapped*14] ;
//			}
//			else
//			{
//				q = (uint8_t *) &font_12x8[0] ;
//			}

//			for( i=7 ; i>=0 ; i-- )
//			{
//				uint8_t b1 ;
//				uint8_t b3 ;

//  		  /*top byte*/
//   		  b1 = *q ;
//  		  /*bottom byte*/
//   		  b3 = *(q+7) ;
//   		  q++;
//				if ( i == 0 )
//				{
//					b1 = 0 ;
//					b3 = 0 ;
//				}
//    		if(inv)
//				{
//				  b1=~b1;
//				  b3=~b3;
//    		}
//				p = r ;
//				for ( mask = 1 ; mask ; mask <<= 1 )
//				{
//					uint16_t v = ( b1 & mask ) ? colour : background ;
//					*p = v ;
//					*(p+1) = v ;
//					*(p+LCD_W) = v ;
//					*(p+LCD_W+1) = v ;
//					v = ( b3 & mask ) ? colour : background ;
//					*(p+LCD_W*16) = v ;
//					*(p+LCD_W*16+1) = v ;
//					*(p+LCD_W*17) = v ;
//					*(p+LCD_W*17+1) = v ;
//					p += LCD_W * 2 ;
//				}
//				r += 2 ;
//    	}
//		}
//	}

//	if ( doNormal )
//	{
//		if ( (c!=0x2E)) x+=FW; //check for decimal point
//	/* each letter consists of ten top bytes followed by
// 	* five bottom by ten bottom bytes (20 bytes per
// 	* char) */
//		q = (uint8_t *) &font_dblsize[(c-0x20)*20] ;
//	  for( i=11 ; i>=0 ; i-- )
//		{
//			uint8_t b1 ;
//			uint8_t b2 ;

//		  /*top byte*/
//	    b1 = i>1 ? *q : 0;
//		  /*bottom byte*/
//	    b2 = i>1 ? *(q+10) : 0;
//			if ( inv )
//			{
//				b1 = ~b1 ;
//				b2 = ~b2 ;
//			}
//	    q++;
//			p = r ;
//			for ( mask = 1 ; mask ; mask <<= 1 )
//			{
//				uint16_t v = ( b1 & mask ) ? colour : background ;
//				*p = v ;
//				*(p+1) = v ;
//				*(p+LCD_W) = v ;
//				*(p+LCD_W+1) = v ;
//				v = ( b2 & mask ) ? colour : background ;
//				*(p+LCD_W*16) = v ;
//				*(p+LCD_W*16+1) = v ;
//				*(p+LCD_W*17) = v ;
//				*(p+LCD_W*17+1) = v ;
//				p += LCD_W * 2 ;
//			}
//			r += 2 ;
//		}
//	}
//	return x ;
}

//uint16_t lcd_putcAttDbl(uint16_t x,uint16_t y,const char c,uint8_t mode)
//{
//	uint16_t *p ;
//	uint16_t *r ;
//  uint8_t *q ;
//	uint8_t mask ;
//	int32_t i ;

//	r = ( uint16_t *) CurrentFrameBuffer ;
//	r += x*2 ;
//	r += y * LCD_W * 2 ;
//	if ( c < 22 )
//	{
//		x = c*FW ;
//		if ( (mode & CONDENSED) )
//		{
//			x = c*8 ;
//		}
//			else
//		{
//			x += x ;
//		}
//		return x ;
//	}
//	x += FW ;
//	uint32_t doNormal = 1 ;
//	unsigned char c_mapped = c ;

//	register bool inv = false ;
//	if (mode & INVERS) inv = true ;
//	if ( (mode & BLINK) && BLINK_ON_PHASE )
//	{
//		inv = !inv ;
//	}
////  register bool   inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);

//	if ( (mode & CONDENSED) )
//	{
//		doNormal = 0 ;

//		if ( doNormal == 0 )
//		{
//			if ( (c!=0x2E)) x+=8-FW; //check for decimal point
//		/* each letter consists of 8 top bytes followed by
//	 	* five bottom by 8 bottom bytes (16 bytes per
//	 	* char) */
//			if( c < 0xC0 )
//			{
//				c_mapped = c - 0x20 ;
//				q = (uint8_t *) &font_12x8[c_mapped*14] ;
//			}
//			else
//			{
//				q = (uint8_t *) &font_12x8[0] ;
//			}

//			for( i=7 ; i>=0 ; i-- )
//			{
//				uint8_t b1 ;
//				uint8_t b3 ;

//  		  /*top byte*/
//   		  b1 = *q ;
//  		  /*bottom byte*/
//   		  b3 = *(q+7) ;
//   		  q++;
//				if ( i == 0 )
//				{
//					b1 = 0 ;
//					b3 = 0 ;
//				}
//    		if(inv)
//				{
//				  b1=~b1;
//				  b3=~b3;
//    		}
//				p = r ;
//				for ( mask = 1 ; mask ; mask <<= 1 )
//				{
//					if ( b1 & mask )
//					{
//						*p ^= 0xFFFF ;
//						*(p+1) ^= 0xFFFF ;
//						*(p+LCD_W) ^= 0xFFFF ;
//						*(p+LCD_W+1) ^= 0xFFFF ;
//					}
//					if ( b3 & mask )
//					{
//						*(p+LCD_W*16) ^= 0xFFFF ;
//						*(p+LCD_W*16+1) ^= 0xFFFF ;
//						*(p+LCD_W*17) ^= 0xFFFF ;
//						*(p+LCD_W*17+1) ^= 0xFFFF ;
//					}
//					p += LCD_W * 2 ;
//				}
//				r += 2 ;
//    	}
//		}
//	}

//	if ( doNormal )
//	{
//		if ( (c!=0x2E)) x+=FW; //check for decimal point
//	/* each letter consists of ten top bytes followed by
// 	* five bottom by ten bottom bytes (20 bytes per
// 	* char) */
//		q = (uint8_t *) &font_dblsize[(c-0x20)*20] ;
//	  for( i=11 ; i>=0 ; i-- )
//		{
//			uint8_t b1 ;
//			uint8_t b2 ;

//		  /*top byte*/
//	    b1 = i>1 ? *q : 0;
//		  /*bottom byte*/
//	    b2 = i>1 ? *(q+10) : 0;
//	    q++;
//			p = r ;
//			for ( mask = 1 ; mask ; mask <<= 1 )
//			{
//				if ( b1 & mask )
//				{
//					*p ^= 0xFFFF ;
//					*(p+1) ^= 0xFFFF ;
//					*(p+LCD_W) ^= 0xFFFF ;
//					*(p+LCD_W+1) ^= 0xFFFF ;
//				}
//				if ( b2 & mask )
//				{
//					*(p+LCD_W*16) ^= 0xFFFF ;
//					*(p+LCD_W*16+1) ^= 0xFFFF ;
//					*(p+LCD_W*17) ^= 0xFFFF ;
//					*(p+LCD_W*17+1) ^= 0xFFFF ;
//				}
//				p += LCD_W * 2 ;
//			}
//			r += 2 ;
//		}
//	}
//	return x ;

//}

uint16_t lcd_putcAttColour(uint16_t x,uint16_t y,const char c,uint8_t mode, uint16_t colour, uint16_t background )
{
	if ( (mode & BLINK) && BLINK_ON_PHASE )
	{
		mode ^= INVERS ;
	}
	if ( mode & DBLSIZE )
	{
		return lcd_putcAttDblColour( x, y, c, mode, colour ) ;
	}
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
  	if(mode&DBLSIZE)
		{
			if ( (mode & CONDENSED) )
			{
				x = c*8 ;
			}
		 	else
			{
				x += x ;
			}
		}
		return x ;
	}

	lcdDrawCharBitmapDma( x*2, y*2, c, mode, colour, background ) ;
	x += FW ;
	if (mode & CONDENSED)
	{
		x += FWNUM-FW ;
	}
	return x ;

//	uint16_t *p ;
//  uint8_t *q ;
//	uint8_t mask ;
//	uint16_t v ;
//	p = ( uint16_t *) CurrentFrameBuffer ;
//	p += x*2 ;
//	p += y * LCD_W * 2 ;

//	q = (uint8_t *) &font_5x8_x20_x7f[(c-0x20)*5] ;

//  register bool inv = false ;
//	if (mode & INVERS) inv = true ;
//	if ( (mode & BLINK) && BLINK_ON_PHASE )
//	{
//		inv = !inv ;
//	}
////  register bool   inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);

//	uint8_t condense = 0 ;

//	if (mode & CONDENSED)
//	{
////		*p = inv ? ~0 : 0;
//		p += 2 ;
//		condense = 1 ;
//		x += FWNUM-FW ;
//	}

//	for ( mask = 1 ; mask ; mask <<= 1 )
//	{
//		uint16_t *r = p ;
//		uint8_t b ;

//		b = *q ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? colour : background ;
//		*r = v ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		if ( condense == 0 )
//		{
//			b = *(q+1) ;
//			if ( inv ) b = ~b ;
//			v = ( b & mask ) ? colour : background ;
//			*r = v ;
//			*(r+1) = v ;
//			*(r+LCD_W) = v ;
//			*(r+LCD_W+1) = v ;
//			r += 2 ;
//		}
//		b = *(q+2) ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? colour : background ;
//		*r = v ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		b = *(q+3) ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? colour : background ;
//		*r = v ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		b = *(q+4) ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? colour : background ;
//		*r = v ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		v = inv ? colour : background ;
//		*r = v ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v ;
//		*(r+LCD_W+1) = v ;

//		p += LCD_W * 2 ;
//	}
//	return x ;
}

uint16_t lcd_putcAtt(uint16_t x,uint16_t y,const char c,uint8_t mode, uint16_t background )
{
	if ( mode & DBLSIZE )
	{
		return lcd_putcAttDblColour( x, y, c, mode ) ;
	}
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
  	if(mode&DBLSIZE)
		{
			if ( (mode & CONDENSED) )
			{
				x = c*8 ;
			}
		 	else
			{
				x += x ;
			}
		}
		return x ;
	}

//extern void lcdDrawCharBitmapDma( uint16_t x, uint16_t y, uint8_t chr, uint32_t mode, uint16_t background ) ;
//	if ( (mode & BOLD) == 0 )
//	{
		if ( (mode & BLINK) && BLINK_ON_PHASE )
		{
			mode ^= INVERS ;
		}
		lcdDrawCharBitmapDma( x*2, y*2, c, mode & (INVERS | CONDENSED | BOLD), LcdForeground, background ) ;
		return (mode & CONDENSED) ? x + FWNUM : x + FW ;
//	}

//	uint16_t *p ;
//  uint8_t *q ;
//	uint8_t mask ;
//	uint16_t v ;
//	uint16_t ov ;

//  register bool inv = false ;
//	if (mode & INVERS) inv = true ;
//	if ( (mode & BLINK) && BLINK_ON_PHASE )
//	{
//		inv = !inv ;
//	}

//	p = ( uint16_t *) CurrentFrameBuffer ;
//	p += x*2 ;
//	p += y * LCD_W * 2 ;

//	x += FW ;

////  register bool   inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);

//	q = (uint8_t *) &font_5x8_x20_x7f[(c-0x20)*5] ;

//	uint8_t condense = 0 ;

//	if (mode & CONDENSED)
//	{
////		*p = inv ? ~0 : 0;
//		p += 2 ;
//		condense=1;
//		x += FWNUM-FW ;
//	}

//	for ( mask = 1 ; mask ; mask <<= 1 )
//	{
//		uint16_t *r = p ;
//		uint8_t b ;

//		b = *q ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? 0 : background ;
//		*r = v ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		ov = (mode & BOLD) ? v : 0xFFFF ;
//		if ( condense == 0 )
//		{
//			b = *(q+1) ;
//			if ( inv ) b = ~b ;
//			v = ( b & mask ) ? 0 : background ;
//			*r = v & ov ;
//			*(r+1) = v ;
//			*(r+LCD_W) = v & ov ;
//			*(r+LCD_W+1) = v ;
//			r += 2 ;
//			ov = (mode & BOLD) ? v : 0xFFFF ;
//		}
//		b = *(q+2) ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? 0 : background ;
//		*r = v & ov ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v & ov ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		ov = (mode & BOLD) ? v : 0xFFFF ;
//		b = *(q+3) ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? 0 : background ;
//		*r = v & ov ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v & ov ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		ov = (mode & BOLD) ? v : 0xFFFF ;
//		b = *(q+4) ;
//		if ( inv ) b = ~b ;
//		v = ( b & mask ) ? 0 : background ;
//		*r = v & ov ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v & ov ;
//		*(r+LCD_W+1) = v ;
//		r += 2 ;
//		ov = (mode & BOLD) ? v : 0xFFFF ;
//		v = ( inv ) ? 0 : background ;
//		*r = v & ov ;
//		*(r+1) = v ;
//		*(r+LCD_W) = v & ov ;
//		*(r+LCD_W+1) = v ;
//		p += LCD_W * 2 ;
//	}
//	return x ;
}

//uint16_t lcd_putcAttSmall(uint16_t x,uint16_t y,const char c,uint8_t mode, uint16_t colour)
//{
//	uint16_t *p ;
//  uint8_t *q ;
//	uint8_t mask ;

//	p = ( uint16_t *) CurrentFrameBuffer ;
//	p += x ;
//	p += y * LCD_W ;
//	// p -> char position
//	q = (uint8_t *) &font_5x8_x20_x7f[(c-0x20)*5] ;
//	for ( mask = 1 ; mask ; mask <<= 1 )
//	{
//		if ( *q & mask )
//		{
//			*p = colour ;
//		}
//		if ( *(q+1) & mask )
//		{
//			*(p+1) = colour ;
//		}
//		if ( *(q+2) & mask )
//		{
//			*(p+2) = colour ;
//		}
//		if ( *(q+3) & mask )
//		{
//			*(p+3) = colour ;
//		}
//		if ( *(q+4) & mask )
//		{
//			*(p+4) = colour ;
//		}
//		p += LCD_W ;
//	}
//	return x+6 ;
//}

void lcd_putsnAttColour( uint16_t x, uint16_t y, const char * s,uint8_t len,uint8_t mode, uint16_t colour )
{
	register char c ;
  while(len!=0) {
    c = *s++ ;
#ifdef BOOT
		if ( c == 0 )
		{
			break ;
		}
#endif

    x = lcd_putcAttColour( x, y, c, mode, colour ) ;
    len--;
  }
}

#else
static uint8_t *dispBufAddress( uint8_t x, uint8_t y )
{
#if (DISPLAY_W==128)
  return &DISPLAY_START[ (y & 0xF8) * 16 + x ];
#else
	return &DISPLAY_START[ y / 8 * DISPLAY_W + x ];
#endif
}

// invers: 0 no 1=yes 2=blink
uint8_t lcd_putcAtt(uint8_t x,uint8_t y,const char c,uint8_t mode)
{
	register int32_t i ;
	register uint8_t *p    = dispBufAddress( x, y ) ;
#if PCBX9D
	if ( x > 211-X9D_OFFSET )
	{
		p -= DISPLAY_W ;
	}
#endif
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
  	if(mode&DBLSIZE)
		{
			if ( (mode & CONDENSED) )
			{
				x = c*8 ;
			}
		 	else
			{
				x += x ;
			}
		}
		return x ;
	}
	x += FW ;
  register uint8_t    *q ;
	if( c < 0xC0 )
	{
		q = (uint8_t *) &font_5x8_x20_x7f[(c-0x20)*5] ;
	}
	else
	{
		if ( ExtraFont )
		{
			q = (uint8_t *) &ExtraFont[(c-0xC0)*5] ;
		}
		else
		{
			q = (uint8_t *) &font_5x8_x20_x7f[0] ;
		}
	}
  register bool inv = false ;
	if (mode & INVERS) inv = true ;
	if ( (mode & BLINK) && BLINK_ON_PHASE )
	{
		inv = !inv ;
	}

	if(mode&DBLSIZE)
  {
		uint32_t doNormal = 1 ;
	  unsigned char c_mapped = c ;

		if ( (mode & CONDENSED) )
		{
			doNormal = 0 ;

			if ( doNormal == 0 )
			{
				if ( (c!=0x2E)) x+=8-FW; //check for decimal point
			/* each letter consists of 8 top bytes followed by
	 		* five bottom by 8 bottom bytes (16 bytes per
	 		* char) */
				if( c < 0xC0 )
				{
					c_mapped = c - 0x20 ;
					q = (uint8_t *) &font_12x8[c_mapped*14] ;
				}
				else
				{
					q = (uint8_t *) &font_12x8[0] ;
				}

				for( i=7 ; i>=0 ; i-- )
				{
					uint8_t b1 ;
					uint8_t b3 ;

  		  	/*top byte*/
   		  	b1 = *q ;
  		  	/*bottom byte*/
   		  	b3 = *(q+7) ;
   		  	q++;
					if ( i == 0 )
					{
						b1 = 0 ;
						b3 = 0 ;
					}
    		  if(inv)
					{
				    b1=~b1;
				    b3=~b3;
    		  }

    		  if(&p[DISPLAY_W+1] < DISPLAY_END)
					{
    		    p[0]=b1;
    		    p[DISPLAY_W] = b3;
    		    p+=1;
    		  }
    		}
			}
		}
		if ( doNormal )
		{
			if ( (c!=0x2E)) x+=FW; //check for decimal point
		/* each letter consists of ten top bytes followed by
	 	* five bottom by ten bottom bytes (20 bytes per
	 	* char) */
	  	unsigned char c_mapped ;
			if( c < 0xC0 )
			{
				c_mapped = c - 0x20 ;
				q = (uint8_t *) &font_10x16_x20_x7f[(c_mapped)*20] ;
			}
			else
			{
				if ( ExtraBigFont )
				{
					q = (uint8_t *) &ExtraBigFont[(c-0xC0)*10] ;
				}
				else
				{
					q = (uint8_t *) &font_10x16_x20_x7f[0] ;
				}
			}
    	for( i=5 ; i>=0 ; i-- )
			{
				uint8_t b1 ;
				uint8_t b3 ;
				uint8_t b2 ;
				uint8_t b4 ;

				if( c < 0xC0 )
				{
	  	  	/*top byte*/
    	  	b1 = i>0 ? *q : 0;
	  	  	/*bottom byte*/
    	  	b3 = i>0 ? *(q+10) : 0;
	  	  	/*top byte*/
    	  	b2 = i>0 ? *(++q) : 0;
	  	  	/*bottom byte*/
    	  	b4 = i>0 ? *(q+10) : 0;
    	  	q++;
				}
				else
				{
	  	  	/*top byte*/
    	  	b1 = i>0 ? *q++ : 0 ;
	  	  	/*bottom byte*/
    	  	b3 = i>0 ? *q++ : 0 ;
	  	  	/*top byte*/
    	  	b2 = i>0 ? *q++ : 0 ;
	  	  	/*bottom byte*/
    	  	b4 = i>0 ? *q++ : 0 ;
				}
    	  if(inv)
				{
			    b1=~b1;
			    b2=~b2;
			    b3=~b3;
			    b4=~b4;
    	  }

    	  if(&p[DISPLAY_W+1] < DISPLAY_END)
				{
    	    p[0]=b1;
    	    p[1]=b2;
    	    p[DISPLAY_W] = b3;
    	    p[DISPLAY_W+1] = b4;
    	    p+=2;
    	  }
    	}
		}
  }
  else
  {
		uint8_t condense=0;

		if (mode & CONDENSED)
		{
			*p = inv ? ~0 : 0;
			p += 1 ;
			condense=1;
			x += FWNUM-FW ;
		}

		y &= 7 ;
		if ( y )
		{ // off grid
    	for( i=5 ; i!=0 ; i-- )
			{
      	uint16_t b = *q++ ;
				if ( inv )
				{
					b = ~b & 0x00FF ;
				}
				b <<= y ;
      	if(p<DISPLAY_END) *p ^= b ;
      	if(&p[DISPLAY_W] < DISPLAY_END)
				{
	        p[DISPLAY_W] ^= b >> 8 ;
				}
				p += 1 ;
			}
			if ( inv )
			{
				uint16_t b = 0xFF ;
     		if(p<DISPLAY_END) *p ^= ( b <<= y) ;
     		if(&p[DISPLAY_W] < DISPLAY_END) p[DISPLAY_W] ^= b >> 8 ;
			}
		}
		else
		{
			uint8_t oldb = 0 ;
    	for( i=5 ; i!=0 ; i-- )
			{
    	  uint8_t b = *q++ ;
  		  if (condense && i==4)
				{
    	    /*condense the letter by skipping column 4 */
    	    continue;
    	  }
    	  if(p<DISPLAY_END) *p++ = inv ? ~(b|oldb) : (b|oldb) ;
				if (mode & BOLD)
				{
					oldb = b ;
				}
    	}
    	if(p<DISPLAY_END) *p++ = inv ? ~oldb : oldb ;
		}
  }
	return x ;
}

#endif

// Puts sub-string from string options
// First byte of string is sub-string length
// idx is index into string (in length units)
// Output length characters
void lcd_putsAttIdx(uint8_t x,uint8_t y,const char * s,uint8_t idx,uint8_t att)
{
	uint8_t length ;
	length = *s++ ;

  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
}

//void lcd_putsAttIdx_right( uint8_t y,const prog_char * s,uint8_t idx,uint8_t att)
//{
//	uint8_t x = 20 - pgm_read_byte(s) ;
//	lcd_putsAttIdx( x, y, s, idx, att ) ;
//}

void lcd_putsnAtt(uint8_t x,uint8_t y, const char * s,uint8_t len,uint8_t mode)
{
	register char c ;
  while(len!=0) {
    c = *s++ ;
#ifdef BOOT
		if ( c == 0 )
		{
			break ;
		}
#endif

    x = lcd_putcAtt(x,y,c,mode);
    len--;
  }
}

void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len)
{
  lcd_putsnAtt( x,y,s,len,0);
}

uint8_t lcd_putsAtt( uint8_t x, uint8_t y, const char *s, uint8_t mode )
{

  while(1)
	{
    char c = *s++ ;
    if(!c) break ;
		if ( c == 31 )
		{

			if ( (y += FH) >= DISPLAY_H )	// Screen height
			{
				break ;
			}

			x = 0 ;
		}
		else
		{
    	x = lcd_putcAtt(x,y,c,mode) ;
		}
  }
  return x;
}

void lcd_puts_Pleft( uint8_t y, const char *s )
{
  lcd_putsAtt( 0, y, s, 0);
}

// This routine skips 'skip' strings, then displays the rest
void lcd_puts_Pskip(uint8_t y,const char * s, uint8_t skip)
{
	while ( skip )
	{
    char c = *s++ ;
    if(!c) return ;
		if ( c == 31 )
		{
			skip -= 1 ;
		}
	}
  lcd_putsAtt( 0, y, s, 0);
}

void lcd_puts_P( uint8_t x, uint8_t y, const char *s )
{
  lcd_putsAtt( x, y, s, 0);
}


void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
{
	uint8_t i ;
  x+=FWNUM*4;
  for(i=0; i<4; i++)
  {
    x-=FWNUM;
    char c = val & 0xf;
    c = c>9 ? c+'A'-10 : c+'0';
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
    val>>=4;
  }
}

void lcd_outhex2(uint8_t x,uint8_t y,uint8_t val)
{
  register int i ;
  x+=FWNUM*2;
  for(i=0; i<2; i++)
  {
    x-=FWNUM;
    char c = val & 0xf;
    c = c>9 ? c+'A'-10 : c+'0';
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
    val>>=4;
  }
}

void lcd_outdez( uint8_t x, uint8_t y, int16_t val )
{
  lcd_outdezAtt(x,y,val,0);
}

void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint16_t mode )
{
  lcd_outdezNAtt( x,y,val,mode,5);
}

void lcd_2_digits( uint8_t x, uint8_t y, uint8_t value, uint16_t attr )
{
	lcd_outdezNAtt( x, y, value, attr + LEADING0, 2 ) ;
}

#define PREC(n) ((n&0x20) ? ((n&0x10) ? 2 : 1) : 0)

uint8_t lcd_outdezNAtt( uint8_t x, uint8_t y, int32_t val, uint16_t mode, int8_t len )
{
  uint8_t fw = FWNUM;
  uint8_t prec = PREC(mode);
  int32_t tmp = abs(val);
  uint8_t xn = 0;
  uint8_t ln = 2;
  char c;
  uint8_t xinc ;
	uint8_t fullwidth = 0 ;
	int32_t i ;
	if ( len < 0 )
	{
		fullwidth = 1 ;
		len = -len ;
	}
#ifndef PCBX12D
#ifndef PCBX10
	if (mode & BOLD)
	{
		fw = FW ;
	}
#endif
#endif
  if (mode & DBLSIZE)
  {
		if ( (mode & CONDENSED) )
		{
    	fw = 8 ;
    	xinc = 8 ;
    	Lcd_lastPos = 8 ;
		}
		else
		{
    	fw += FWNUM ;
    	xinc = 2*FWNUM;
    	Lcd_lastPos = 2*FW;
		}
  }
  else
  {
    xinc = FWNUM ;
    Lcd_lastPos = FW;
  }

  if (mode & LEFT)
	{
    if(val<0)
    {
      x += fw;
    }
    if (tmp >= 1000)
		{
	    if (tmp >= 100000)
  	    x += fw;
	    if (tmp >= 10000)
  	    x += fw;
			x += 3 * fw ;
		}
		else
		{
	    if (tmp >= 100)
  	    x += fw;
    	if (tmp >= 10)
      	x += fw;
		}
//    if (tmp >= 10000)
//      x += fw;
//    if (tmp >= 1000)
//      x += fw;
//    if (tmp >= 100)
//      x += fw;
//    if (tmp >= 10)
//      x += fw;
    if ( prec )
    {
      if ( prec == 2 )
      {
        if ( tmp < 100 )
        {
          x += fw;
        }
      }
      if ( tmp < 10 )
      {
        x+= fw;
      }
    }
  }
  else
  {
    x -= xinc;
  }
  Lcd_lastPos += x ;

//  if ( prec == 2 )
//  {
//    mode -= LEADING0;  // Can't have PREC2 and LEADING0
//  }

  for ( i=1; i<=len; i++)
	{
    c = (tmp % 10) + '0';
		lcd_putcAtt(x, y, c, mode);
    if (prec==i)
		{
      if (mode & DBLSIZE)
			{
        xn = x ;
				if ( (mode & CONDENSED) )
				{
					x -= 1 ;
        	xn = x-1 ;
				}
				else
				{
	        if(c=='2' || c=='3' || c=='1') ln++;
	        uint8_t tn = (tmp/10) % 10;
	        if(tn==2 || tn==4)
					{
	          if (c=='4')
						{
	            xn++;
	          }
	          else
						{
	            xn--; ln++;
	          }
	        }
				}
      }
      else
			{
        x -= 2;
				uint8_t oldPlotType = plotType ;
				plotType = PLOT_COLOUR ;
        if (mode & INVERS)
				{
          lcd_vline(x+1, y, 8);
				}
				plotType = oldPlotType ;
        lcd_plot(x+1, y+6);
      }
      if (tmp >= 10)
        prec = 0;
    }
    tmp /= 10;
    if (!tmp)
    {
      if (prec)
      {
        if ( prec == 2 )
        {
          if ( i > 1 )
          {
            prec = 0 ;
          }
        }
        else
        {
          prec = 0 ;
        }
      }
      else if (mode & LEADING0)
			{
				if ( fullwidth == 0 )
				{
        	mode -= LEADING0;
				}
			}
      else
        break;
    }
    x-=fw;
  }
  if (xn)
	{
    lcd_hline(xn, y+2*FH-4, ln);
    lcd_hline(xn, y+2*FH-3, ln);
  }
  if(val<0) lcd_putcAtt(x-fw,y,'-',mode);
	return 0 ;		// Stops compiler creating two sets of POPS, saves flash
}


#ifndef PCBX12D
#ifndef PCBX10
void lcd_write_bits( uint8_t *p, uint8_t mask )
{
  if(p<DISPLAY_END)
	{
		uint8_t temp = *p ;
		if ( plotType != PLOT_XOR )
		{
			temp |= mask ;
		}
		if ( plotType != PLOT_BLACK )
		{
			temp ^= mask ;
		}
		*p = temp ;
	}
}
#endif
#endif


void lcd_plot( register uint8_t x, register uint8_t y )
{
	uint8_t *p = dispBufAddress( x, y ) ;
//#ifdef GREY_SCALE
//  register uint8_t *p   = &DISPLAY_START[ y / 2 * DISPLAY_W + x ];
//#else
//  register uint8_t *p   = &DISPLAY_START[ y / 8 * DISPLAY_W + x ];
//#endif
#ifdef GREY_SCALE
	lcd_write_bits( p, (y&1) ? 0xF0 : 0x0F ) ;
#else
	lcd_write_bits( p, BITMASK(y%8) ) ;
#endif
}

void lcd_hlineStip( unsigned char x, unsigned char y, int16_t w, uint8_t pat )
{
  if(w<0) {x+=w; w=-w;}
#ifdef GREY_SCALE
  register uint8_t *p = &DISPLAY_START[ y / 2 * DISPLAY_W + x ] ;
	register uint8_t msk = (y&1) ? 0xF0 : 0x0F ;
#else
	uint8_t *p = dispBufAddress( x, y ) ;
	register uint8_t msk = BITMASK(y%8) ;
#endif
  while(w)
	{
    if ( p>=DISPLAY_END)
    {
      break ;
    }
    if(pat&1)
		{
			lcd_write_bits( p, msk ) ;
      pat = (pat >> 1) | 0x80;
    }
		else
		{
      pat = pat >> 1;
    }
    w--;
    p++;
  }
}

#if defined(PCBX12D) || defined(PCBX10)
void lcdDrawFilledRect( uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t pat, uint8_t att )
{
	uint8_t oldPlotType = plotType ;
	uint32_t i ;

	for ( i=y; i<y+h; i++ )
	{
  	lcd_hlineStip( x, i, w, 0xff ) ;
	}
	plotType = oldPlotType ;
}
#else
void lcdDrawFilledRect( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pat, uint8_t att )
{
	uint8_t oldPlotType = plotType ;

	uint32_t i ;
	plotType = PLOT_BLACK ;
	if ( att & ERASE )
	{
		plotType = PLOT_WHITE ;
	}
  for ( i=y; i<y+h; i++ )
	{
  	lcd_hlineStip( x, i, w, 0xff ) ;

//    if ((att&ROUND) && (i==y || i==y+h-1))
//      lcdDrawHorizontalLine(x+1, i, w-2, pat, att);
//    else
//      lcdDrawHorizontalLine(x, i, w, pat, att);
//    pat = (pat >> 1) + ((pat & 1) << 7);
  }
	plotType = oldPlotType ;
}
#endif


void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent )
{
	uint8_t solid ;
	if ( percent > 100 )
	{
		percent = 100 ;
	}
	solid = (w-2) * percent / 100 ;
	lcd_rect( x, y, w, h ) ;

	if ( solid )
	{
		w = y + h - 1 ;
		y += 1 ;
		x += 1 ;
		while ( y < w )
		{
 			lcd_hline(x, y, solid ) ;
			y += 1 ;
		}
	}
}

// Reverse video 8 pixels high, w pixels wide
// Vertically on an 8 pixel high boundary
void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink )
{
	if ( blink && BLINK_ON_PHASE )
	{
		return ;
	}
	uint8_t end = x + w ;
	uint8_t *p = dispBufAddress( x, y ) ;
//#ifdef GREY_SCALE
//  register uint8_t *p = &DISPLAY_START[ y / 2 * DISPLAY_W + x ] ;
//#else
//  register uint8_t *p = &DISPLAY_START[ y / 8 * DISPLAY_W + x ] ;
//#endif

	y &= 7 ;
	if ( y )
	{ // off grid
		while ( x < end )
		{
#ifdef GREY_SCALE
			*p ^= 0xFF ;
			*(p+DISPLAY_W) ^= 0xFF ;
			*(p+DISPLAY_W*2) ^= 0xFF ;
			*(p+DISPLAY_W*3) ^= 0xFF ;
			p += 1 ;
#else
     	uint16_t b = 0xFF ;
			b <<= y ;
			if(p<DISPLAY_END) *p ^= b ;
	    if(&p[DISPLAY_W] < DISPLAY_END) p[DISPLAY_W] ^= b >> 8 ;
			p += 1 ;
#endif
			x += 1 ;
		}
	}
	else
	{
		while ( x < end )
		{
#ifdef GREY_SCALE
			*p ^= 0xFF ;
			*(p+DISPLAY_W) ^= 0xFF ;
			*(p+DISPLAY_W*2) ^= 0xFF ;
			*(p+DISPLAY_W*3) ^= 0xFF ;
			p += 1 ;
#else
			if(p<DISPLAY_END) *p++ ^= 0xFF ;
#endif
			x += 1 ;
		}
	}
}

void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h )
{
	uint8_t oldPlotType = plotType ;
	if ( oldPlotType != PLOT_WHITE )
	{
		plotType = PLOT_BLACK ;
	}
  lcd_vline(x, y, h ) ;
	if ( w > 1 )
	{
  	lcd_vline(x+w-1, y, h ) ;
	}
 	lcd_hline(x+1, y+h-1, w-2 ) ;
 	lcd_hline(x+1, y, w-2 ) ;
	plotType = oldPlotType ;
}

void lcd_hline( uint8_t x, uint8_t y, int8_t w )
{
  lcd_hlineStip(x,y,w,0xff);
}

#ifndef PCBX12D
#ifndef PCBX10
void lcd_vline( uint8_t x, uint8_t y, int8_t h )
{
  if (h<0) { y+=h; h=-h; }

#ifdef GREY_SCALE
  register uint8_t *p   = &DISPLAY_START[ y / 2 * DISPLAY_W + x ];
  y &= 0x01 ;
	if ( y )
	{
    uint8_t msk = 0x0F ;
    h -= 1 ;
		lcd_write_bits( p, msk ) ;
    p += DISPLAY_W ;
	}

  while( h >= 2 )
	{
		h -= 2 ;
		lcd_write_bits( p, 0xFF ) ;
    p += DISPLAY_W ;
  }
	if ( h > 0 )
	{
  	lcd_write_bits( p, 0xF0 ) ;
	}
#else
	uint8_t *p = dispBufAddress( x, y ) ;
  y &= 0x07 ;
	if ( y )
	{
    uint8_t msk = ~(BITMASK(y)-1) ;
    h -= 8-y ;
    if (h < 0)
      msk -= ~(BITMASK(8+h)-1) ;
		lcd_write_bits( p, msk ) ;
    p += DISPLAY_W ;
	}

  while( h >= 8 )
	{
		h -= 8 ;
		lcd_write_bits( p, 0xFF ) ;
    p += DISPLAY_W ;
  }
	if ( h > 0 )
	{
  	lcd_write_bits( p, (BITMASK(h)-1) ) ;
	}
#endif
}
#endif
#endif

void lcd_line( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t pat, uint8_t att )
{
  int dx = x2-x1 ;      /* the horizontal distance of the line */
  int dy = y2-y1 ;      /* the vertical distance of the line */
  int dxabs = abs(dx) ;
  int dyabs = abs(dy) ;
  int sdx = dx > 0 ? 1 : dx < 0 ? -1 : 0 ;
  int sdy = dy > 0 ? 1 : dy < 0 ? -1 : 0 ;
  int x = dyabs>>1 ;
  int y = dxabs>>1 ;
  int px = x1 ;
  int py = y1 ;

	lcd_plot( x2, y2 ) ;

  if (dxabs >= dyabs)
	{
    /* the line is more horizontal than vertical */
    for (int i=0; i<dxabs; i++)
		{
      if ((1<<(px%8)) & pat)
			{
//        lcdDrawPoint(px, py, att);
        lcd_plot(px, py ) ;
      }
      y += dyabs ;
      if (y>=dxabs)
			{
        y -= dxabs ;
        py += sdy ;
      }
      px += sdx ;
    }
  }
  else
	{
    /* the line is more vertical than horizontal */
    for (int i=0; i<dyabs; i++)
		{
      if ((1<<(py%8)) & pat)
			{
//        lcdDrawPoint(px, py, att);
        lcd_plot(px, py ) ;
      }
      x += dxabs;
      if (x >= dyabs)
			{
        x -= dyabs ;
        px += sdx ;
      }
      py += sdy ;
    }
  }
}


#ifdef SIMU
bool lcd_refresh = true;
uint8_t lcd_buf[DISPLAY_W*DISPLAY_H/8];
#endif


#if PCBX9D
#ifdef WIDE_SCREEN
#ifndef REV9E
const uint8_t arrows[] = {
10,64,80,
0xFF,0xF7,0xF3,0xF9,0x00,0x00,0xF9,0xF3,0xF7,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFC,0xFC,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xCF,0x87,0x03,0x49,0xCF,0xCF,0xCF,0xCF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x3F,0x3F,0xFF,0xFF,0xFF,0xFF,
0xFF,0xEF,0xCF,0x9F,0x00,0x00,0x9F,0xCF,0xEF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

0xFF,0xFF,0x3F,0x7F,0xFF,0x3F,0x8F,0xE3,0xF8,0xFF,
0xFF,0xFF,0xFF,0xFE,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xCF,0xCF,0xCF,0xCF,0x49,0x03,0x87,0xCF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xFF,0xFF,0xFF,
0xFF,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0xFF,
0xFF,0x9E,0x8C,0xC0,0xE1,0xE1,0xC0,0x8C,0x9E,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

} ;
#endif
#endif
#endif

#if PCBX9D
uint8_t ImageDisplay = 0 ;
uint8_t ImageX ;
uint8_t ImageY ;
#endif

void lcd_clear()
{
  memset( DisplayBuf, 0, sizeof( DisplayBuf) ) ;
#if PCBX9D
#if defined(REVPLUS) || defined(REVNORM)
 #ifndef REV9E
	lcd_img( 212-X9D_OFFSET, 0, arrows, 0, 0 ) ;
	lcd_img( 212-X9D_OFFSET-10, 0, arrows, 1, 0 ) ;
 #endif // nREV9E
#endif	// normal/plus


#endif // PCBX9D
}

// LCD i/o pins
// LCD_RES     PC27
// LCD_CS1     PC26
// LCD_E       PC12
// LCD_RnW     PC13
// LCD_A0      PC15
// LCD_D0      PC0
// LCD_D1      PC7
// LCD_D2      PC6
// LCD_D3      PC5
// LCD_D4      PC4
// LCD_D5      PC3
// LCD_D6      PC2
// LCD_D7      PC1

#define LCD_DATA	0x000000FFL
#ifndef REVA
#define LCD_A0    0x00000080L
#else
#define LCD_A0    0x00008000L
#endif
#define LCD_RnW   0x00002000L		// bit 13
#define LCD_E     0x00001000L
#define LCD_CS1   0x04000000L		// bit 26
#define LCD_RES   0x08000000L


uint8_t LcdLock ;
uint16_t LcdInputs ;
