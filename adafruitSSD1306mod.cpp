/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

#include <avr/pgmspace.h>
#ifndef __SAM3X8E__
 #include <util/delay.h>
#endif
#include <stdlib.h>

//#include <Wire.h>

//#include "Adafruit_GFX.h"
#include "adafruitSSD1306mod.h"
#include "font8x8cw90.c"
static uint8_t charbuffer [128] = 
{
	(uint8_t)'B', (uint8_t)'O', (uint8_t)'K', (uint8_t)'O', (uint8_t)'N', (uint8_t)'T', (uint8_t)'E',(uint8_t)'P',(uint8_t)' ',(uint8_t)'S',(uint8_t)'P',(uint8_t)'C',(uint8_t)'-',(uint8_t)'1',(uint8_t)' ',(uint8_t)' ',
    (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',
    (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',
    (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',
	(uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',
    (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',
    (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',(uint8_t)' ',
    (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ', (uint8_t)' ',(uint8_t)' ',(uint8_t)'(',(uint8_t)'c',(uint8_t)')',(uint8_t)' ',(uint8_t)'2',(uint8_t)'0',(uint8_t)'1',(uint8_t)'5',
    
};

Adafruit_SSD1306::Adafruit_SSD1306(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS)  {
  cs = CS;
  rst = RST;
  dc = DC;
  sclk = SCLK;
  sid = SID;
  hwSPI = false;
}

// constructor for hardware SPI - we indicate DataCommand, ChipSelect, Reset 
Adafruit_SSD1306::Adafruit_SSD1306(int8_t DC, int8_t RST, int8_t CS)  {
  dc = DC;
  rst = RST;
  cs = CS;
  hwSPI = true;
}

// initializer for I2C - we only indicate the reset pin!
Adafruit_SSD1306::Adafruit_SSD1306(int8_t reset)  {
  sclk = dc = cs = sid = -1;
  rst = reset;
}
  

void Adafruit_SSD1306::begin(uint8_t vccstate, uint8_t i2caddr, bool reset) {
  _vccstate = vccstate;
  _i2caddr = i2caddr;

  // set pin directions
  if (sid != -1){
    pinMode(dc, OUTPUT);
    pinMode(cs, OUTPUT);
    csport      = portOutputRegister(digitalPinToPort(cs));
    cspinmask   = digitalPinToBitMask(cs);
    dcport      = portOutputRegister(digitalPinToPort(dc));
    dcpinmask   = digitalPinToBitMask(dc);
    if (!hwSPI){
      // set pins for software-SPI
      pinMode(sid, OUTPUT);
      pinMode(sclk, OUTPUT);
      clkport     = portOutputRegister(digitalPinToPort(sclk));
      clkpinmask  = digitalPinToBitMask(sclk);
      mosiport    = portOutputRegister(digitalPinToPort(sid));
      mosipinmask = digitalPinToBitMask(sid);
      }
    if (hwSPI){
      SPI.begin ();
#ifdef __SAM3X8E__
      SPI.setClockDivider (9); // 9.3 MHz
#else
      SPI.setClockDivider (SPI_CLOCK_DIV2); // 8 MHz
#endif
      }
    }
  else
  {
    // I2C Init
    //Wire.begin();
#ifdef __SAM3X8E__
    // Force 400 KHz I2C, rawr! (Uses pins 20, 21 for SDA, SCL)
    TWI1->TWI_CWGR = 0;
    TWI1->TWI_CWGR = ((VARIANT_MCK / (2 * 400000)) - 4) * 0x101;
#endif
  }

  if (reset) {
    // Setup reset pin direction (used by both SPI and I2C)  
    pinMode(rst, OUTPUT);
    digitalWrite(rst, HIGH);
    // VDD (3.3V) goes high at start, lets just chill for a ms
    delay(1);
    // bring reset low
    digitalWrite(rst, LOW);
    // wait 10ms
    delay(10);
    // bring out of reset
    digitalWrite(rst, HIGH);
    // turn on VCC (9V?)
  }

   #if defined SSD1306_128_32
    // Init sequence for 128x32 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x1F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x10); }
    else 
      { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    ssd1306_command(0x8F);
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x22); }
    else 
      { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
  #endif

  #if defined SSD1306_128_64
    // Init sequence for 128x64 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x3F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x10); }
    else 
      { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x9F); }
    else 
      { ssd1306_command(0xCF); }
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x22); }
    else 
      { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
  #endif

  #if defined SSD1306_96_16
    // Init sequence for 96x16 OLED module
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80
    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(0x0F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x00);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x10); }
    else 
      { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x2);	//ada x12
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x10); }
    else 
      { ssd1306_command(0xAF); }
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC) 
      { ssd1306_command(0x22); }
    else 
      { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
  #endif

  ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}


void Adafruit_SSD1306::invertDisplay(uint8_t i) {
  if (i) {
    ssd1306_command(SSD1306_INVERTDISPLAY);
  } else {
    ssd1306_command(SSD1306_NORMALDISPLAY);
  }
}

void Adafruit_SSD1306::ssd1306_command(uint8_t c) { 
  if (sid != -1)
  {
    // SPI
    //digitalWrite(cs, HIGH);
    *csport |= cspinmask;
    //digitalWrite(dc, LOW);
    *dcport &= ~dcpinmask;
    //digitalWrite(cs, LOW);
    *csport &= ~cspinmask;
    fastSPIwrite(c);
    //digitalWrite(cs, HIGH);
    *csport |= cspinmask;
  }
  else
  {
    // I2C
    uint8_t control = 0x00;   // Co = 0, D/C = 0
    //Wire.beginTransmission(_i2caddr);
    //WIRE_WRITE(control);
    //WIRE_WRITE(c);
    //Wire.endTransmission();
  }
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void Adafruit_SSD1306::startscrollright(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X00);
  ssd1306_command(0XFF);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void Adafruit_SSD1306::startscrollleft(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X00);
  ssd1306_command(0XFF);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void Adafruit_SSD1306::startscrolldiagright(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);  
  ssd1306_command(0X00);
  ssd1306_command(SSD1306_LCDHEIGHT);
  ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X01);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void Adafruit_SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);  
  ssd1306_command(0X00);
  ssd1306_command(SSD1306_LCDHEIGHT);
  ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X01);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void Adafruit_SSD1306::stopscroll(void){
  ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void Adafruit_SSD1306::dim(boolean dim) {
  uint8_t contrast;

  if (dim) {
    contrast = 0; // Dimmed display
  } else {
    if (_vccstate == SSD1306_EXTERNALVCC) {
      contrast = 0x9F;
    } else {
      contrast = 0xCF;
    }
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  ssd1306_command(SSD1306_SETCONTRAST);
  ssd1306_command(contrast);
}

void Adafruit_SSD1306::ssd1306_data(uint8_t c) {
  if (sid != -1)
  {
    // SPI
    //digitalWrite(cs, HIGH);
    *csport |= cspinmask;
    //digitalWrite(dc, HIGH);
    *dcport |= dcpinmask;
    //digitalWrite(cs, LOW);
    *csport &= ~cspinmask;
    fastSPIwrite(c);
    //digitalWrite(cs, HIGH);
    *csport |= cspinmask;
  }
  else
  {
    // I2C
    uint8_t control = 0x40;   // Co = 0, D/C = 1
    //Wire.beginTransmission(_i2caddr);
    //WIRE_WRITE(control);
    //WIRE_WRITE(c);
    //Wire.endTransmission();
  }
}

void Adafruit_SSD1306::display(void) {
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  #if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
  #endif

  if (sid != -1)
  {
    // SPI
    *csport |= cspinmask;
    *dcport |= dcpinmask;
    *csport &= ~cspinmask;

    //for (uint16_t i=0; i<((SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT)>>3); i++) {
    //  fastSPIwrite(buffer[i]);
      //ssd1306_data(buffer[i]);
    //}
	

	//b = 7;
	uint8_t b;
	unsigned char d;
	for(uint16_t c = 0; c<128; c++)
	{
	
		
		if( charbuffer[c] & B10000000 )
		{
			
			for(uint8_t i = 0; i< 8;i++)
			{
					
					b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+i);
					d = b ^ 0xff;
					fastSPIwrite(d);
			}

		}
		else
		{
		
			for(uint8_t i = 0; i< 8;i++)
			{
			
					b = pgm_read_byte(fontbmp+charbuffer[c]*8+i);
					fastSPIwrite((unsigned char)b);
			}
		}
	}

	/*
	fastSPIwrite(255);
	
	for(uint16_t i =0; i<((SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT)>>3)-2; i++)
	{
		fastSPIwrite(0);
	
	}
	fastSPIwrite(255);
	*/
    *csport |= cspinmask;
  }
  else
  {
    // save I2C bitrate
#ifndef __SAM3X8E__
    uint8_t twbrbackup = TWBR;
    TWBR = 12; // upgrade to 400KHz!
#endif

    //Serial.println(TWBR, DEC);
    //Serial.println(TWSR & 0x3, DEC);

    // I2C
    for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
      // send a bunch of data in one xmission
     // Wire.beginTransmission(_i2caddr);
      //WIRE_WRITE(0x40);
      for (uint8_t x=0; x<16; x++) {
  //WIRE_WRITE(buffer[i]);
  i++;
      }
      i--;
      //Wire.endTransmission();
    }
#ifndef __SAM3X8E__
    TWBR = twbrbackup;
#endif
  }
}

void Adafruit_SSD1306::begin_display()
{
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  #if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
  #endif

  if (sid != -1)
  {
    // SPI
    *csport |= cspinmask;
    *dcport |= dcpinmask;
    *csport &= ~cspinmask;
	}
}

void Adafruit_SSD1306::end_display()
{
	*csport |= cspinmask;
}

void Adafruit_SSD1306::display_row(int row)
{
	uint8_t b;
	unsigned char d;
	for(uint16_t c = row<<4; c<(row<<4)+16; c++)
	{
	
		
		if( charbuffer[c] & B10000000 )
		{
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+1);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+2);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+3);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+4);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+5);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+6);
			d = b ^ 0xff;
			fastSPIwrite(d);
			b =   pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+7);
			d = b ^ 0xff;
			fastSPIwrite(d);
		}
		else
		{
			b = pgm_read_byte(fontbmp+charbuffer[c]*8);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+1);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+2);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+3);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+4);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+5);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+6);
			fastSPIwrite((unsigned char)b);
			b = pgm_read_byte(fontbmp+charbuffer[c]*8+7);
			fastSPIwrite((unsigned char)b);


		}
	}

}

void Adafruit_SSD1306::display_row(int row, int fld)
{
	uint8_t b;
	unsigned char d;
	for(uint16_t c = row<<4; c<(row<<4)+16; c++)
	{
		if( charbuffer[c] & B10000000 )
		{
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+1));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+2));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+3));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+4));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+5));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+6));
			fastSPIwrite(0xff ^ pgm_read_byte(fontbmp+(charbuffer[c] & 127)*8+7));
		}
		else
		{
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+1));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+2));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+3));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+4));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+5));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+6));
			fastSPIwrite(pgm_read_byte(fontbmp+charbuffer[c]*8+7));
		}
	}

}

// clear everything
void Adafruit_SSD1306::clearDisplay(void) {
  //memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}


inline void Adafruit_SSD1306::fastSPIwrite(uint8_t d) {
  
  if(hwSPI) {
    (void)SPI.transfer(d);
  } else {
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      *clkport &= ~clkpinmask;
      if(d & bit) *mosiport |=  mosipinmask;
      else        *mosiport &= ~mosipinmask;
      *clkport |=  clkpinmask;
    }
  }
  //*csport |= cspinmask;
}

