// IMPORTANT: LIBRARY MUST BE SPECIFICALLY CONFIGURED FOR EITHER TFT SHIELD
// OR BREAKOUT BOARD USAGE.

// Lcdwiki GUI library
// MIT license

#if defined(__SAM3X8E__)
	#include <include/pio.h>
    #define PROGMEM
    #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
    #define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif

#include "pins_arduino.h"
#include "wiring_private.h"
#include "LCDWIKI_KBV.h"
#include "lcd_registers.h"
#include "lcd_mode.h"
#if (CONFIG_USE_8BIT_BUS==1)
#include "mcu_8bit_magic.h"
#elif (CONFIG_USE_8BIT_BUS==0)
#include "mcu_16bit_magic.h"
#endif

#define TFTLCD_DELAY16  0xFFFF
#define TFTLCD_DELAY8   0x7F
#define MAX_REG_NUM     24

//static uint8_t have_reset;

//#define LEFT_SHIFT(x) (1<<x) //x value 0:mipi dcs rev1
                             //        1:auto readinc
                             //        2:read bgr
                             //        3:read low and high
                             //        4:read 24 bits
                             //        5:xsa and xea 16 bit
                             //        6:read no dummy
                             //        8:invert gs
                             //        9:invert ss
                             //       10:mv axis
                             //       11:invert rgb
                             //       12:rev screen
                             //       13:filp vert	
			                 //       14:flip horiz

//The mode,width and heigth of supported LCD modules
lcd_info current_lcd_info[] = 
						 { 
							 0x9325,240,320,			
							 0x9328,240,320,
							 0x9341,240,320,
							 0x9090,320,480,
							 0x7575,240,320,
							 0x9595,240,320,
							 0x9486,320,480,
							 0x7735,128,160,
							 0x9488,320,480,
							 0x9481,320,480,
						 };

// Constructor for breakout board (configurable LCD control lines).
// Can still use this w/shield, but parameters are ignored.
// if modules is unreadable or you don't know the width and height of modules,you can use this constructor.
LCDWIKI_KBV::LCDWIKI_KBV(uint16_t model,uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset)
{	
  #ifndef USE_ADAFRUIT_SHIELD_PIN
	  // Convert pin numbers to registers and bitmasks
	  _reset	 = reset;
  #ifdef __AVR__
		csPort	   = portOutputRegister(digitalPinToPort(cs));
		cdPort	   = portOutputRegister(digitalPinToPort(cd));
		wrPort	   = portOutputRegister(digitalPinToPort(wr));
		rdPort	   = portOutputRegister(digitalPinToPort(rd));
  #endif
  #if defined(__SAM3X8E__)
		csPort	   = digitalPinToPort(cs);
		cdPort	   = digitalPinToPort(cd);
		wrPort	   = digitalPinToPort(wr);
		rdPort	   = digitalPinToPort(rd);
  #endif
	  csPinSet	 = digitalPinToBitMask(cs);
	  cdPinSet	 = digitalPinToBitMask(cd);
	  wrPinSet	 = digitalPinToBitMask(wr);
	  rdPinSet	 = digitalPinToBitMask(rd);
	  csPinUnset = ~csPinSet;
	  cdPinUnset = ~cdPinSet;
	  wrPinUnset = ~wrPinSet;
	  rdPinUnset = ~rdPinSet;
  #ifdef __AVR__
		*csPort   |=  csPinSet; // Set all control bits to HIGH (idle)
		*cdPort   |=  cdPinSet; // Signals are ACTIVE LOW
		*wrPort   |=  wrPinSet;
		*rdPort   |=  rdPinSet;
  #endif
  #if defined(__SAM3X8E__)
		csPort->PIO_SODR  |=  csPinSet; // Set all control bits to HIGH (idle)
		cdPort->PIO_SODR  |=  cdPinSet; // Signals are ACTIVE LOW
		wrPort->PIO_SODR  |=  wrPinSet;
		rdPort->PIO_SODR  |=  rdPinSet;
  #endif
	  pinMode(cs, OUTPUT);	  // Enable outputs
	  pinMode(cd, OUTPUT);
	  pinMode(wr, OUTPUT);
	  pinMode(rd, OUTPUT);
	  if(reset) 
	  {
		 digitalWrite(reset, HIGH);
		 pinMode(reset, OUTPUT);
	  }
  #endif
  #ifdef USE_ADAFRUIT_SHIELD_PIN 
  	 CS_IDLE; // Set all control bits to idle state
 	 WR_IDLE;
  	 RD_IDLE;
  	 CD_DATA;
 	 digitalWrite(5, HIGH); // Reset line
 	 pinMode(A3, OUTPUT);   // Enable outputs
 	 pinMode(A2, OUTPUT);
 	 pinMode(A1, OUTPUT);
 	 pinMode(A0, OUTPUT);
  	 pinMode( 5, OUTPUT);
 #endif
 	rotation = 0;
 	lcd_model = current_lcd_info[model].lcd_id;
	WIDTH = current_lcd_info[model].lcd_wid;
	HEIGHT = current_lcd_info[model].lcd_heg;
/*
	switch(lcd_model)
	{
		case 0x9486:
			WIDTH = 320;
			HEIGHT = 480;
            break;
		default:
			WIDTH = 0;
			HEIGHT = 0;
			break;
	}
*/
    setWriteDir();
 	width = WIDTH;
	height = HEIGHT;		
}

// Constructor for breakout board (configurable LCD control lines).
// Can still use this w/shield, but parameters are ignored.
// if modules is readable or you know the width and height of modules,you can use this constructor.
LCDWIKI_KBV::LCDWIKI_KBV(int16_t wid,int16_t heg,uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset)
{	
  #ifndef USE_ADAFRUIT_SHIELD_PIN
	  // Convert pin numbers to registers and bitmasks
	  _reset	 = reset;
  #ifdef __AVR__
		csPort	   = portOutputRegister(digitalPinToPort(cs));
		cdPort	   = portOutputRegister(digitalPinToPort(cd));
		wrPort	   = portOutputRegister(digitalPinToPort(wr));
		rdPort	   = portOutputRegister(digitalPinToPort(rd));
  #endif
  #if defined(__SAM3X8E__)
		csPort	   = digitalPinToPort(cs);
		cdPort	   = digitalPinToPort(cd);
		wrPort	   = digitalPinToPort(wr);
		rdPort	   = digitalPinToPort(rd);
  #endif
	  csPinSet	 = digitalPinToBitMask(cs);
	  cdPinSet	 = digitalPinToBitMask(cd);
	  wrPinSet	 = digitalPinToBitMask(wr);
	  rdPinSet	 = digitalPinToBitMask(rd);
	  csPinUnset = ~csPinSet;
	  cdPinUnset = ~cdPinSet;
	  wrPinUnset = ~wrPinSet;
	  rdPinUnset = ~rdPinSet;
  #ifdef __AVR__
		*csPort   |=  csPinSet; // Set all control bits to HIGH (idle)
		*cdPort   |=  cdPinSet; // Signals are ACTIVE LOW
		*wrPort   |=  wrPinSet;
		*rdPort   |=  rdPinSet;
  #endif
  #if defined(__SAM3X8E__)
		csPort->PIO_SODR  |=  csPinSet; // Set all control bits to HIGH (idle)
		cdPort->PIO_SODR  |=  cdPinSet; // Signals are ACTIVE LOW
		wrPort->PIO_SODR  |=  wrPinSet;
		rdPort->PIO_SODR  |=  rdPinSet;
  #endif
	  pinMode(cs, OUTPUT);	  // Enable outputs
	  pinMode(cd, OUTPUT);
	  pinMode(wr, OUTPUT);
	  pinMode(rd, OUTPUT);
	  if(reset) 
	  {
		 digitalWrite(reset, HIGH);
		 pinMode(reset, OUTPUT);
	  }
  #endif
  #ifdef USE_ADAFRUIT_SHIELD_PIN 
  	 CS_IDLE; // Set all control bits to idle state
 	 WR_IDLE;
  	 RD_IDLE;
  	 CD_DATA;
 	 digitalWrite(5, HIGH); // Reset line
 	 pinMode(A3, OUTPUT);   // Enable outputs
 	 pinMode(A2, OUTPUT);
 	 pinMode(A1, OUTPUT);
 	 pinMode(A0, OUTPUT);
  	 pinMode( 5, OUTPUT);
 #endif
 	rotation = 0;
 	lcd_model = 0xFFFF;
    setWriteDir();
	WIDTH = wid;
	HEIGHT = heg;
 	width = WIDTH;
	height = HEIGHT;		
}

// Initialization lcd modules
void LCDWIKI_KBV::Init_LCD(void)
{
	reset();
	if(lcd_model == 0xFFFF)
	{
		lcd_model = Read_ID(); 
	}
//	uint16_t ID = Read_ID(); 
	start(lcd_model);
//	Set_Rotation(0);
}

// Initialization common to both shield & breakout configs
void LCDWIKI_KBV::reset(void)
{
//	have_reset = 1;
//    setWriteDir();
//Set all control bits to idle state
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;

#ifdef USE_ADAFRUIT_SHIELD_PIN
  digitalWrite(5, LOW);
  delay(2);
  digitalWrite(5, HIGH);
//  delay(100);
  //digitalWrite(5, LOW);
 // delay(100);
#else
  if(_reset) 
  {
    digitalWrite(_reset, LOW);
    delay(2);
    digitalWrite(_reset, HIGH);
//	delay(100);
	//digitalWrite(_reset, LOW);
	//delay(100);
  }
#endif
  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00);
  for(uint8_t i=0; i<3; i++)
  {
  	WR_STROBE; // Three extra 0x00s
  }
  CS_IDLE;
}

void LCDWIKI_KBV::Write_Cmd(uint16_t cmd)
{
	writeCmd16(cmd);
}

void LCDWIKI_KBV::Write_Data(uint16_t data)
{
	writeData16(data);
}

void LCDWIKI_KBV::Write_Cmd_Data(uint16_t cmd, uint16_t data)
{
	writeCmdData16(cmd,data);
}

//Write a command and N datas
void LCDWIKI_KBV::Push_Command(uint16_t cmd, uint8_t *block, int8_t N)
{
  	CS_ACTIVE;
    writeCmd16(cmd);
    while (N-- > 0) 
	{
        uint8_t u8 = *block++;
        writeData8(u8); 
    }
    CS_IDLE;
}

// Sets the LCD address window 
void LCDWIKI_KBV::Set_Addr_Window(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	CS_ACTIVE;
		uint8_t x_buf[] = {x1>>8,x1&0xFF,x2>>8,x2&0xFF}; 
		uint8_t y_buf[] = {y1>>8,y1&0xFF,y2>>8,y2&0xFF}; 
	
		Push_Command(XC, x_buf, 4); //set x address
		Push_Command(YC, y_buf, 4); //set y address
	CS_IDLE;		
}

// Unlike the 932X drivers that set the address window to the full screen
// by default (using the address counter for drawPixel operations), the
// 7575 needs the address window set on all graphics operations.  In order
// to save a few register writes on each pixel drawn, the lower-right
// corner of the address window is reset after most fill operations, so
// that drawPixel only needs to change the upper left each time.
void LCDWIKI_KBV::Set_LR(void)
{
	CS_ACTIVE;
	writeCmdData8(HX8347G_COLADDREND_HI,(width -1)>>8);
	writeCmdData8(HX8347G_COLADDREND_LO,width -1);
	writeCmdData8(HX8347G_ROWADDREND_HI,(height -1)>>8);
	writeCmdData8(HX8347G_ROWADDREND_LO,height -1);
	CS_IDLE;
}

//push color table for 16bits
void LCDWIKI_KBV::Push_Any_Color(uint16_t * block, int16_t n, bool first, uint8_t flags)
{
	uint16_t color;
    uint8_t h, l;
	bool isconst = flags & 1;
//	bool isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first) 
	{  
		if(lcd_driver == ID_932X)
		{
			writeCmd8(ILI932X_START_OSC);
			
		}
		writeCmd8(CC);		
    }
    while (n-- > 0) 
	{
        if (isconst) 
		{
			color = pgm_read_word(block++);		
        } 
		else 
		{
			color = (*block++);			

		}		
        writeData16(color);
    }
    CS_IDLE;
}

//push color table for 8bits
void LCDWIKI_KBV::Push_Any_Color(uint8_t * block, int16_t n, bool first, uint8_t flags)
{
	uint16_t color;
    uint8_t h, l;
	bool isconst = flags & 1;
	bool isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first) 
	{
		if(lcd_driver == ID_932X)
		{
			writeCmd8(ILI932X_START_OSC);
			
		}
		writeCmd8(CC);		
    }
    while (n-- > 0) 
	{
        if (isconst) 
		{
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } 
		else 
		{
		    h = (*block++);
            l = (*block++);
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
        writeData16(color);
    }
    CS_IDLE;
}

//Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t LCDWIKI_KBV::Color_To_565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

//read value from lcd register 
uint16_t LCDWIKI_KBV::Read_Reg(uint16_t reg, int8_t index)
{
	uint16_t ret,high;
    uint8_t low;
//  if (!have_reset)
 // {
 //     reset();
//  }
	CS_ACTIVE;
    writeCmd16(reg);
    setReadDir();
    delay(1); 
	do 
	{ 
	//ead8(high);
	//ead8(low);
	//et = (high << 8) | lowc
		read16(ret);  //read 16bits
	}while (--index >= 0);   
//    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

//read graph RAM data
int16_t LCDWIKI_KBV::Read_GRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h)
{
	uint16_t ret, dummy;
    int16_t n = w * h;
    uint8_t r, g, b, tmp;
    Set_Addr_Window(x, y, x + w - 1, y + h - 1);
    while (n > 0) 
	{
        CS_ACTIVE;
		writeCmd16(RC);
        setReadDir();
		if(lcd_driver == ID_932X)
		{
			while(n)
			{
				for(int i =0; i< 2; i++)
				{
					read8(r);
					read8(r);
					read8(r);
					read8(g);
				}
				*block++ = (r<<8 | g);
				n--;
			}
			Set_Addr_Window(0, 0, width - 1, height - 1);
		}
		else 
		{
			read8(r);
        	while (n) 
			{
				if(R24BIT == 1)
				{
        			read8(r);
         			read8(g);
        			read8(b);
            		ret = Color_To_565(r, g, b);
				}
				else if(R24BIT == 0)
				{
					read16(ret);
				}
            	*block++ = ret;
            	n--;
        	}
        }
//        RD_IDLE;
        CS_IDLE;
        setWriteDir();
    }
	return 0;
}

//read LCD controller chip ID 
uint16_t LCDWIKI_KBV::Read_ID(void)
{
	uint16_t ret;
	if ((Read_Reg(0x04,0) == 0x00)&&(Read_Reg(0x04,1) == 0x8000))
	{
		uint8_t buf[] = {0xFF, 0x83, 0x57};
		Push_Command(HX8357D_SETC, buf, sizeof(buf));
		ret = (Read_Reg(0xD0,0) << 16) | Read_Reg(0xD0,1);
		if((ret == 0x990000) || (ret == 0x900000))
		{
			return 0x9090;
		}
	}
	ret = Read_Reg(0xD3,1); //0x9341 0x9486
	if(ret == 0x9486)
	{
		return 0x9486;
	}
	else
	{
		return Read_Reg(0, 0); //others
	}
}

//set x,y  coordinate and color to draw a pixel point 
void LCDWIKI_KBV::Draw_Pixe(int16_t x, int16_t y, uint16_t color)
{
	if((x < 0) || (y < 0) || (x > Get_Width()) || (y > Get_Height()))
	{
		return;
	}
	Set_Addr_Window(x, y, x, y);
	CS_ACTIVE;
	writeCmdData16(CC, color);
	CS_IDLE;
}

//fill area from x to x+w,y to y+h
void LCDWIKI_KBV::Fill_Rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	int16_t end;
	if (w < 0) 
	{
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
    {
        x = 0;
    }
    if (end > Get_Width())
    {
        end = Get_Width();
    }
    w = end - x;
    if (h < 0) 
	{
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
    {
        y = 0;
    }
    if (end > Get_Height())
    {
        end = Get_Height();
    }
    h = end - y;
    Set_Addr_Window(x, y, x + w - 1, y + h - 1);//set area
	CS_ACTIVE;
    if(lcd_driver == ID_932X)
	{
		writeCmd8(ILI932X_START_OSC);
			
	}
	writeCmd8(CC);	
	if (h > w) 
	{
        end = h;
        h = w;
        w = end;
    }
	while (h-- > 0) 
	{
		end = w;
		do 
		{
   			writeData16(color);//set color data
        } while (--end != 0);
	}
	if(lcd_driver == ID_932X)
	{
		Set_Addr_Window(0, 0, width - 1, height - 1);
	}
	else if(lcd_driver == ID_7575)
	{
		Set_LR();
	}
	CS_IDLE;
}

//Scroll display 
void LCDWIKI_KBV::Vert_Scroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines; 
    int16_t vsp;
    int16_t sea = top;
    if (offset <= -scrollines || offset >= scrollines)
    {
		offset = 0; //valid scroll
    }
	vsp = top + offset; // vertical start position
    if (offset < 0)
    {
        vsp += scrollines;          //keep in unsigned range
    }
    sea = top + scrollines - 1;
	if(lcd_driver == ID_932X)
	{
		writeCmdData8(SC1, (1 << 1) | 0x1);        //!NDL, VLE, REV
        writeCmdData8(SC2, vsp);        //VL#
	}
	else
	{
  		uint8_t d[6];           // for multi-byte parameters
  		d[0] = top >> 8;        //TFA
  		d[1] = top;
  		d[2] = scrollines >> 8; //VSA
  		d[3] = scrollines;
  		d[4] = bfa >> 8;        //BFA
  		d[5] = bfa;
		Push_Command(SC1, d, 6);
		d[0] = vsp >> 8;        //VSP
  		d[1] = vsp;
		Push_Command(SC2, d, 2);
		if(lcd_driver == ID_7575)
		{
			d[0] = (offset != 0) ? 0x08:0;
			Push_Command(0x01, d, 1);
		}
		else if (offset == 0) 
		{
			Push_Command(0x13, NULL, 0);
		}
	}
}

//get lcd width
int16_t LCDWIKI_KBV::Get_Width(void) const
{
	return width;
}

//get lcd height
int16_t LCDWIKI_KBV::Get_Height(void) const
{
	return height;
}

//set clockwise rotation
void LCDWIKI_KBV::Set_Rotation(uint8_t r)
{
    rotation = r & 3;           // just perform the operation ourselves on the protected variables
    width = (rotation & 1) ? HEIGHT : WIDTH;
    height = (rotation & 1) ? WIDTH : HEIGHT;
	CS_ACTIVE;
		uint8_t val;
		switch (rotation) 
		{
		   	case 0:
		     	val = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR; //0 degree 
		     	break;
		   	case 1:
		     	val = ILI9341_MADCTL_MV | ILI9341_MADCTL_ML | ILI9341_MADCTL_BGR ; //90 degree 
		     	break;
		 	case 2:
		    	val = ILI9341_MADCTL_MY |ILI9341_MADCTL_BGR; //180 degree 
		    	break;
		   	case 3:
		     	val = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR; //270 degree
		     	break;
		 }
		 writeCmdData8(MD, val); 
 	Set_Addr_Window(0, 0, width - 1, height - 1);
	Vert_Scroll(0, HEIGHT, 0);
	CS_IDLE;
}

//get current rotation
//0  :  0 degree 
//1  :  90 degree
//2  :  180 degree
//3  :  270 degree
uint8_t LCDWIKI_KBV::Get_Rotation(void) const
{
	return rotation;
}

//Anti color display 
void LCDWIKI_KBV::Invert_Display(boolean i)
{
	CS_ACTIVE;
	uint8_t val = VL^i;
	if(lcd_driver == ID_932X)
	{
		writeCmdData8(0x61, val);
	}
	else if(lcd_driver == ID_7575)
	{
		writeCmdData8(0x01, val ? 8 : 10);
	}
	else
	{
		writeCmd8(val ? 0x21 : 0x20);
	}
	CS_IDLE;
}

void LCDWIKI_KBV:: init_table8(const void *table, int16_t size)
{
	uint8_t i;
    uint8_t *p = (uint8_t *) table, dat[MAX_REG_NUM];            //R61526 has GAMMA[22] 
    while (size > 0) 
	{
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8) 
		{
            delay(len);
            len = 0;
        } 
		else 
		{
            for (i = 0; i < len; i++)
            {
                dat[i] = pgm_read_byte(p++);
            }
			Push_Command(cmd,dat,len);
        }
        size -= len + 2;
    }
}

void LCDWIKI_KBV:: init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) 
	{
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY16)
        {
            delay(d);
        }
        else 
		{
			writeCmdData16(cmd, d);                      //static function
        }
        size -= 2 * sizeof(int16_t);
    }
}

void LCDWIKI_KBV::start(uint16_t ID)
{
	reset();
	delay(200);
	switch(ID)
	{
		case 0x9486:
			lcd_driver = ID_9486;
			//WIDTH = 320,HEIGHT = 480;
			//width = WIDTH, height = HEIGHT;
			XC=ILI9341_COLADDRSET,YC=ILI9341_PAGEADDRSET,CC=ILI9341_MEMORYWRITE,RC=HX8357_RAMRD,SC1=0x33,SC2=0x37,MD=ILI9341_MADCTL,VL=0,R24BIT=0;
			static const uint8_t ILI9486_regValues[] PROGMEM = 
			{
				0xF1, 6, 0x36, 0x04, 0x00, 0x3C, 0x0F, 0x8F,
				0xF2, 9, 0x18, 0xA3, 0x12, 0x02, 0xB2, 0x12, 0xFF, 0x10, 0x00, 
				0xF8, 2, 0x21, 0x04,
				0xF9, 2, 0x00, 0x08,
				0x36, 1, 0x08, 
				0xB4, 1, 0x00,
				0xC1, 1, 0x41,
				0xC5, 4, 0x00, 0x91, 0x80, 0x00,
				0xE0, 15, 0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00,
				0xE1, 15, 0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10 ,0x03, 0x24, 0x20, 0x00,				
				0x3A, 1, 0x55,
				0x11,0,
				0x36, 1, 0x28,
				TFTLCD_DELAY8, 120,
				0x29,0
			};
			init_table8(ILI9486_regValues, sizeof(ILI9486_regValues));
			break;
		default:
			lcd_driver = ID_UNKNOWN;
			break;		
	}
	Set_Rotation(rotation); 
	Invert_Display(false);
}
