#include <Arduino.h>

/* Pin definitions:
Most of these pins can be moved to any digital or analog pin.
DN(MOSI)and SCLK should be left where they are (SPI pins). The
LED (backlight) pin should remain on a PWM-capable pin. */
const int scePin =  10;  // SCE - Chip select, pin 3 on OLED.
const int rstPin =  8;   // RST - Reset, pin 4 on OLED.
const int dcPin =    7;  // DC - Data/Command, pin 5 on OLED.
const int sdinPin = 11;  // DN(MOSI) - Serial data, pin 6 on OLED.
const int sclkPin = 13;  // SCLK - Serial clock, pin 7 on OLED.

/* PCD8544-specific defines: */
#define OLED_COMMAND  0
#define OLED_DATA     1

/* 84x48 OLED Defines: */
#define OLED_WIDTH   128   // Note: x-coordinates go wide
#define OLED_HEIGHT  64    // Note: y-coordinates go high
#define WHITE        0     // For drawing pixels. A 0 draws white.
#define BLACK        1     // A 1 draws black.

//
// Font table:
// This table contains the hex values that represent pixels for a
// font that is 5 pixels wide and 8 pixels high. Each byte in a row
// represents one, 8-pixel, vertical column of a character. 5 bytes
// per character. 
//

static const byte ASCII[][5] PROGMEM = {
  // First 32 characters (0x00-0x19) are ignored. These are
  // non-displayable, control characters.
   {0x00, 0x00, 0x00, 0x00, 0x00} // 0x20
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 0x21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 0x22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 0x23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 0x24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 0x25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 0x26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 0x27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 0x28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 0x29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 0x2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 0x2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 0x2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 0x2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 0x2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 0x2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 0x30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 0x31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 0x32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 0x33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 0x34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 0x35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 0x36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 0x37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 0x38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 0x39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 0x3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 0x3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 0x3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 0x3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 0x3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 0x3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 0x40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 0x41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 0x42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 0x43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 0x44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 0x45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 0x46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 0x47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 0x48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 0x49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 0x4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 0x4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 0x4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 0x4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 0x4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 0x4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 0x50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 0x51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 0x52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 0x53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 0x54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 0x55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 0x56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 0x57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 0x58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 0x59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 0x5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 0x5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 0x5c \ (keep this to escape the backslash)
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 0x5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 0x5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 0x5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 0x60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 0x61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 0x62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 0x63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 0x64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 0x65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 0x66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 0x67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 0x68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 0x69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 0x6a j
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 0x6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 0x6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 0x6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 0x6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 0x6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 0x70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 0x71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 0x72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 0x73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 0x74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 0x75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 0x76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 0x77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 0x78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 0x79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 0x7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 0x7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 0x7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 0x7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 0x7e ~
  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 0x7f DEL
};

//
// This memory is where the image is drawn before it is uploaded to
// OLED. It is arranged such that the first OLED_WIDTH vector of 
// bytes represent the first 8 rows where each bit is black or 
// white. The actual data written to the OLED is not the same and
// must be mapped so that each row is set of bytes where each byte
// contains two nibbles one nibble per pixel grey scale. We don't 
// support the grey scales on a per pixel basis because we don't
// have enough memory.
// 
byte displayMap[OLED_WIDTH * OLED_HEIGHT / 8] = { 0x0 };

//
// There are two memory banks in the OLED, data/RAM and commands.
// This function sets the DC pin high or low depending, and then
// sends the data byte
//
inline void OLEDWrite(byte data_or_command, byte data)
{
     //Tell the OLED that we are writing either to data or a command
     digitalWrite(dcPin, data_or_command);
     //Send the data
     digitalWrite(scePin, LOW);
     SPI.transfer(data); 
     digitalWrite(scePin, HIGH);
}

//
// This function sets a pixel on displayMap to your preferred
// color. 1=Black, 0= white.
//
inline void oledSetPixel(int x, int y, boolean bw)
{
    // First, double check that the coordinate is in range.
    if ((x >= 0) && (x < OLED_WIDTH) && (y >= 0) && (y < OLED_HEIGHT)) {  
        byte shift = y % 8;
        if (bw) 
           displayMap[x + (y/8)*OLED_WIDTH] |= 1<<shift;
        else   // If white clear the bit.
           displayMap[x + (y/8)*OLED_WIDTH] &= ~(1<<shift);
    }
}

//
// setLine draws a line from x0,y0 to x1,y1 with the set color.
// This function was grabbed from the SparkFun ColorOLEDShield
// library.
//
inline void oledSetLine(int x0, int y0, int x1, int y1, boolean bw)
{
    int dy = y1 - y0; // Difference between y0 and y1
    int dx = x1 - x0; // Difference between x0 and x1
    int stepx, stepy;
    //
    if (dy < 0) {
        dy = -dy;
        stepy = -1;
    } else 
        stepy = 1;
    //
    if (dx < 0) {
       dx = -dx;
       stepx = -1;
    } else 
       stepx = 1;
    //
    dy <<= 1;                             // dy is now 2*dy
    dx <<= 1;                             // dx is now 2*dx
    oledSetPixel(x0, y0, bw);             // Draw the first pixel.

    if (dx > dy) {
        int fraction = dy - (dx >> 1);
        while (x0 != x1) {
            if (fraction >= 0) {
                y0 += stepy;
                fraction -= dx;
             }
             x0 += stepx;
             fraction += dy;
              oledSetPixel(x0, y0, bw);
        }
     } else {
        int fraction = dx - (dy >> 1);
        while (y0 != y1) {
            if (fraction >= 0) {
                x0 += stepx;
                fraction -= dy;
            }
            y0 += stepy;
            fraction += dx;
            oledSetPixel(x0, y0, bw);
        }
     }
}

//
// This function will draw a char (defined in the ASCII table
// near the beginning of this sketch) at a defined x and y).
// The color can be either black (1) or white (0).
//
inline void oledSetChar(char character, int x, int y, boolean bw)
{    
     byte column; // temp byte to store character's column bitmap
     for(int i=0; i<5; i++)  {                                      // 5 columns (x) per character
         column = pgm_read_byte(&ASCII[character - 0x20][i]);
         for(int j=0; j<8; j++) {                                   // 8 rows (y) per character
             if (column & (0x01 << j)) // test bits to set pixels
                oledSetPixel(x+i, y+j, bw);
            else
                oledSetPixel(x+i, y+j, !bw);
         }
     }
}

// 
// Draw a string of characters. at location x,y in the given color.
//
inline void oledSetStr(char * dString, int x, int y, boolean bw)
{    
     while (*dString != 0x00)  {           // loop until null terminator
        oledSetChar(*dString++, x, y, bw);
        x +=5;
        for(int i=y; i<y+8; i++) {
            oledSetPixel(x, i, !bw);
        }
        x++;
        if (x > (OLED_WIDTH - 5))  {        // Enables wrap around
            x = 0;
            y += 8;
        }
     }
}


// 
// Clear the display map to each black or white. Won't see a result until you call oledUpdateDisplay.
//
inline void oledClearDisplay(boolean bw)
{
     for (int i=0; i<(OLED_WIDTH * OLED_HEIGHT / 8); i++) {  
          displayMap[i] = bw ? 0xFF : 0;
     }
}

// This will actually draw on the display, whatever is currently in the displayMap array. We have two challanges
// Here the first is that the bit map is arranged with pixels vertically per byte, so the first 8 rows are in the
// first row of byptes, while the OLED wants them arranged two pixels per byte (grey scale). Since we don't have
// enough memory to keep the entire display we keep only 1/2 the display and then duplicate lines of pixels. So
// the OED is 128 by 128 grey but we work with a 128 x 64 black and white display and then map it to the OLED.
// The results are a bit ugly but it allows the use of a dirt cheap Arduino with very little memory.
//
inline void oledUpdateDisplay()
{ 
     const  byte byteMap[2] = { 0xf, 0x0 };                           // white is 4 bit nibble 0xf, black is 0x0
     for(int y=0; y < OLED_HEIGHT; y++) {
         byte *mapp = &displayMap[(y/8)*OLED_WIDTH];
         byte duplicate[OLED_WIDTH/2];
         byte *dp = &duplicate[0];
         for(int x = 0; x < OLED_WIDTH-1; x+=2) {                  
             int  shift    = y % 8;
             byte pixone   = *mapp++;                       
             byte pixtwo   = *mapp++;                       
             byte colone   = byteMap[(pixone>>shift) & 1];
	           byte coltwo   = byteMap[(pixtwo>>shift) & 1];
             *dp = (colone<<4) | coltwo;
             OLEDWrite(OLED_DATA, *dp++);
         }
         dp = &duplicate[0];
         for(int x = 0; x < OLED_WIDTH-1; x+= 2) {
             OLEDWrite(OLED_DATA, *dp++);
         }
     }
}

//
// Send a reset command to the OLED and then reconfigure all the different register settings.
//
inline void oledReset()
{
  digitalWrite(rstPin,HIGH);
  delay(100);
  digitalWrite(rstPin,LOW);
  delay(100);
  digitalWrite(rstPin,HIGH);
  delay(100);
  //
  OLEDWrite(OLED_COMMAND,0xae);    //--turn off oled panel
  //
  OLEDWrite(OLED_COMMAND,0x15);    //set column address
  OLEDWrite(OLED_COMMAND,0x00);    //start column   0
  OLEDWrite(OLED_COMMAND,0x7f);    //end column   127
  //
  OLEDWrite(OLED_COMMAND,0x75);    //set row address
  OLEDWrite(OLED_COMMAND,0x00);    //start row   0
  OLEDWrite(OLED_COMMAND,0x7f);    //end row   127
  //
  OLEDWrite(OLED_COMMAND,0x81);    //set contrast control
  OLEDWrite(OLED_COMMAND,0x80);
  //
  OLEDWrite(OLED_COMMAND,0xa0);    //gment remap
  OLEDWrite(OLED_COMMAND,0x51);    //51
  //
  OLEDWrite(OLED_COMMAND,0xa1);    //start line
  OLEDWrite(OLED_COMMAND,0x00);
  //
  OLEDWrite(OLED_COMMAND,0xa2);    //display offset
  OLEDWrite(OLED_COMMAND,0x00);
  //
  OLEDWrite(OLED_COMMAND,0xa4);    //rmal display
  OLEDWrite(OLED_COMMAND,0xa8);    //set multiplex ratio
  OLEDWrite(OLED_COMMAND,0x7f);
  //
  OLEDWrite(OLED_COMMAND,0xb1);    //set phase leghth
  OLEDWrite(OLED_COMMAND,0xf1);
  //
  OLEDWrite(OLED_COMMAND,0xb3);    //set dclk
  OLEDWrite(OLED_COMMAND,0x00);    //80Hz:0xc1 90Hz:0xe1   100Hz:0x00   110Hz:0x30 120Hz:0x50   130Hz:0x70     01
  //
  OLEDWrite(OLED_COMMAND,0xab);    //
  OLEDWrite(OLED_COMMAND,0x01);    //
  //
  OLEDWrite(OLED_COMMAND,0xb6);    //set phase leghth
  OLEDWrite(OLED_COMMAND,0x0f);
  //
  OLEDWrite(OLED_COMMAND,0xbe);
  OLEDWrite(OLED_COMMAND,0x0f);

  OLEDWrite(OLED_COMMAND,0xbc);
  OLEDWrite(OLED_COMMAND,0x08);

  OLEDWrite(OLED_COMMAND,0xd5);
  OLEDWrite(OLED_COMMAND,0x62);

  OLEDWrite(OLED_COMMAND,0xfd);
  OLEDWrite(OLED_COMMAND,0x12);
  delay(200);
  OLEDWrite(OLED_COMMAND,0xAF);    // and turn it on
}

//
// Initialize the OLED display. Start by setting up the control PINS etc., then reset it and then go through
// a set of basic commands.
//
inline void oledBegin(void)
{
  //Configure control pins
  pinMode(scePin, OUTPUT);
  pinMode(rstPin, OUTPUT);
  pinMode(dcPin, OUTPUT);
  pinMode(sdinPin, OUTPUT);
  pinMode(sclkPin, OUTPUT);

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  oledReset();
  oledClearDisplay(BLACK);
  oledUpdateDisplay();
}
