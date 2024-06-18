/*

  PrintHelloWorld.ino
  
  Use the (Arduino compatible) u8g2 function "print"  to draw a text.
  Also use the Arduino F() macro to place the text into PROGMEM area.

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include <Arduino.h>
#include <U8g2lib.h>

//#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
/*
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
*/
/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
  This is a page buffer example.    
*/

// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8G2_NULL u8g2(U8G2_R0);  // null device, a 8x8 pixel display which does nothing
//U8G2_SSD1327_WS_128X128_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
U8G2_SSD1327_WS_128X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);


// End of constructor list

uint16_t loopcounter0 = 0;
uint16_t loopcounter1 = 0;
uint8_t h = 60;
uint8_t wertcounter = 0;
uint8_t wert = 0;


void setup(void) 
{
  pinMode(23,OUTPUT);
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 20);
    u8g2.print(F("CNC"));
    u8g2.setCursor(50, 20);
    u8g2.print(F("Hotwire"));
    u8g2.setCursor(4, 40);
    u8g2 .print(u8x8_u8toa(loopcounter1, 3));
    u8g2.drawFrame(60,50,12,h);
    u8g2.drawBox(61,50+h-wert,10,wert);
  } while ( u8g2.nextPage() );
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.firstPage();
}

void loop(void) 
{
  
  loopcounter0++;
  if(loopcounter0 == 0x1FFF)
  {
    
    loopcounter0 = 0;
     loopcounter1++;
    if (loopcounter1 == 0x1F)
    {
      u8g2.firstPage();
      digitalWriteFast(23,!(digitalRead(23)));
      loopcounter1 = 0;
      wertcounter++;
      if(wertcounter%2==0)
      {
      wert = map(wertcounter,0,255,0,60);
      u8g2.drawFrame(60,50,12,h);
      u8g2.drawBox(61,50+h-wert,10,wert);
      u8g2.updateDisplayArea(7,6,2,8);
      }
      else 
      {
       u8g2.setCursor(4, 40);
      u8g2 .print(u8x8_u8toa(wertcounter, 3));
      u8g2.updateDisplayArea(0,3,4,2);
      }
   

  
    /*
 do {
    u8g2.setCursor(0, 20);
    u8g2.print(F("CNC"));
    u8g2.setCursor(50, 20);
    u8g2.print(F("Hotwire"));
    //u8g2.setCursor(0, 40);
    //u8g2 .print(u8x8_u8toa(loopcounter, 3));
    //u8g2.drawFrame(60,50,12,h);
    //u8g2.drawBox(61,50+h-wert,10,wert);
  } while ( u8g2.nextPage() );
  */
  
     
    }
   
  }
  //delay(100);
}
