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
//#include <U8g2lib.h>

//#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#include <ADC.h>
#include <util/delay.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
//#include "rgb_lcd.h"

//rgb_lcd grove_lcd;
const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

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

// full buffer
//U8G2_SSD1327_WS_128X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);


// End of constructor list

uint16_t loopcounter0 = 0;
uint16_t loopcounter1 = 0;

#define LOOPLED 23
uint8_t h = 60;
uint8_t wertcounter = 0;
uint8_t wert = 0;

// LCD
//rgb_lcd lcd;



// SPI
#define BUFSIZE 8
volatile uint16_t ADC_Wert0 = 0;
volatile uint16_t ADC_Wert1 = 0;

volatile unsigned char incoming[BUFSIZE];
volatile short int received=0;
volatile uint8_t spistatus = 0;
#define RECEIVED	0


uint16_t spicounter=0;
uint8_t spicheck = 0; // laufvariable
volatile uint8_t transferindex = 0; // pos von data auf SPI
volatile uint8_t out_data[BUFSIZE];
volatile uint8_t in_data[BUFSIZE];

uint8_t paketnummer = 0;

// Define SOFT-SPI pin assignments
const int MOSI_PIN = 11; // MOSI
const int MISO_PIN = 12; // MISO
const int SCK_PIN = 13;  // SCK
const int SS_PIN = 10;   // SS
#define SOFT_BUFSIZE  8


ADC *adc = new ADC(); // adc object

#define SOFT 0

#define CLOCKSPEED 4000000

void setADC0(uint8_t avrg, uint8_t res)
{
  adc->adc0->setAveraging(avrg); // set number of averages
  adc->adc0->setResolution(res); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  adc->adc0->recalibrate();
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);

}

uint8_t softspitransfer(uint8_t data) 
{
  uint8_t receivedData = 0;
  uint8_t spidelay = 2;
  // Pull SS low to select the slave
  digitalWrite(SS_PIN, LOW);
  _delay_us(spidelay);
  // Send and receive 8 bits
  for (int i = SOFT_BUFSIZE; i >= 0; i--) {
    // Set MOSI according to the data bit
    _delay_us(2*spidelay);
    digitalWrite(MOSI_PIN, (data & (1 << i)) ? HIGH : LOW);
    _delay_us(spidelay);
    // Pulse the clock
    digitalWrite(SCK_PIN, HIGH);
    _delay_us(spidelay);
    // Read MISO
    /*
    if (digitalRead(MISO_PIN)) {
      receivedData |= (1 << i);
    }
    */
    // Pulse the clock
    digitalWrite(SCK_PIN, LOW);
    _delay_us(1);
  }

  // Pull SS high to release the slave
  digitalWrite(SS_PIN, HIGH);

  return receivedData;
}

void SOFT_SPI_out2data(uint8_t data0,uint8_t data1)
{
      digitalWriteFast(SS,LOW);
      softspitransfer(data0);
      digitalWriteFast(SS,HIGH);
      _delay_us(10);
      //_delay_ms(40);
      
      digitalWriteFast(SS,LOW);
      softspitransfer(data1);
      digitalWriteFast(SS,HIGH);
      

}


void SPI_out2data(uint8_t data0,uint8_t data1)
{
   SPI.beginTransaction(SPISettings(CLOCKSPEED, MSBFIRST, SPI_MODE0));
      digitalWriteFast(SS,LOW);
      SPI.transfer(data0);
      digitalWriteFast(SS,HIGH);
      _delay_us(6);
      digitalWriteFast(SS,LOW);
      SPI.transfer(data1);
      digitalWriteFast(SS,HIGH);
      SPI.endTransaction();

}

/*
void SPI_out2data4(uint8_t code,uint8_t data)
{
   SPI.beginTransaction(SPISettings(CLOCKSPEED, MSBFIRST, SPI_MODE0));
      digitalWriteFast(SS,LOW);
      SPI.transfer(data0);
      digitalWriteFast(SS,HIGH);
      _delay_us(6);
      //_delay_ms(40);
      digitalWriteFast(SS,LOW);
      SPI.transfer(data1);
      digitalWriteFast(SS,HIGH);
      SPI.endTransaction();

}
*/
void setup(void) 
{
  pinMode(23,OUTPUT);
  pinMode(SS,OUTPUT);

  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

// Initialize Soft-SPI pins
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(SS_PIN, OUTPUT);
   // Set initial states
  digitalWrite(SS_PIN, HIGH);
  digitalWrite(SCK_PIN, LOW);
  
  if (SOFT == 0)
  {
    SPI.begin(); 
  }
  // codes fuer 1. byte
  out_data[0] = 0xFF; // sync
  out_data[2] = 101;
  out_data[4] = 102;
  out_data[6] = 103;

  // ADC
  /*
  adc->adc0->setAveraging(2); // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  adc->adc0->recalibrate();
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  */

  setADC0(8,8);



  /*
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 20);
    u8g2.print(F("CNC"));
    u8g2.setCursor(50, 10);
    u8g2.print(F("Hotwire"));
    u8g2.setCursor(4, 40);
    u8g2 .print(u8x8_u8toa(loopcounter1, 3));
    u8g2.drawFrame(60,50,12,h);
    u8g2.drawBox(61,50+h-wert,10,wert);
  } while ( u8g2.nextPage() );
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.firstPage();
  */

  // LCD groove
   /*
  // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);

    lcd.setRGB(colorR, colorG, colorB);

    // Print a message to the LCD.
    lcd.print("hello, world!");
    _delay_ms(1000);
    lcd.clear();
*/

lcd.init();
lcd.backlight();
lcd.setCursor(0, 0);
lcd.print("setup");
_delay_ms(1000);
lcd.clear();
}

void loop(void) 
{
  
  loopcounter0++;
  if(loopcounter0 == 0xFF)
  {
    loopcounter0 = 0;
     loopcounter1++;
    if (loopcounter1 == 0xFF)
    {
      loopcounter1 = 0;
      digitalWrite(LOOPLED,!(digitalRead(LOOPLED)));
      wertcounter++;

      ADC_Wert0 = analogRead(A0);
      ADC_Wert1 = analogRead(A1);
      //ADC_Wert1 = analogRead(A1);
      uint8_t adcdiff = (ADC_Wert0 > ADC_Wert1) ? (ADC_Wert0 - ADC_Wert1) : (ADC_Wert1 - ADC_Wert0);
      //transferindex &= 0x07;

      //lcd.setCursor(0,0);
      //lcd.print(ADC_Wert0);
      //lcd.setCursor(6,0);
      //lcd.print((ADC_Wert1 & 0xFF00)>>8);
      //lcd.print((ADC_Wert1));
      out_data[1] = transferindex; // data sync  
      out_data[3] = ADC_Wert0;      // data 0
      out_data[5] = ADC_Wert1;      // data 1
      out_data[7] = adcdiff;        // data 2

      char buf[4];
      lcd.setCursor(0, 0);
      lcd.print(out_data[0]);
      lcd.print(": ");
      sprintf(buf,"%3d",out_data[1]);
      lcd.print(buf);
      lcd.setCursor(0, 1);
      lcd.print(out_data[2]);
      lcd.print(": ");
      sprintf(buf,"%3d",out_data[3]);
      lcd.print(buf);
      
      lcd.setCursor(0, 2);
      lcd.print(out_data[4]);
      lcd.print(": ");
      sprintf(buf,"%3d",out_data[5]);
      lcd.print(buf);
      lcd.setCursor(0, 3);
      lcd.print(out_data[6]);
      lcd.print(": ");
      sprintf(buf,"%3d",out_data[7]);
      lcd.print(buf);
      


      paketnummer = transferindex%4; // pos im paket 01 23 45 67

      



      if (SOFT)
      {
         SOFT_SPI_out2data(out_data[2*paketnummer],out_data[2*paketnummer+1]);
      }
      else
      {
          SPI_out2data(out_data[2*paketnummer],out_data[2*paketnummer+1]);
      }
      //
     
     

            //out_data[6]++;
      transferindex++;
      
      /*   
      //     u8g2.firstPage();
      //    digitalWriteFast(23,!(digitalRead(23)));
      
      
      
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
