
/* 
 * This file is part of the 6581 SID Emulator distribution (https://github.com/kokotisp/6581-SID-teensy).
 * Copyright (c) 2018 Petros Kokotis.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

//COMPILED @216mHz overclocked + FASTEST + PURE CODE
#define FASTLED_ALLOW_INTERRUPTS 1
//#define FASTLED_INTERRUPT_RETRY_COUNT 1
//#define FASTLED_FORCE_SOFTWARE_SPI

#include <FastLED.h>
#include <Audio.h>
//#include <Wire.h>
//#include <SD.h>
//#include <SerialFlash.h>
//#include <avdweb_VirtualDelay.h>
#include "reSID.h"

const byte CS = 11;
const byte RW = 12;
//Connect 13 pin of teensy to pin 12 of commodore cpu (Via a level shifter of cource)
const byte A05 = 13;

#define DATA_PIN 34
#define CLOCK_PIN 33
#define NUM_LEDS 10

CRGB leds[NUM_LEDS];
//VirtualDelay delay1, delay2, delay3, delay4;
boolean screensaver;
uint8_t address_lines;
uint8_t data_lines;

  #define PORTC_PDIR GPIOC_PDIR
  #define pinCountC 8
  uint8_t portPinsC[pinCountC] = {15, 22, 23, 9, 10, 13, 11, 12}; // Port C bits D0, D1, D2, D3, D4, A5, CS, RW - These are pin numbers used on teensy
  
  //A5 signal can be found on pin 12 of the cpu.At this point is not needed.
  //All other connections go to sid chip. Please refer to 6581 sid datasheet fo details
  
  #define PORTD_PDIR GPIOD_PDIR
  #define pinCountD 8
  uint8_t portPinsD[pinCountD] = {2, 14, 7, 8, 6, 20, 21, 5}; // Port D bits A0, A1, A2, A3, A4, A5, A6, A7
  
//Using both dac pins as sound output. We get 12bit stereo sound.
AudioPlaySID             playSID;  //xy=189,110
AudioPlaySID             playSID1;  //xy=189,110
AudioOutputAnalogStereo  dacs;           //xy=366,111
AudioAmplifier           amp1;           //xy=601,326
//For some reason my audio channels dont have the same amplification
AudioConnection          patchCord1(playSID, amp1);
AudioConnection          patchCord2(amp1, 0, dacs, 0);
AudioAmplifier           amp2;           //xy=601,326
AudioConnection          patchCord3(playSID, amp2);
AudioConnection          patchCord4(amp2, 0, dacs, 1);
AudioAnalyzePeak         peak1;
AudioConnection          patchCord5(playSID, 0, peak1, 0);

// GUItool: end automatically generated code

void setup() {
 // Serial.begin(9600);
  AudioMemory(40);
   amp1.gain(2);
  amp2.gain(0.02);
  dacs.analogReference(INTERNAL);
 // Setting PORTC and PORTD pins as inputs
  for (int a=0; a<pinCountC; a++) {
    pinMode(portPinsC[a], INPUT);
  }
    for (int a=0; a<pinCountD; a++) {
    pinMode(portPinsD[a], INPUT);
  }
  //Fastled APA102
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR,DATA_RATE_KHZ(250)>(leds, NUM_LEDS);//,DATA_RATE_KHZ(250)
 
  

 // Attach an interrupt when RW goes LOW. Seems to work well.
attachInterrupt(digitalPinToInterrupt(RW), ReadLines, LOW);
  screensaver=1;
while (screensaver==1){
  //Larson scanner effect when not playing music
  CylonBounce(0xff, 0, 0, 2, 100, 250);
//Interrupt breaks this loop
  
}
}

elapsedMillis fps;

void loop() {
if (fps>35){
             if (peak1.available()) {
                
                int monoPeak = peak1.read() * 12.0;
               // Serial.println(monoPeak);
                    setAll(0,0,0);
                    for(int pixel = 0; pixel < monoPeak/2; pixel++){
                    leds[pixel] = CRGB::Blue;  
                    leds[pixel+1] = CRGB::Red; 
                    int tmp = NUM_LEDS - pixel -1; 
                    leds[tmp] = CRGB::Blue;  
                    leds[tmp-1] = CRGB::Red;                
                    }
                   FastLED.show();
                   fps=0;
           }
}
}



void ReadLines() {
  
// Reading address and data lines of sid chip
     address_lines = PORTC_PDIR ;//& B00011111 ;
     data_lines = PORTD_PDIR ;
    //
   // 
   
      // If both RW and CS signals are LOW 
      if ((address_lines &  B11000000) < 0x01 ){
        //Check if note is for SID at $d4200
        if ((address_lines &  B00100000) < 0x01){
      // Send data to reSID.         
        playSID.setreg(address_lines &  B00011111 ,data_lines);
        //Enable next line for pseudo stereo
        //playSID1.setreg(address_lines &  B00011111 ,data_lines);
        screensaver=0;
        //playSID1.setreg(address_lines &  B00011111 ,data_lines);
       
      }
      //For dual sid support.
      //else {
      //  playSID1.setreg(address_lines &  B00011111 ,data_lines);
      //}
  }
  //
}
 

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){



  for(int i = 0; i < NUM_LEDS-EyeSize-2; i++) {

    setAll(0,0,0);

    setPixel(i, red/10, green/10, blue/10);

    for(int j = 1; j <= EyeSize; j++) {

      setPixel(i+j, red, green, blue); 

    }

    setPixel(i+EyeSize+1, red/10, green/10, blue/10);

    showStrip();

   delay(SpeedDelay);

  }



 delay(ReturnDelay);



  for(int i = NUM_LEDS-EyeSize-2; i > 0; i--) {

    setAll(0,0,0);

    setPixel(i, red/10, green/10, blue/10);

    for(int j = 1; j <= EyeSize; j++) {

      setPixel(i+j, red, green, blue); 

    }

    setPixel(i+EyeSize+1, red/10, green/10, blue/10);

    showStrip();

   delay(SpeedDelay);

  }

  

 delay(ReturnDelay);

}

void showStrip() {

 #ifdef ADAFRUIT_NEOPIXEL_H

   // NeoPixel

   strip.show();

 #endif

 #ifndef ADAFRUIT_NEOPIXEL_H

   // FastLED

   FastLED.show();

 #endif

}



void setPixel(int Pixel, byte red, byte green, byte blue) {

 #ifdef ADAFRUIT_NEOPIXEL_H

   // NeoPixel

   strip.setPixelColor(Pixel, strip.Color(red, green, blue));

 #endif

 #ifndef ADAFRUIT_NEOPIXEL_H

   // FastLED

   leds[Pixel].r = red;

   leds[Pixel].g = green;

   leds[Pixel].b = blue;

 #endif

}



void setAll(byte red, byte green, byte blue) {

  for(int i = 0; i < NUM_LEDS; i++ ) {

    setPixel(i, red, green, blue); 

  }

  showStrip();

}
