
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

//COMPILED WITH FASTEST + PURE CODE

#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <SerialFlash.h>
#include "reSID.h"

const byte CS = 11;
const byte RW = 12;
const byte Α5 = 13;


  #define PORTC_PDIR GPIOC_PDIR
  #define pinCountC 8
  uint8_t portPinsC[pinCountC] = {15, 22, 23, 9, 10, 13, 11, 12}; // Port C bits D0, D1, D2, D3, D4, A5, CS, RW - These are pin numbers used on teensy
  
  //A5 signal can be found on pin 12 of the cpu.At this point is not needed.
  //All other connections go to sid chip. Please refer to 6581 sid datasheet fo details
  
  #define PORTD_PDIR GPIOD_PDIR
  #define pinCountD 8
  uint8_t portPinsD[pinCountD] = {2, 14, 7, 8, 6, 20, 21, 5}; // Port D bits A0, A1, A2, A3, A4, A5, A6, A7
  
//Using both dac pins as sound output. We get 12bit stereo sound.
  // GUItool: begin automatically generated code
AudioPlaySID             playSID;  //xy=189,110
AudioPlaySID             playSID1;  //xy=189,110
AudioOutputAnalogStereo           dacs;           //xy=366,111
AudioConnection          patchCord1(playSID, 0, dacs, 0);
AudioConnection          patchCord2(playSID1, 0, dacs, 1);
//AudioOutputAnalog          dac1;           //xy=366,111
//AudioConnection          patchCord1(playSID, 0, dac1, 0);

// GUItool: end automatically generated code

void setup() {
  Serial.begin(9600);
  AudioMemory(30);

 // Setting PORTC and PORTD pins as inputs
  for (int a=0; a<pinCountC; a++) {
    pinMode(portPinsC[a], INPUT);
  }
    for (int a=0; a<pinCountD; a++) {
    pinMode(portPinsD[a], INPUT);
  }
 // Attach an interrupt when RW goes LOW. Seems to work well.
attachInterrupt(digitalPinToInterrupt(RW), blink, LOW);
  //playSID.begin();
}

void loop() {
}


uint8_t address_lines;
uint8_t data_lines;


void blink() {

// Reading address and data lines of sid chip
     address_lines = PORTC_PDIR ;//& B00011111 ;
     data_lines = PORTD_PDIR ;

      // If both RW and CS signals are LOW 
      if ((address_lines &  B11000000) < 0x01 ){
      // Send data to reSID. At this point sound is pseudo-stereo meaning both channels play the same tones. But there are plans for dual sid.
        playSID.setreg(address_lines &  B00011111 ,data_lines);
        playSID1.setreg(address_lines &  B00011111 ,data_lines);
      }
  }

 
 

