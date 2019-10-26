/*
  YUKARI SPEED DISPLAY
  - Using Arduino Nano
  - Using the U8g2 Graphics Library !!!!!!!  The Open Iconic font collection is available from U8g2 2.22 only
  - Using a 128x32 SSD1306 OLED display
  - Using two infrared detectors connected to input pins 2 and 3

  The behaviour of the two infrared sensors is tracked using state machines, state0 and state1. Their state can be:
  - waiting: input waiting to be active (falling edge) to start measurement (counting time)
  - counting: counting the time (transition time stored in t0Count/t1Count)
  - ending: measurement is over, waiting for restTime ms without transition (transition time stored in t0End/t1End)
  - ended: cycle is over, going back to waiting when both input states are "ended"

  SSD1306 128x32 OLED connected to
  - SCL: A5
  - SDA: A4
*/

//#define DEBUG

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// U8g2 Contructor (Frame Buffer mode)
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int timeToSpeed (unsigned long timeMs) {   // time in milliseconds
float timeS = 0.0;                         // time in seconds
float speedMps = 0.0;                      // speed in meter per second
float speedKmh = 0.0;                      // speed in km per hour
float speedKmhN = 0.0;                     // speed in km per hour, N gauge
 
  timeS = ((float)timeMs / 1000.0);        // convert milliseconds to seconds
  speedMps = (0.09 / timeS);               // divide distance by time, here 9 cm = 0.09 m
  speedKmh = (speedMps * 3.6);             // convert speed to km per hour
  speedKmhN = (speedKmh * 160.0);          // convert speed to km per hour, N gauge
  return((int)speedKmhN); 
}

void printSpeed(unsigned long t0,unsigned long t1) {
  const u8g2_uint_t rightOffset = 79;
  char speedText[20];
  int speedKmh;
  
  u8g2.clearBuffer();                                 // Clear the internal display memory
  u8g2.setFont(u8g2_font_inr30_mr);                   // Choose a font
  if(t0 == t1)  {
    u8g2.drawStr(0,31,"---");                         // Write the speed (none in this case)
  }
  else {
    if (t1 > t0)
      speedKmh = timeToSpeed(t1 - t0);
    else
      speedKmh = timeToSpeed(t0 - t1);
    
    if (speedKmh > 999)  {
      u8g2.drawStr(0,31,"---");                         // Write the speed (none in this case)
    }
    else  {
      sprintf(speedText, "%3d", speedKmh);
      if (speedText[0] == '0') speedText[0] = 'O';
      if (speedText[1] == '0') speedText[1] = 'O';
      if (speedText[2] == '0') speedText[2] = 'O';
      u8g2.drawStr(0,31,speedText);                     // Write the speed    
    }
    
    u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);

    if (t1 > t0)                                        // Write the direction
      u8g2.drawStr(rightOffset,15,"\x4E\x4E\x4E");
    else
      u8g2.drawStr(rightOffset,15,"\x4D\x4D\x4D");     
  }

  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.drawStr(rightOffset+1,31,"km/h");                // Write the unit (km/h)
  u8g2.sendBuffer();                                    // Transfer internal display memory to the display
}

void setup(void) {
  int input0,input1;
  int state0,state1;
  unsigned long t0Count,t1Count,t0End,t1End,curTime;

  const unsigned long restTime = 4000;
  const int waiting =  1;
  const int counting = 2;
  const int ending =   3;
  const int ended =    4;
  
  state0 = ending;
  state1 = ending;
  t0Count = 0;
  t1Count = 0;
  t0End = 0;
  t1End = 0;
  
  pinMode(2, INPUT);        // set digital pin 2 (first sensor output) as input, active low when a train is detected
  pinMode(3, INPUT);        // set digital pin 3 (second sensor output) as input, active low when a train is detected
  delay(500);               // Delay required before initializing the display
  u8g2.begin();

  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting program");
  Serial.println("0 -> ending");
  Serial.println("1 -> ending");
  #endif

  printSpeed(0,0);               // init display with default contents
  
  while (1)  {
    input0 = digitalRead(2);     // read the first input pin
    input1 = digitalRead(3);     // read the second input pin
    curTime = millis();          // store the current time (in ms)
    
    if (state0 == waiting && input0 == 0)  {                    // Input 0 falling edge, store time and start "counting"
      #ifdef DEBUG
      Serial.println("0 -> counting");
      #endif
      state0 = counting;
      t0Count = curTime;
    }

    if (state1 == waiting && input1 == 0)  {                    // input 1 falling edge, store time and start "counting"
      #ifdef DEBUG
      Serial.println("1 -> counting");
      #endif
      state1 = counting;
      t1Count = curTime;
    }

    if (state0 == counting && curTime - t0Count > restTime)  {  // state is "counting" for too long, abort
      #ifdef DEBUG
      Serial.println("0 -> ending");
      #endif
      state0 = ending;
      state1 = ending;
    }

    if (state1 == counting && curTime - t1Count > restTime)  {  // state is "counting" for too long, abort
      #ifdef DEBUG
      Serial.println("1 -> ending");
      #endif
      state0 = ending;
      state1 = ending;
    }

    if (state0 == counting && state1 == counting)  {            // when both states are "counting", we can measure and print the speed
      #ifdef DEBUG
      Serial.println("0/1 -> printing/ending");
      #endif
      printSpeed(t0Count,t1Count);
      state0 = ending;
      state1 = ending;
    }

    if (state0 == ending && input0 == 0)  {                     // in "ending" state, store the last time input0 was 0 (active)
      t0End = curTime;
    }

    if (state1 == ending && input1 == 0)  {                     // in "ending" state, store the last time input1 was 0 (active)
      t1End = curTime;
    }

    if (state0 == ending && curTime - t0End > restTime)  {      // move from "ending" to "ended" when input has been 1 (inactive) for more than restTime
      #ifdef DEBUG
      Serial.println("0 -> ended");
      #endif
      state0 = ended;
    }

    if (state1 == ending && curTime - t1End > restTime)  {      // move from "ending" to "ended" when input has been 1 (inactive) for more than restTime
      #ifdef DEBUG
      Serial.println("1 -> ended");
      #endif
      state1 = ended;
    }

    if (state0 == ended && state1 == ended)  {                  // if both states are "ended", change them to "waiting" to restart the process
      #ifdef DEBUG
      Serial.println("0/1 -> waiting");
      #endif
      state0 = waiting;
      state1 = waiting;
    }
  }
}

void loop(void) {
}
