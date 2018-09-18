/*  OctoWS2811 Rainbow.ino - Rainbow Shifting Test
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.


  Required Connections
  --------------------
    pin 2:  LED Strip #1    OctoWS2811 drives 8 LED Strips.
    pin 14: LED strip #2    All 8 are the same length.
    pin 7:  LED strip #3
    pin 8:  LED strip #4    A 100 ohm resistor should used
    pin 6:  LED strip #5    between each Teensy pin and the
    pin 20: LED strip #6    wire to the LED strip, to minimize
    pin 21: LED strip #7    high frequency ringining & noise.
    pin 5:  LED strip #8
    pin 15 & 16 - Connect together, but do not use
    pin 4 - Do not use
    pin 3 - Do not use as PWM.  Normal use is ok.
    pin 1 - Output indicating CPU usage, monitor with an oscilloscope,
            logic analyzer or even an LED (brighter = CPU busier)
*/

#include <OctoWS2811.h>

const int ledsPerStrip = 10;

DMAMEM int displayMemory[ledsPerStrip*6];
int drawingMemory[ledsPerStrip*6];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);

const uint8_t  gamma8[] = {
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,2,2,2,2,3,3,3,4,4,4,5,
5,6,6,7,7,8,9,9,10,11,11,12,13,14,15,16,
17,18,19,20,21,22,23,24,25,27,28,29,31,32,34,35,
37,39,40,42,44,46,48,50,51,54,56,58,60,62,64,67,
69,72,74,77,79,82,85,87,90,93,96,99,102,105,109,112,
115,119,122,126,129,133,137,140,144,148,152,156,160,164,169,173,
177,182,186,191,196,200,205,210,215,220,225,231,236,241,247,252,
252,247,241,236,231,225,220,215,210,205,200,196,191,186,182,177,
173,169,164,160,156,152,148,144,140,137,133,129,126,122,119,115,
112,109,105,102,99,96,93,90,87,85,82,79,77,74,72,69,
67,64,62,60,58,56,54,51,50,48,46,44,42,40,39,37,
35,34,32,31,29,28,27,25,24,23,22,21,20,19,18,17,
16,15,14,13,12,11,11,10,9,9,8,7,7,6,6,5,
5,4,4,4,3,3,3,2,2,2,2,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};

unsigned int h2rgb(unsigned int v1, unsigned int v2, unsigned int hue)
{
  if (hue < 60) return v1 * 60 + (v2 - v1) * hue;
  if (hue < 180) return v2 * 60;
  if (hue < 240) return v1 * 60 + (v2 - v1) * (240 - hue);
  return v1 * 60;
}

int makeColor(unsigned int hue, unsigned int saturation, unsigned int lightness)
{
  unsigned int red, green, blue;
  unsigned int var1, var2;

  if (hue > 359) hue = hue % 360;
  if (saturation > 100) saturation = 100;
  if (lightness > 100) lightness = 100;

  // algorithm from: http://www.easyrgb.com/index.php?X=MATH&H=19#text19
  if (saturation == 0) {
    red = green = blue = lightness * 255 / 100;
  } else {
    if (lightness < 50) {
      var2 = lightness * (100 + saturation);
    } else {
      var2 = ((lightness + saturation) * 100) - (saturation * lightness);
    }
    var1 = lightness * 200 - var2;
    red = h2rgb(var1, var2, (hue < 240) ? hue + 120 : hue - 240) * 255 / 600000;
    green = h2rgb(var1, var2, hue) * 255 / 600000;
    blue = h2rgb(var1, var2, (hue >= 120) ? hue - 120 : hue + 240) * 255 / 600000;
  }
  return (red << 16) | (green << 8) | blue;
}

// define pins
#define P_PWM4  23 
#define P_PWM2  22
#define P_PWM3  10
#define P_PWM1  9  
#define P_CKIN1 11
#define P_CKIN2 12
#define P_CKIN3 19
#define P_NOISEOUT 13
#define P_VERT_DAT 0
#define P_POT1 18
#define P_POT2 19


volatile unsigned int freq_count_1 = 0;
volatile unsigned int freq_count_2 = 0;
volatile unsigned int freq_count_3 = 0;
void isr_input_change_1() {
  ++freq_count_1;
}
void isr_input_change_2() {
  ++freq_count_2;
}
void isr_input_change_3() {
  ++freq_count_3;
}


void setRgbLeds(int color, int k) {
    for (int x=0; x < 10; x++) {
        leds.setPixel(50+ x , 0);
    }
     leds.setPixel(45+ k , color);
     leds.setPixel(50+ k , color);
     leds.setPixel(55+ k , color);
    leds.show();
}

byte bmp[8] = {
  0b00000000,
  0b00111100,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b00000000
};
byte bmp2[8] = {
  0b10011001,
  0b01100110,
  0b00011000,
  0b11000011,
  0b01011010,
  0b00100101,
  0b11111111,
  0b00000000
};
void setup() {
  

  pinMode(P_CKIN1, INPUT_PULLUP);
  pinMode(P_CKIN2, INPUT_PULLUP);
  pinMode(P_CKIN3, INPUT_PULLUP);
  pinMode(P_POT1, INPUT);
  pinMode(P_POT2, INPUT);
  pinMode(P_NOISEOUT, OUTPUT);
  pinMode(P_VERT_DAT, OUTPUT);

  pinMode(P_PWM4, OUTPUT);

  //RISING/HIGH/CHANGE/LOW/FALLING
  //attachInterrupt (P_CKIN1, isr_input_change_1, FALLING);  
  //attachInterrupt (P_CKIN2, isr_input_change_2, FALLING);  
  //attachInterrupt (P_CKIN3, isr_input_change_3, FALLING);  
  
  leds.begin();
  setRgbLeds(0,11);
  Serial.begin(9600);
}


byte clock_phase_1 = 0;
byte clock_phase_2 = 0;
byte clock_phase_3 = 0;

byte clock_count1 = 0;
byte clock_count2 = 0;
byte clock_count3 = 0;


 uint16_t lfsr = 1;
uint16_t state = 0;

byte update_strip = 0;
byte vert_scan = 0;
byte cycles = 0;
byte bmpindex = 0;
int twister = 0;
    byte mode = 0;

void loop() {
  /*if((freq_count_1^pwm_phase_1)&~3) {
    analogWrite(P_PWM1, (byte)(pwm_phase_1 >> 2));
    pwm_phase_1 = freq_count_1;
  }*/

byte clock1 = (CORE_PIN19_PINREG & CORE_PIN19_BITMASK); // pin 9
byte clock2 = (CORE_PIN12_PINREG & CORE_PIN12_BITMASK); // pin 12
byte clock3 = (CORE_PIN11_PINREG & CORE_PIN11_BITMASK); // pin 11


  if(clock_phase_1^clock1) {
    clock_phase_1 = clock1;
    ++clock_count1;
    analogWrite(P_PWM1, gamma8[clock_count1]);

    
  }
  
  if(clock_phase_2^clock2) {
    clock_phase_2 = clock2;
    ++clock_count2;
    analogWrite(P_PWM2, gamma8[clock_count2]);

    /////////////////////////////////
    if(mode & 1) {
      if(++vert_scan >= 15) {
        CORE_PIN0_PORTSET = CORE_PIN0_BITMASK; // OUT    
        vert_scan = 0;
      }
      else {
        CORE_PIN0_PORTCLEAR = CORE_PIN0_BITMASK; // OUT    
      }
    }
    /////////////////////////////////

  }

  /*
  if(cycles) {
    if(!--cycles) {
      CORE_PIN23_PORTCLEAR = CORE_PIN23_BITMASK; // LED    
    }
  }*/

  
  if(clock_phase_3^clock3) {
    clock_phase_3 = clock3;
    ++clock_count3;
    analogWrite(P_PWM3, gamma8[clock_count3]);

    byte data_bit = 0;
    /*if(mode == 1) {
        // taps: 16 14 13 11; feedback polynomial: x^16 + x^14 + x^13 + x^11 + 1 
      uint16_t b  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
      lfsr =  (lfsr >> 1) | (b << 15);
      uint16_t density = ((uint16_t)analogRead(A4))<<6;
      if(lfsr<density) {
        data_bit=1;
      }
    }*/
    switch(mode/2) {
      case 0:
        data_bit = 0;
        break;
      case 1:
        data_bit = 1;
        break;
      case 2:
        if(bmpindex > 63) bmpindex = 0;
        data_bit = (bmp[bmpindex>>3]>>(bmpindex&7))&1;
        ++bmpindex;
        break;
      case 3:
        if(bmpindex > 63) bmpindex = 0;
        data_bit = (bmp2[bmpindex>>3]>>(bmpindex&7))&1;
        ++bmpindex;
        break;
    }

    int noise_density = analogRead(A4);
    
    analogWrite(23,noise_density/4);
        // taps: 16 14 13 11; feedback polynomial: x^16 + x^14 + x^13 + x^11 + 1 
      uint16_t b  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
      lfsr =  (lfsr >> 1) | (b << 15);
      uint16_t density = ((uint16_t)noise_density)<<6;
      if(lfsr<density) {
        data_bit^=1;
      }
    
    
     


    if(data_bit) {
      CORE_PIN13_PORTSET = CORE_PIN13_BITMASK; // OUT
      //CORE_PIN23_PORTSET = CORE_PIN23_BITMASK; // LED
      if(!(mode & 1)) {
        CORE_PIN0_PORTSET = CORE_PIN0_BITMASK; // OUT       
      }
    } 
    else {
      CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK; // OUT
      if(!(mode & 1)) {
        CORE_PIN0_PORTCLEAR = CORE_PIN0_BITMASK; // OUT       
      }
    }
  }
  
  if(!++update_strip) {
    int r = analogRead(A3);
    
    if(++twister > 1000) twister  = 0;
    setRgbLeds(makeColor(r>>2, 255, 10), twister/100);

    
    int lower = 128*mode - 10;
    int upper = 128*mode + 10;
    if(r < lower || r > upper) {
      mode = r/128; // 8 modes
    }
  }
//  analogWrite(P_PWM4, gamma8[analogRead(A4)/8]);
///    Serial.print(analogRead(A4));
//  Serial.print(",");
//  Serial.println(analogRead(A3));

}


