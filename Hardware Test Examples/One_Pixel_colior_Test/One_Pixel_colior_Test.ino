#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <Wire.h>               // i2c Library
#include <tca9544a.h>
#include "veml6040.h"

VEML6040 RGBWSensor;


typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;




//TwoWire Wire
TCA9544A pixel1(Wire);
TCA9544A pixel2(Wire);
TCA9544A pixel3(Wire);
TCA9544A pixel4(Wire);

uint8_t TCA9554a_1_Adress = 0x77;
uint8_t TCA9554a_2_Adress = 0x71;
uint8_t TCA9554a_3_Adress = 0x72;
uint8_t TCA9554a_4_Adress = 0x73;


bool touchButtonState = false;
bool hallSensorState[4] = {false, false, false, false,};



void setup() 
{
  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println("TCA9554 Simple Test Sketch");

 

// PINOUT
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();



  // Begin communication with the pixel1
  if (!pixel1.begin(TCA9554a_1_Adress)) { // return true if connected without error
    Serial.print("Failed to find TCA9544a 1 chip on Address ");
    Serial.print(TCA9554a_1_Adress, HEX);
    Serial.println(", check Code for the correct i2c address, eg 0x70 - 0x77");
  }
  else { 
    Serial.print("Found TCA9544a 1 chip on Address ");
    Serial.println(TCA9554a_1_Adress, HEX);
  }


  pixel1.setChannel(1);



  pixel1.setChannel(0);
  if(!RGBWSensor.begin()) {
    Serial.println("ERROR: couldn't detect the sensor");
  }
  else {
    //init RGBW sensor with: - 320ms integration time - auto mode - color sensor enable
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  }


  delay(2000);


}




void loop() 
{
  readColourSensor1();
  delay(1000);
}







hsv hsv_color;
rgb rgb_color;


hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;
    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;
    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = 0.0;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}




void readColourSensor1()
{
  double r,g,b;
  // read Sensor
  r = RGBWSensor.getRed()/32768.0;
  g = RGBWSensor.getGreen()/32768.0;
  b = RGBWSensor.getBlue()/32768.0;

  Serial.print("RED: "); Serial.print(r);
  Serial.print(" GREEN: "); Serial.print(g);
  Serial.print(" BLUE: "); Serial.println(b);

  double      min, max, delta;

    min = r < g ? r : g;
    min = min  < b ? min  : b;

    max = r > g ? r : g;
    max = max  > b ? max  : b;

  Serial.print("Reading Colour: ");
  Serial.print("RED: ");
  Serial.print(RGBWSensor.getRed());  
  Serial.print(" GREEN: ");
  Serial.print(RGBWSensor.getGreen());  
  Serial.print(" BLUE: ");
  Serial.print(RGBWSensor.getBlue());  
  Serial.print(" WHITE: ");
  Serial.print(RGBWSensor.getWhite()); 
  Serial.print(" CCT: ");
  Serial.print(RGBWSensor.getCCT());  
  Serial.print(" AL: ");
  Serial.println(RGBWSensor.getAmbientLight()); 

      rgb_color.r = RGBWSensor.getRed() / 16384.0;
    rgb_color.g = RGBWSensor.getGreen() / 16384.0;
    rgb_color.b = RGBWSensor.getBlue() / 16384.0;
    hsv_color = rgb2hsv(rgb_color);

    Serial.print("hue: "), Serial.print(hsv_color.h);
    Serial.print(" sat: "), Serial.print(hsv_color.s);
    Serial.print(" val: "), Serial.println(hsv_color.v);
}




int detect_color() {


    uint8_t color_detected = 0;
    rgb_color.r = RGBWSensor.getRed() / 28000.0;
    rgb_color.g = RGBWSensor.getGreen() / 28000.0;
    rgb_color.b = RGBWSensor.getBlue() / 28000.0;
    hsv_color = rgb2hsv(rgb_color);

    Serial.print("hue: "), Serial.print(hsv_color.h);
    Serial.print(" saturation: "), Serial.print(hsv_color.s);
    Serial.print(" value: "), Serial.println(hsv_color.v);

/*
    if (hsv_color.s >= 0.4) {           // if below 0.4 its too dark, we dont regocnize as a colour
        uint8_t k = 0;
        for (k = 0; k < settings.NoteAmount; k++) {
            //printf("array_size: %d, color_min: %d, color_max: %d\"\n\r", settings.NoteAmount, NoteConfiguration[k].min_hue, NoteConfiguration[k].max_hue);
            if ((hsv_color.h >= NoteConfiguration[k].min_hue) && (hsv_color.h <= NoteConfiguration[k].max_hue)) {
                color_detected = NoteConfiguration[k].note;
                printf("\"color\":\"%d\"\r", color_detected);
                return color_detected;
            }
        }
    }
*/
     return -1;  //-> we dont play a note
}
