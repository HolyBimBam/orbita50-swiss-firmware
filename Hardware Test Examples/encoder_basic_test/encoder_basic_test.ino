//#include <Adafruit_TinyUSB.h>   // has to be first item in list
#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <MIDI.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_seesaw.h"
#include "TCA9555.h"



/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_seesaw.h"
//#include <seesaw_neopixel.h>
*/


#define DEBUG 1

////////////////////////////////////////////
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debug2(x,y) Serial.print(x,y)
#define debugln(x) Serial.println(x)
#define debug2ln(x,y) Serial.println(x,y)
#else
#define debug(x) 
#define debug2(x,y) 
#define debugln(x)
#define debug2ln(x,y)
#endif
////////////////////////////////////////////


////////////////// I2C Device Adresses //////
#define TCA_ADDR 0x27
#define ENCODER_ADDR  0x36
// MCP 0x60
// DISPLAY 0x70 
#define PIXEL1_ADDR 0x71
#define PIXEL2_ADDR 0x72
#define PIXEL3_ADDR 0x73
#define PIXEL4_ADDR 0x74
/////////////////////////////////////////////


#define BUTTON_UPDATE_RATE 20
#define MOTOR_UPDATE_RATE 100

#define MIDI_TX_PIN 4

#define SDA0_PIN  12
#define SCL0_PIN  13
#define SDA1_PIN  22
#define SCL1_PIN  23

#define MULTIPLEX_INH_PIN 9
#define MULTIPLEX_A_PIN 10
#define MULTIPLEX_B_PIN 11

#define MOTOR_DIR_PIN 19
#define MOTOR_PWM_PIN 20

#define INT_BRIDGE_PIN 24
#define INT_EXT_PIN 25

#define NEOPIXEL_PIN  21 
#define NUMPIXELS 16 // Popular NeoPixel ring size
#define BRIGHTNESS 255

#define ENCODER_SWITCH        24
#define ENCODER_ADDR          0x36


Adafruit_7segment display = Adafruit_7segment();
Adafruit_seesaw encoder;


int32_t encoder_position;
int motor_pwm_pin = 20;         // the PWM pin the LED is attached to

#define SDA0_PIN  12
#define SCL0_PIN  13

void setup() {

  Serial.begin(115200);
  //while (!Serial) delay(1);
  Serial.println("motor Test Start");

  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.begin();

  // SETUP 7-SEGMENT DISPLAY
  display.begin(0x70);
  display.print("ORBI");
  display.writeDisplay();


  pinMode(20, OUTPUT);

  if (! encoder.begin(ENCODER_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");




  // use a pin for the built in encoder switch
  encoder.pinMode(ENCODER_SWITCH, INPUT_PULLUP);
  encoder_position = encoder.getEncoderPosition();
  Serial.println("Turning on interrupts");
  delay(10);
  encoder.setGPIOInterrupts((uint32_t)1 << ENCODER_SWITCH, 1);
  encoder.enableEncoderInterrupt();



  // SETUP MOTOR PINS
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);

}


void loop() {
  updateEncoder();
  delay(10);
}




/*
 * This example shows how to read from a seesaw encoder module.
 * The available encoder API is:
 *      int32_t getEncoderPosition();
        int32_t getEncoderDelta();
        void enableEncoderInterrupt();
        void disableEncoderInterrupt();
        void setEncoderPosition(int32_t pos);
 */

void updateEncoder() {

  static bool enc_ButtonState = false;

  if (! encoder.digitalRead(ENCODER_SWITCH)) {
    if(!enc_ButtonState){
      Serial.println("Encoder Switch pressed!");
      enc_ButtonState = true;
    }
  } else if (enc_ButtonState){
      Serial.println("Encoder Switch released");
      enc_ButtonState = false;
  }

  int32_t new_position = encoder.getEncoderPosition();
  // did we move arounde?
  if (encoder_position != new_position && new_position >= 0 && new_position <= 255) {
    Serial.println(new_position);         // display new position
    encoder_position = new_position;      // and save for next round
    display.print(encoder_position);
    display.writeDisplay();
  }

  analogWrite(20, encoder_position);

}

