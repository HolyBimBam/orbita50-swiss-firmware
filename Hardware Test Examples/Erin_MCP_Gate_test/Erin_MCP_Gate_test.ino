#include <Adafruit_TinyUSB.h>   // has to be first item in list
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

#include <tca9544a.h>
#include "veml6040.h"



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
#define MCP_ADDR 0x60
#define DISPLAY_ADDR 0x70 
#define PIXEL1_ADDR 0x71
#define PIXEL2_ADDR 0x72
#define PIXEL3_ADDR 0x73
#define PIXEL4_ADDR 0x74
/////////////////////////////////////////////


#define BUTTON_UPDATE_RATE 40
#define MOTOR_UPDATE_RATE 500   // for now to prevent too quick changes

#define MIN_MOTOR_SPEED 130
#define MAX_MOTOR_SPEED 180

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
#define NUMPIXELS 28 
#define BRIGHTNESS 255
#define BUTTON_COLOR pixels.Color(250, 110, 20)

#define ENCODER_SWITCH        24
#define CONFIG_ENCODER0_A_PIN 9 //TODO: VERIFY THIS HACK
#define CONFIG_ENCODER0_B_PIN 8
////////////////////////////////////////////////////////




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



hsv hsv_color;
rgb rgb_color;





Adafruit_7segment display = Adafruit_7segment();
Adafruit_seesaw encoder;
Adafruit_MCP4728 mcp;
TCA9535 TCA(TCA_ADDR);

TCA9544A pixel1(Wire);
TCA9544A pixel2(Wire);
TCA9544A pixel3(Wire);
TCA9544A pixel4(Wire);

VEML6040 RGBWSensor;


Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);



// USB MIDI object
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI)  // USB HOST IO via UART PIN 4/5


void handleControlChange(byte channel, byte number, byte value)   // For Remote Control via Browser/Midi-CC Input
{
  if(number >= 21 && number < 33){
    //threshHold[number-21] = value*2;
  }
}

////////////////////////////////////////////////////////






bool trackIsOn[4]={1,1,1,1};
bool motorIsOn = false;
int currentMotorSpeed = 0;
int targetMotorSpeed = 0;
int32_t encoder_position;
bool hallSensorState[4] = {false, false, false, false,};



void setup() 
{
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  usb_midi.setStringDescriptor("ORBITA BIG");

  // Initialize MIDI, and listen to channel 1
  MIDI_USB.begin(1);  // cann also be omni// all channels: (MIDI_CHANNEL_OMNI);
  MIDI_USB.turnThruOff();
  //MIDI_USB.setHandleNoteOn(handleNoteOn);
  //MIDI_USB.setHandleNoteOff(handleNoteOff);
  MIDI_USB.setHandleControlChange(handleControlChange);
  MIDI.begin(1);  //listenign to MIDI Channel 1 or MIDI_CHANNEL_OMNI
  MIDI.turnThruOff();




#if DEBUG == 1
  Serial.begin(115200);
  //while (!Serial) delay(1);
  debugln("ORBITA BIG SWISS EDITION");
#endif


  // SETUP ERIN MULTIPLEXER FOR MIDI CONNECTION
  pinMode(MULTIPLEX_A_PIN, OUTPUT);
  pinMode(MULTIPLEX_B_PIN, OUTPUT);
  pinMode(MULTIPLEX_INH_PIN, OUTPUT);
  digitalWrite(MULTIPLEX_A_PIN, HIGH);
  digitalWrite(MULTIPLEX_B_PIN, HIGH);
  digitalWrite(MULTIPLEX_INH_PIN, HIGH);


  // SETUP MOTOR PINS
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, 0);


  // SETUP I2C BUS and DEVICES
  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.begin();


  // SETUP 7-SEGMENT DISPLAY
  display.begin(0x70);
  display.print("ORB");
  display.writeDisplay();


  // SETUP ENCODER
  if (! encoder.begin(ENCODER_ADDR)) {
    debugln("Couldn't find seesaw on default address");
  }
  encoder.pinMode(ENCODER_SWITCH, INPUT_PULLUP); // use a pin for the built in encoder switch
  encoder_position = encoder.getEncoderPosition();
  debugln("Turning on interrupts");
  delay(10);
  encoder.setGPIOInterrupts((uint32_t)1 << ENCODER_SWITCH, 1);
  encoder.enableEncoderInterrupt();


  // SETUP BUTTONS
  TCA.begin();
  //Set pinMode16 INPUT
  TCA.pinMode16(0xFFFF);


  // SETUP NEOPIXEL LEDS
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(BRIGHTNESS);
  for(int i=3; i>0; i--) { // For each pixel...
    pixels.setPixelColor(i, BUTTON_COLOR);
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(200);
  }
  updateLEDs();

}


void loop() 
{
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif


  updateHallSensors();
  updateEncoder();
  updateButtons();
  updateMotor();


  //updateMIDIBuffer();


  // read any new MIDI messages
  while (MIDI_USB.read()) {}
  while (MIDI.read()) {}
}


void setup1(){

}

void loop1(){

}







/*
  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }
  */



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
      debugln("Encoder Switch pressed!");
      enc_ButtonState = true;
      if(motorIsOn) stopMotor();
      else startMotor();
    }
  } else if (enc_ButtonState){
      debugln("Encoder Switch released");
      enc_ButtonState = false;
  }

  int32_t new_position = encoder.getEncoderPosition();
  // did we move arounde?
  if (encoder_position != new_position && new_position >= MIN_MOTOR_SPEED && new_position <= MAX_MOTOR_SPEED) {
    debugln(new_position);         // display new position
    encoder_position = new_position;      // and save for next round
    display.print(encoder_position);
    display.writeDisplay();
  } else if (new_position < MIN_MOTOR_SPEED){
    encoder_position = MIN_MOTOR_SPEED;
    encoder.setEncoderPosition(encoder_position);
  } else if(new_position > MAX_MOTOR_SPEED){
    encoder_position = MAX_MOTOR_SPEED;
    encoder.setEncoderPosition(encoder_position);
  }

  targetMotorSpeed = encoder_position;

}





void updateLEDs()
{
  for(int i=0; i<4; i++) { // For each pixel...
    if(trackIsOn[i]) {
      switch(i){
        case 0:
          pixels.setPixelColor(3, BUTTON_COLOR);
          pixels.setPixelColor(16, pixels.Color(255, 255, 255));
          pixels.setPixelColor(18, pixels.Color(255, 255, 255));
          break;
        case 1:
          pixels.setPixelColor(2, BUTTON_COLOR);
          pixels.setPixelColor(19, pixels.Color(255, 255, 255));
          pixels.setPixelColor(21, pixels.Color(255, 255, 255));
          break;
        case 2:
          pixels.setPixelColor(1, BUTTON_COLOR);
          pixels.setPixelColor(22, pixels.Color(255, 255, 255));
          pixels.setPixelColor(24, pixels.Color(255, 255, 255));
          break;
        case 3:
          pixels.setPixelColor(0, BUTTON_COLOR);
          pixels.setPixelColor(25, pixels.Color(255, 255, 255));
          pixels.setPixelColor(27, pixels.Color(255, 255, 255));
      default:
        break;
      }
    }
    else  {
      switch(i){
        case 0:
          pixels.setPixelColor(3, pixels.Color(0, 0, 0));
          pixels.setPixelColor(16, pixels.Color(0, 0, 0));
          pixels.setPixelColor(18, pixels.Color(0, 0, 0));
          break;
        case 1:
          pixels.setPixelColor(2, pixels.Color(0, 0, 0));
          pixels.setPixelColor(19, pixels.Color(0, 0, 0));
          pixels.setPixelColor(21, pixels.Color(0, 0, 0));
          break;
        case 2:
          pixels.setPixelColor(1, pixels.Color(0, 0, 0));
          pixels.setPixelColor(22, pixels.Color(0, 0, 0));
          pixels.setPixelColor(24, pixels.Color(0, 0, 0));
          break;
        case 3:
          pixels.setPixelColor(0, pixels.Color(0, 0, 0));
          pixels.setPixelColor(25, pixels.Color(0, 0, 0));
          pixels.setPixelColor(27, pixels.Color(0, 0, 0));
      default:
        break;
      }
    }
  }
  pixels.show();   // Send the updated pixel colors to the hardware.
}


void updateButtons()
{
  static long lastUpdate = millis();
  static bool buttonState[4] = {0,0,0,0};

  if(millis()-lastUpdate > BUTTON_UPDATE_RATE)
  {
    for (int i = 0; i < 4; i++)
    {
      int val = !TCA.read1(i);

      if(val != buttonState[3-i]){
        if(val){
          // Button just pressed
          debug("Button ");
          debug(3-i);
          debugln(" just pressed");
          trackIsOn[3-i] = !trackIsOn[3-i];
          updateLEDs();
          
        }
        else {
          debug("Button ");
          debug(3-i);
          debugln(" just released");
        }

        buttonState[3-i] = val;
      }
    }
    lastUpdate = millis();
  }
}


void stopMotor(){
  motorIsOn = false;
  targetMotorSpeed = 0;
  analogWrite(MOTOR_PWM_PIN, 0);
  encoder.setEncoderPosition(0);
  display.print(targetMotorSpeed);
  display.writeDisplay();
}

void startMotor(){
  motorIsOn = true;
  targetMotorSpeed = MIN_MOTOR_SPEED;
  analogWrite(MOTOR_PWM_PIN, 100);
  currentMotorSpeed = 100;
  encoder.setEncoderPosition(MIN_MOTOR_SPEED);
  display.print(MIN_MOTOR_SPEED);
  display.writeDisplay();
}


void updateMotor()
{
  static long lastUpdate = millis();

  if(millis()-lastUpdate > MOTOR_UPDATE_RATE)
  {
    if(!motorIsOn){  // MOTOR is OFF
      if(currentMotorSpeed != 0){
        currentMotorSpeed = 0;
        analogWrite(MOTOR_PWM_PIN, 0);
      }
    } 
    else {
      if(currentMotorSpeed != targetMotorSpeed){
        if(currentMotorSpeed < targetMotorSpeed){
          if((targetMotorSpeed - currentMotorSpeed) > 3){
            currentMotorSpeed = currentMotorSpeed+3;
          }
        }
        else {
          currentMotorSpeed = targetMotorSpeed;
        }
      }
      analogWrite(MOTOR_PWM_PIN, currentMotorSpeed);
    }
    lastUpdate = millis();
  }
}



void sendNoteOn(uint8_t pitch=64, uint8_t velocity=88, uint8_t channel = 1)
{
  MIDI_USB.sendNoteOn(pitch, velocity, channel);
  MIDI.sendNoteOn(pitch, velocity, channel);
}

void sendNoteOff(uint8_t pitch=64, uint8_t channel = 1)
{
  MIDI_USB.sendNoteOff(pitch, 0, channel);
  MIDI.sendNoteOff(pitch, 0, channel);
}


void setupBridgeSensors()
{
  // Begin communication with the pixel1
  if (!pixel1.begin(PIXEL1_ADDR)) { // return true if connected without error
    debug("Failed to find TCA9544a 1 chip on Address ");
    debug2(PIXEL1_ADDR, HEX);
    debugln(", check Code for the correct i2c address, eg 0x70 - 0x77");
    while (1);
  }
  debug("Found TCA9544a 1 chip on Address ");
  debug2ln(PIXEL1_ADDR, HEX);

  // Begin communication with the pixel2
  if (!pixel2.begin(PIXEL2_ADDR)) { // return true if connected without error
    debug("Failed to find TCA9544a 2 chip on Address ");
    debug2(PIXEL2_ADDR, HEX);
    debugln(", check Code for the correct i2c address, eg 0x70 - 0x77");
    while (1);
  }
  debug("Found TCA9544a 2 chip on Address ");
  debug2ln(PIXEL2_ADDR, HEX);

  // Begin communication with the pixel3
  if (!pixel3.begin(PIXEL3_ADDR)) { // return true if connected without error
    debug("Failed to find TCA9544a 3 chip on Address ");
    debug2(PIXEL3_ADDR, HEX);
    debugln(", check Code for the correct i2c address, eg 0x70 - 0x77");
    while (1);
  }
  debug("Found TCA9544a 3 chip on Address ");
  debug2ln(PIXEL3_ADDR, HEX);

  // Begin communication with the pixel4
  if (!pixel4.begin(PIXEL4_ADDR)) { // return true if connected without error
    debug("Failed to find TCA9544a 4 chip on Address ");
    debug2(PIXEL4_ADDR, HEX);
    debugln(", check Code for the correct i2c address, eg 0x70 - 0x77");
    while (1);
  }
  debug("Found TCA9544a 4 chip on Address ");
  debug2ln(PIXEL4_ADDR, HEX);


  pixel1.setChannel(1);
  pixel2.setChannel(1);
  pixel3.setChannel(1);
  pixel4.setChannel(1);


  pixel1.setChannel(0);
  if(!RGBWSensor.begin()) {
    debugln("ERROR: couldn't detect the sensor");
    while(1){}
  }
  else {
    //init RGBW sensor with: - 320ms integration time - auto mode - color sensor enable
    RGBWSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  }
  pixel1.setChannel(1);

  pixel2.setChannel(0);
  if(!RGBWSensor.begin()) {
    debugln("ERROR: couldn't detect the sensor");
    while(1){}
  }
  else {
    //init RGBW sensor with: - 320ms integration time - auto mode - color sensor enable
    RGBWSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  }
  pixel2.setChannel(1);

  pixel3.setChannel(0);
  if(!RGBWSensor.begin()) {
    debugln("ERROR: couldn't detect the sensor");
    while(1){}
  }
  else {
    //init RGBW sensor with: - 320ms integration time - auto mode - color sensor enable
    RGBWSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  }
  pixel3.setChannel(1);

  pixel4.setChannel(0);
  if(!RGBWSensor.begin()) {
    debugln("ERROR: couldn't detect the sensor");
    while(1){}
  }
  else {
    //init RGBW sensor with: - 320ms integration time - auto mode - color sensor enable
    RGBWSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  }
  pixel4.setChannel(1);

  // SET PIXEL to CH 1 -> no i2C dev connected, so we can select each pixels channel 0 one after another
  pixel1.setChannel(1);
  pixel2.setChannel(1);
  pixel3.setChannel(1);
  pixel4.setChannel(1);
}





void updateHallSensors()
{
  // READ HALL SENSORS
  readHallSensorX(0);
  readHallSensorX(1);
  readHallSensorX(2);
  readHallSensorX(3);
}



bool readHallSensorX(uint8_t track)
{
  // Check if pin 4 is pulled low (magnet event)
  if (isHallSensorHigh(track)) {
    if(!hallSensorState[track]) {
      Serial.print("Magnet Detected T"); Serial.println(track);
      hallSensorState[track] = true;
      //turnGateOnOff(track, true);
      detect_color(track);
    }
    return true;
  } else if(hallSensorState[track]){
    hallSensorState[track] = false;
    //sendNoteOff(64+track,1);
    //turnGateOnOff(track, false);
    return false;
  }
  return false;
}



bool isHallSensorHigh(uint8_t track)
{
  switch (track){
    case 0:
      if(bitRead(pixel1.readInterrupts(),0) == HIGH) return true;
      else return false;
    case 1:
      if(bitRead(pixel2.readInterrupts(),0) == HIGH) return true;
      else return false;  
    case 2:
      if(bitRead(pixel3.readInterrupts(),0) == HIGH) return true;
      else return false;
    case 3:
      if(bitRead(pixel4.readInterrupts(),0) == HIGH) return true;
      else return false; 
    default:
      break;
  } 
  return false;
}






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



void readColourSensor1(){
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
}




int detect_color(uint8_t track) 
{
  if(track == 0) pixel1.setChannel(0);
  else if(track == 1) pixel2.setChannel(0);
  else if(track == 2) pixel3.setChannel(0);
  else if(track == 3) pixel4.setChannel(0);

  uint8_t color_detected = 0;
  rgb_color.r = RGBWSensor.getRed() / 28000.0;
  rgb_color.g = RGBWSensor.getGreen() / 28000.0;
  rgb_color.b = RGBWSensor.getBlue() / 28000.0;
  hsv_color = rgb2hsv(rgb_color);

  Serial.print("hue: "), Serial.print(hsv_color.h);
  Serial.print(" saturation: "), Serial.print(hsv_color.s);
  Serial.print(" value: "), Serial.println(hsv_color.v);

  if (hsv_color.s >= 0.0002) {           // if below 0.4 its too dark, we dont regocnize as a colour
    sendNoteOn(64+track,99,1);
  }

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

  if(track == 0) pixel1.setChannel(1);
  else if(track == 1) pixel2.setChannel(1);
  else if(track == 2) pixel3.setChannel(1);
  else if(track == 3) pixel4.setChannel(1);


  return -1;  //-> we dont play a note
}