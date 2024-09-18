#include <Adafruit_TinyUSB.h>   // has to be first item in list?
#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
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
//#include <LittleFS.h>
#include <EEPROM.h>

// ERIN USES WSW25Q16 FLASH with 2MB

/////// SETTINGS ///////////////////////////

#define VERSION_NUMBER 3

#define VEML_TRIGGER_MODE 1

#define DEBUG 1

#define BRIGHTNESS 100
#define BUTTON_COLOR pixels.Color(255, 80, 5)

#define BUTTON_UPDATE_RATE 40
#define MOTOR_UPDATE_RATE 400   // for now to prevent too quick changes

#define MIN_MOTOR_SPEED 145
#define MAX_MOTOR_SPEED 220

uint8_t midiNotes[4][4] = {{36,37,38,39}, 
                           {40,41,42,43}, 
                           {44,45,46,47}, 
                           {48,49,50,51}};

                           

uint8_t cvNotes[4][4] = {{16,28,40,52}, 
                         {28,32,35,40}, 
                         {28,31,35,40}, 
                         {28,32,35,37}};

uint16_t dac_PitchValues[] = {
    //Octave -1 (F#3 to B3)
    0,     68,  137,  205,  273,  341, 
    //Octave 0 (C4 to B4)
    410,  478,  546,  614,  683,  751,  819,  887,  956, 1024, 1092, 1161,
    //Octave 1 (C5 to B5)
    1229, 1297, 1365, 1434, 1502, 1570, 1638, 1707, 1775, 1843, 1911, 1980, 
    //Octave 2 (C6 to B6)
    2048, 2116, 2185, 2253, 2321, 2389, 2458, 2526, 2594, 2662, 2731, 2799, 
    //Octave 3 (C7 to B7)
    2867, 2935, 3004, 3072, 3140, 3209, 3277, 3345, 3413, 3482, 3550, 3618, 
    //Octave 4 (C8 to F8)
    3686, 3755, 3823, 3891, 3959, 4028
}; // dac_values[6] = C4 -> input 16

uint8_t midiNotesChromatic[4][4] = {{36,37,38,39}, 
                           {40,41,42,43}, 
                           {44,45,46,47}, 
                           {48,49,50,51}};

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



////////////////// PINOUTS //////////////////

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

#define GATE1_PIN 27
#define GATE2_PIN 1
#define GATE3_PIN 23
#define GATE4_PIN 25

#define NEOPIXEL_PIN  21 
#define NUMPIXELS 28 

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


////////////////////////////////////////////////////////


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



#define PIN_SERIAL2_TX (4u)
#define PIN_SERIAL2_RX (5u)

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI)  // USB HOST IO via UART PIN 4/5





////////////////////////////////////////////////////////

bool trackIsOn[4]={1,1,1,1};
bool motorIsOn = false;
bool motorJustStarted = false;
int currentMotorSpeed = 0;
uint8_t currentCVMode = 0;
int targetMotorSpeed = 0;
int32_t encoder_position;
bool hallSensorState[4] = {false, false, false, false,};
hsv hsv_color;
rgb rgb_color;
char buff[288];
uint8_t motorMinSpeed = MIN_MOTOR_SPEED;
uint8_t motorMaxSpeed = MAX_MOTOR_SPEED;
static int lastMeasuredHue[4] = {50,50,50,50};
long sensorReadOutTime[4] = {-1,-1,-1,-1};

////////////////////////////////////////////////////////




void handleControlChange(byte channel, byte number, byte value)   // For Remote Control via Browser/Midi-CC Input
{
  debug("RX MIDI CC: "); debug(number); debug("-"); debugln(value);
  if(number >= 21 && number < 37)
  {
    midiNotes[((number-21)/4)][((number-21)%4)] = value;
    //saveSettings();
  } 
  else if(number >= 37 && number < 53)
  {
    if(value<10) value = 10;     // C4 = 0.5 V = step 6 - 40-C4-6  ->34-F#3-0  -> 34+(5+12)-1 Steps: 93
    if (value>70) value = 70;
    cvNotes[(int)((number-37)/4)][((number-37)%4)] = value;
    debug("Changed Pitch of Track "); debug((int)((number-37)/4)); debug(" col: "); debug(((number-37)%4)); debug(" to "); debugln(value);
    //saveSettings();
  }
  else if(number == 53){
    if(value>126) { 
        debugln("Save Settings to Memory");
        //saveSettingsToFile(); 
        saveSettings();
    } 
  }
  else if(number == 54){
    if(value == 1)  {
      currentCVMode = 1;     // CV COLOR MODe
      debugln("Changed to Color CV Mode");
    }
    else if (value == 0) {
      currentCVMode = 0;               // CV PITCH MOE
      debugln("Changed to Pitch CV Mode");
    }
    else if (value == 66){
      debugln("Resetting MIDI Pitches to Chromatic");
      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++){
          midiNotes[t][p] = midiNotesChromatic[t][p];
        }
      }
    }
    else if (value == 77){
      debugln("Resetting MOTOR VALUES");
      motorMinSpeed = MIN_MOTOR_SPEED;
      motorMaxSpeed = MAX_MOTOR_SPEED;
    }
  }
  else if(number == 55){
    if(value >=0 && value<=127){
      motorMinSpeed = value+127;
    }
  }
  else if(number == 56){
    if(value >=0 && value<=127){
      motorMaxSpeed = value+127;
    }
  }
}





/*
  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }
  */

/////// SETUP /////////////////////////////////////////////////


void setup() 
{
/*
    // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
  // Initialize MIDI, and listen to all MIDI channels
  usb_midi.setStringDescriptor("ORBITA SWISS");  // seems to not  work
  USBDevice.setManufacturerDescriptor("PLAYTRONICA                     ");
  USBDevice.setProductDescriptor     ("ORBITA SWISS                    ");    // works
*/

  // SETUP USB MIDI
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) { TinyUSBDevice.begin(0); }
  usb_midi.setStringDescriptor("ORBITA SWISS");
  USBDevice.setManufacturerDescriptor("PLAYTRONICA                     ");
  USBDevice.setProductDescriptor     ("ORBITA SWISS                    ");    // TO TEST!


  // Initialize MIDI, and listen to channel 1
  MIDI_USB.begin(1);  // cann also be omni// all channels: (MIDI_CHANNEL_OMNI);
  MIDI_USB.turnThruOff();
  MIDI_USB.setHandleControlChange(handleControlChange);
  //MIDI_USB.setHandleNoteOn(handleNoteOn);
  //MIDI_USB.setHandleNoteOff(handleNoteOff);
  MIDI.begin(1);  //listenign to MIDI Channel 1 or MIDI_CHANNEL_OMNI
  MIDI.turnThruOff();


  // SETUP SERIAL DEBUG PORT (can be deactivated via DEBUG define = 0)
#if DEBUG == 1
  Serial.begin(115200);
  //while (!Serial) delay(1);
  delay(4000);
  debugln("ORBITA BIG SWISS EDITION");
#endif

  //loadSettingsFromMemory();
  EEPROM.begin(512);
  loadSettings();

  // SETUP ERIN MULTIPLEXER FOR MIDI CONNECTION
  pinMode(MULTIPLEX_A_PIN, OUTPUT);
  pinMode(MULTIPLEX_B_PIN, OUTPUT);
  pinMode(MULTIPLEX_INH_PIN, OUTPUT);
  digitalWrite(MULTIPLEX_A_PIN, HIGH);  //HIGH
  digitalWrite(MULTIPLEX_B_PIN, LOW);   //LOW
  digitalWrite(MULTIPLEX_INH_PIN, LOW); //LOW 


  // SETUP MOTOR PINS
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  //pinMode(MOTOR_PWM_PIN, OUTPUT);
  //analogWrite(MOTOR_PWM_PIN, 0);
  
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.f);        //HBB: was (&config, 8.f), the smaller the number, the more usable PWM speeds we have
    pwm_config_set_wrap(&config, 255);
    pwm_init(pwm_gpio_to_slice_num(MOTOR_PWM_PIN), &config, true);
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(MOTOR_PWM_PIN, 0);





  // SETUP GATES
  pinMode(GATE1_PIN, OUTPUT);
  pinMode(GATE2_PIN, OUTPUT);
  pinMode(GATE3_PIN, OUTPUT);
  pinMode(GATE4_PIN, OUTPUT);
  digitalWrite(GATE1_PIN, LOW);
  digitalWrite(GATE2_PIN, LOW);
  digitalWrite(GATE3_PIN, LOW);
  digitalWrite(GATE4_PIN, LOW);


  // SETUP I2C BUS and DEVICES
  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.begin();


  // SETUP 7-SEGMENT DISPLAY
  display.begin(0x70);
  display.print("ORB");
  display.writeDisplay();


  // SETUP ENCODER 
  if (! encoder.begin(ENCODER_ADDR)) { debugln("ERROR: Couldn't find Encoder on default address"); }
  encoder.pinMode(ENCODER_SWITCH, INPUT_PULLUP); // use a pin for the built in encoder switch
  encoder_position = encoder.getEncoderPosition();
  delay(10);
  encoder.setGPIOInterrupts((uint32_t)1 << ENCODER_SWITCH, 1);  //Turning on interrupts
  encoder.enableEncoderInterrupt();


  // SETUP BUTTONS
  if (!TCA.begin()) { debugln("ERROR: Failed to find TCA chip from Button Board"); }
  TCA.pinMode16(0xFFFF);  //Set pinMode16 INPUT


  // SETUP COLOR/HALL SENSORS ////////////////////////
  if (!pixel1.begin(PIXEL1_ADDR)) { // return true if connected without error
    debug("ERROR: Failed to find TCA9544a 1 chip on Address "); 
    debug2ln(PIXEL1_ADDR, HEX);
  }
  if (!pixel2.begin(PIXEL2_ADDR)) { 
    debug("ERROR: Failed to find TCA9544a 2 chip on Address "); 
    debug2ln(PIXEL2_ADDR, HEX);
  }
  if (!pixel3.begin(PIXEL3_ADDR)) { 
    debug("ERROR: Failed to find TCA9544a 3 chip on Address ");
    debug2ln(PIXEL3_ADDR, HEX);
  }
  if (!pixel4.begin(PIXEL4_ADDR)) { 
    debug("ERROR: Failed to find TCA9544a 4 chip on Address ");
    debug2ln(PIXEL4_ADDR, HEX);
  }

  pixel1.setChannel(1);   //SET PIXEL to CH 1 -> no i2C dev connected, so we can select each pixels channel 0 one after another
  pixel2.setChannel(1);
  pixel3.setChannel(1);
  pixel4.setChannel(1);

  pixel1.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 1"); }
  else { 
    if(VEML_TRIGGER_MODE) RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    else RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE); 
  }
  pixel1.setChannel(1);

  pixel2.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 2"); }
  else { 
    if(VEML_TRIGGER_MODE) RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    else RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);   
  }
  pixel2.setChannel(1);

  pixel3.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 3"); }
  else { 
    if(VEML_TRIGGER_MODE) RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    else RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);   
  }
  pixel3.setChannel(1);

  pixel4.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 4"); }
  else { 
    if(VEML_TRIGGER_MODE) RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    else RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);   
  }
  pixel4.setChannel(1);


  // SETUP MCP CV DAC ////////////////////////
  if (!mcp.begin()) { debugln("ERROR: Failed to find MCP4728 chip"); }
  setCV(0, 2048);
  setCV(1, 2048);
  setCV(2, 2048);
  setCV(3, 2048);


  // SETUP NEOPIXEL LEDS ////////////////////////
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(BRIGHTNESS);
  for(int i=3; i>0; i--) { // For each pixel...
    pixels.setPixelColor(i, BUTTON_COLOR);
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(200);
  }
  updateLEDs();

}


/////// LOOP /////////////////////////////////////////////////



void loop() 
{
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  if(VEML_TRIGGER_MODE == 0) {
    updateColorSensor(0);
    updateColorSensor(1);
    updateColorSensor(2);
    updateColorSensor(3);
  }

  updateHallSensors();
  updateEncoder();
  updateButtons();
  updateMotor();
  //updateMIDIBuffer();

  // read any new MIDI messages
  while (MIDI_USB.read()) {}
  while (MIDI.read()) {}
}


void setup1()
{

}


void loop1()
{

}


////////////////////////////////////////////////////////



void updateEncoder() 
{
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
  if (encoder_position != new_position && new_position >= motorMinSpeed && new_position <= motorMaxSpeed) {
    debugln(new_position);         // display new position
    encoder_position = new_position;      // and save for next round
    display.print(encoder_position-motorMinSpeed+1);
    display.writeDisplay();
  } else if (new_position < motorMinSpeed){
    encoder_position = motorMinSpeed;
    encoder.setEncoderPosition(encoder_position);
  } else if(new_position > motorMaxSpeed){
    encoder_position = motorMaxSpeed;
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



void stopMotor()
{
  motorIsOn = false;
  targetMotorSpeed = 0;
  //analogWrite(MOTOR_PWM_PIN, 0);
  pwm_set_gpio_level(MOTOR_PWM_PIN, 0);
  encoder.setEncoderPosition(0);
  display.print(targetMotorSpeed);
  display.writeDisplay();
}



void startMotor()
{
  motorIsOn = true;
  targetMotorSpeed = motorMinSpeed;
  //analogWrite(MOTOR_PWM_PIN, 180);
  pwm_set_gpio_level(MOTOR_PWM_PIN, 200);
  currentMotorSpeed = 200;
  motorJustStarted = true;
  encoder.setEncoderPosition(motorMinSpeed);
  display.print(1);
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
        //analogWrite(MOTOR_PWM_PIN, 0);
        pwm_set_gpio_level(MOTOR_PWM_PIN, 0);
        debug("Motor Speed: "); debugln(currentMotorSpeed);
      }
    } 
    else {
      if(currentMotorSpeed != targetMotorSpeed){
        if(currentMotorSpeed < targetMotorSpeed){
          if((targetMotorSpeed - currentMotorSpeed) > 3){
            currentMotorSpeed = currentMotorSpeed+3;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          } 
          else {
            currentMotorSpeed = targetMotorSpeed;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          }
        }
        else if(motorJustStarted) {
          // wait a bit and then slow down
          if((currentMotorSpeed - targetMotorSpeed) > 6){
            currentMotorSpeed = currentMotorSpeed-6;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          } else {
            currentMotorSpeed = targetMotorSpeed;
            motorJustStarted = false;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          }
        }
        else {
            currentMotorSpeed = targetMotorSpeed;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
        }
      }
      //analogWrite(MOTOR_PWM_PIN, currentMotorSpeed);
      pwm_set_gpio_level(MOTOR_PWM_PIN, currentMotorSpeed);
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





void updateHallSensors()
{
  readHallSensorX(0);
  readHallSensorX(1);
  readHallSensorX(2);
  readHallSensorX(3);
}


bool readHallSensorX(uint8_t track)
{
  static int lastNote[4] = {-1,-1,-1,-1};

  // Check if pin 4 is pulled low (magnet event)
  if (isHallSensorHigh(track)) {
    if(!hallSensorState[track]) {
      hallSensorState[track] = true;
      
      if(trackIsOn[track]) {

        debug("Magnet Detected T"); debugln(track);

        uint8_t note;
        if(VEML_TRIGGER_MODE) note = detect_color_TriggerMode(track);
        else note = detect_color_fromHue(track);

        turnGateOnOff(track, true);

        if(lastNote[track] != -1) {
          sendNoteOff(lastNote[track]);
          lastNote[track] = -1;
        }
        if(note>=0) { 
          sendNoteOn(midiNotes[track][note]);
          if(currentCVMode == 0) playCV_MIDI_Pitch(track, cvNotes[track][note]);
          lastNote[track] = midiNotes[track][note];
        };
      } else {
        if(lastNote[track] != -1) {
          sendNoteOff(lastNote[track]);
          turnGateOnOff(track, false);
          lastNote[track] = -1;
        }
      }

    }
    return true;
  } else if(hallSensorState[track]){
    hallSensorState[track] = false;
    if(trackIsOn[track]) {
      turnGateOnOff(track, false);
      if(lastNote[track] != -1) {
        sendNoteOff(lastNote[track]);
        lastNote[track] = -1;
      }
    }
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







void updateColorSensor(uint8_t track) 
{
  
  if(track == 0) pixel1.setChannel(0);
  else if(track == 1) pixel2.setChannel(0);
  else if(track == 2) pixel3.setChannel(0);
  else if(track == 3) pixel4.setChannel(0);

  if(VEML_TRIGGER_MODE) {
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    delay(42);
    sensorReadOutTime[track]=millis()+42;
  }

  rgb_color.r = RGBWSensor.getRed() / 16496.0;
  rgb_color.g = RGBWSensor.getGreen() / 16496.0;
  rgb_color.b = RGBWSensor.getBlue() / 16496.0;
  hsv_color = rgb2hsv(rgb_color);


  if(hsv_color.h != lastMeasuredHue[track]) {
    lastMeasuredHue[track]=hsv_color.h;
    // Set CV per Track to Hue Value in CV Colour MODE
    if(currentCVMode == 1)      setCV(track, map(hsv_color.h, 0, 390, 0, 4095));
  }

/*
  debug("Colour: ");
  debug("R: ");
  debug(RGBWSensor.getRed());  
  debug(" G: ");
  debug(RGBWSensor.getGreen());  
  debug(" B: ");
  debug(RGBWSensor.getBlue());  
  debug(" W: ");
  debug(RGBWSensor.getWhite()); 
  debug(" CCT: ");
  debugln(RGBWSensor.getCCT());  
  //debug(" AL: ");
  //debugln(RGBWSensor.getAmbientLight()); 
  debug("hue: "), debug(hsv_color.h);
  debug(" sat: "), debug(hsv_color.s);
  debug(" val: "), debugln(hsv_color.v);
*/

  // set back to other channel so we can read other sensors with same address
  if(track == 0) pixel1.setChannel(1);
  else if(track == 1) pixel2.setChannel(1);
  else if(track == 2) pixel3.setChannel(1);
  else if(track == 3) pixel4.setChannel(1);
}


int detect_color_fromHue(uint8_t track) 
{
  int color_detected = -1;

    if(lastMeasuredHue[track] > 270 || lastMeasuredHue[track] < 10){
      debug("MAGENTA");
      color_detected =  3;
    } else if(lastMeasuredHue[track] >= 10 && lastMeasuredHue[track] < 70){
      debug("RED");
      color_detected =  0;
    } else if(lastMeasuredHue[track] >= 70 && lastMeasuredHue[track] < 170){
      debug("YELLOW");
      color_detected =  1;
    } else  {
      debug("BLUE");
      color_detected =  2;
    }
  debug(" - Hue: ");
  debugln(lastMeasuredHue[track]);

  return color_detected;  //-> if we dont play a note we send -1
}




int detect_color_TriggerMode(uint8_t track) 
{
  updateColorSensor(track);
  return (detect_color_fromHue(track));  //-> if we dont play a note we send -1
}


void turnGateOnOff(uint8_t track, bool onf){
  switch(track){
    case 0:
      digitalWrite(GATE1_PIN, onf);
      break;
    case 1:
      digitalWrite(GATE2_PIN, onf);
      break;
    case 2:
      digitalWrite(GATE3_PIN, onf);
      break;
    case 3:
      digitalWrite(GATE4_PIN, onf);
      break;          
    default:
      break;
  }
}


void setCV(uint8_t _track, int _cvVal)
{
  if(_cvVal>=0 && _cvVal<= 4095){
    switch (_track){
      case 0:
        mcp.setChannelValue(MCP4728_CHANNEL_A, _cvVal);
        break;
      case 1:
        mcp.setChannelValue(MCP4728_CHANNEL_B, _cvVal);
        break;
      case 2:
        mcp.setChannelValue(MCP4728_CHANNEL_C, _cvVal);
        break;
      case 3:
        mcp.setChannelValue(MCP4728_CHANNEL_D, _cvVal);
        break;
      default:
        break;
    }
  }
}

void playCV_MIDI_Pitch(uint8_t _track, uint8_t _note)
{
  if(_note>=10 && _note<=70){
    setCV(_track, dac_PitchValues[_note-10]); // dac_values[6] = C4 -> input 16
  }
}




//////////////////////////////////////////////////////////////////////////////

/*
void generateNewSettingsFile()
{
    //construct Filename
    String filename = "settings.txt";
    char number[2] = {'0','0'};

    File f = LittleFS.open(filename, "w");
    if (f) {
      f.write("1.0.3");                           // change to define
      f.write("\n");

      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++){
          number[0] = '0';
          number[1] = '0';
          (String(midiNotes[t][p])).toCharArray(number, 3);   
          f.write(number, strlen(number));
          f.write("\n");
        }
      }
      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++){
          number[0] = '0';
          number[1] = '0';
          (String(cvNotes[t][p])).toCharArray(number, 3);   
          f.write(number, strlen(number));
          f.write("\n");
        }
      }

      number[0] = '0';
      number[1] = '0';
      (String(currentCVMode)).toCharArray(number, 3);   
      f.write(number, strlen(number));
      f.write("\n");
      f.close();
    }
    debugln("Wrote new Settings File ");  
}




void loadSettingsFromMemory()
{
  uint8_t newIntNumber = 0;
  bool fileChanged = false;
  
  File i = LittleFS.open("settings.txt", "r");
  if (i) 
  {
    debugln("Reading settings.txt:");
    bzero(buff, 288);

    if (i.read((uint8_t *)buff, 63))  // if there is enough in mem to fill buffer
    {            
      String newString = strtok(buff, "\n");
      debug("Firmware Version in Memory: ");
      debugln(newString);
      // if not currrent Version generate new Settings File
      debug("loading Midi Notes: ");

      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++)
        {
          newIntNumber = ((String)strtok(NULL, "\n")).toInt();
          if(newIntNumber>=0 && newIntNumber<=127){
            debug("t");debug(t);debug("-");debug("p");debug(p);debug(": ");
            debugln(newIntNumber);
            midiNotes[t][p] = newIntNumber;
          } else {
            debugln("No valid Data for MIDI Note "); debug("t");debug(t);debug("-");debug("p");debug(p);debug(": ");
            midiNotes[t][p] = 40+t*4+p;
            fileChanged = true;
          }
        }
      }

      debug("loading CV Notes: ");

      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++)
        {
          newIntNumber = ((String)strtok(NULL, "\n")).toInt();
          if(newIntNumber>=34 && newIntNumber<=93){   // C4 = 0.5 V = step 6 - 40-C4-6  ->34-F#3-0  -> 34+(5+12)-1 Steps: 93
            debug("t");debug(t);debug("-");debug("p");debug(p);debug(": ");
            debugln(newIntNumber);
            cvNotes[t][p] = newIntNumber;
          } else {
            debugln("No valid Data for CV Note "); debug("t");debug(t);debug("-");debug("p");debug(p);debug(": ");
            cvNotes[t][p] = 40+t*4+p;
            fileChanged = true;
          }
        }
      }

      debug("loading CV Mode: ");

      newIntNumber = ((String)strtok(NULL, "\n")).toInt();
      if(newIntNumber>=0 && newIntNumber<=2){
        debug("Mode: ");
        debugln(newIntNumber);
        currentCVMode = newIntNumber;
      } else {
        debugln("No valid Data for CV MODE "); 
        currentCVMode=0;;
        fileChanged = true;
      }
 
    }
    while (i.available()) {
      Serial.write(i.read());
    }
    debugln("---------------");
    i.close();

    if(fileChanged == true){
      // SAVE CURRENT (corrected) SETTINGS
      debugln("Found invalid Data, Saving changed Settings to file settings.txt");
      saveSettingsToFile();
    }

  } else {
    debugln("settings.txt does not exist, generating a new one");
    // make a new valid Settings.txt file
    generateNewSettingsFile();
    //loadSettingsFromMemory();

  }
}

*/


/*
void saveSettingsToFile()
{
    //construct Filename
    String filename = "settings.txt";
    char number[2] = {'0','0'};

    File f = LittleFS.open(filename, "w");
    if (f) {
      f.write(VERSION_NUMBER);                           // change to define
      f.write("\n");

      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++){
          number[0] = '0';
          number[1] = '0';
          (String(midiNotes[t][p])).toCharArray(number, 3);   
          f.write(number, strlen(number));
          f.write("\n");
        }
      }
      for (int t=0; t<4; t++){
        for (int p=0; p<4; p++){
          number[0] = '0';
          number[1] = '0';
          (String(cvNotes[t][p])).toCharArray(number, 3);   
          f.write(number, strlen(number));
          f.write("\n");
        }
      }

      number[0] = '0';
      number[1] = '0';
      (String(currentCVMode)).toCharArray(number, 3);   
      f.write(number, strlen(number));
      f.write("\n");
      f.close();

      debugln("Wrote  Settings to Memory ");  
    } 
    else {
      debugln("ERROR: could not find settings.txt file");
    }
}

*/









void saveSettings()
{
  int addr = 0;
  int val = 0;

  // Write Version
  addr = 222;
  val = (int)(VERSION_NUMBER);
  EEPROM.write(addr, val);
  addr++;

  // Write MIDI Notes
  for (int t=0; t<4; t++){
    for (int p=0; p<4; p++){
      val = midiNotes[t][p];
      EEPROM.write(addr, val);
      addr++;
    }
  }

  // Write CV Notes
  for (int t=0; t<4; t++){
    for (int p=0; p<4; p++){
      val = cvNotes[t][p];
      EEPROM.write(addr, val);
      addr++;
    }
  }

  // Write CV Mode
  val = currentCVMode;
  EEPROM.write(addr, val);
  addr++;

  // write Motor Settings
  val = motorMinSpeed;
  EEPROM.write(addr, val);
  addr++;

  val = motorMaxSpeed;
  EEPROM.write(addr, val);
  addr++;
  
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }

  debugln("Wrote new Settings File ");  
}




void loadSettings()
{
  int addr = 222;
  int val = EEPROM.read(addr);
  addr++;

  if(val == VERSION_NUMBER){
    debug("Memory contains current Version number: ");
    debugln(val);

    // Read MIDI Notes
    debug("Midi Notes: ");
    for (int t=0; t<4; t++){
      for (int p=0; p<4; p++){
        val = EEPROM.read(addr);
        if(val>=0 && val<=127) {
          midiNotes[t][p] = val;
          debug(val); debug(" ");
        }
        addr++;
      }
    }
    debugln("");

    // Read CV Notes
    debug("CV Notes: ");
    for (int t=0; t<4; t++){
      for (int p=0; p<4; p++){
        val = EEPROM.read(addr);
        if(val>=10 && val<=70) {
          cvNotes[t][p] = val;
          debug(val); debug(" ");
        }
        addr++;
      }
    }
    debugln("");

    // Read CV Mode
    val = currentCVMode;
    val = EEPROM.read(addr);
    if(val>=0 && val<=127) {
      currentCVMode = val;
      debug("Current CV Mode: "); debugln(val);

    }
    addr++;

    // read Motor Settings
    val =  EEPROM.read(addr);
    if(val>= 50 && val<=130){
      motorMinSpeed = val;
      debug("motorMinSpeed: "); debugln(val);
    }
    addr++;

    val =  EEPROM.read(addr);
    if(val>= 130 && val<=255){
      motorMaxSpeed = val;
      debug("motorMaxSpeed: "); debugln(val);
    }
    addr++;




  } else {
    debug("Memory doesnt contain currention Version Number: ");
    debug(val);
    debugln(" , writing New Memory");
    saveSettings();
  }
}
