// ORBITA SWISS EDITION FIRMWARE
//
// written 2024 by STEFFEN SENNERT 
// for PLAYTRONICA
//
// SELECT: RP2040 generic (use variant file)
// ERIN USES WSW25Q16 FLASH with 2MB



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
#include <EEPROM.h>


/////// SETTINGS ///////////////////////////

#define VERSION_NUMBER 3
#define DEBUG 1

//#define VEML_TRIGGER_MODE 1

#define BRIGHTNESS 100
#define BUTTON_COLOR pixels.Color(255, 80, 5)

#define BUTTON_UPDATE_RATE 40
#define MOTOR_UPDATE_RATE 400   // for now to prevent too quick changes

#define MIN_MOTOR_SPEED 145
#define MAX_MOTOR_SPEED 220


#define START_HUE_RED     10 
#define START_HUE_YELLOW  70 
#define START_HUE_BLUE    170 
#define START_HUE_MAGENTA 270 

#define MIDI_NOTE_LENGTH 100

uint8_t midiNotes[4][4] = {{36,37,38,39}, 
                           {40,41,42,43}, 
                           {44,45,46,47}, 
                           {48,49,50,51}};         

uint8_t cvNotes[4][4] = {{16,28,40,52}, 
                         {28,32,35,40}, 
                         {28,31,35,40}, 
                         {28,32,35,37}};


uint16_t colorStartHueValues[4][4] = {{START_HUE_RED, START_HUE_YELLOW, START_HUE_BLUE, START_HUE_MAGENTA}, 
                                      {START_HUE_RED, START_HUE_YELLOW, START_HUE_BLUE, START_HUE_MAGENTA}, 
                                      {START_HUE_RED, START_HUE_YELLOW, START_HUE_BLUE, START_HUE_MAGENTA}, 
                                      {START_HUE_RED, START_HUE_YELLOW, START_HUE_BLUE, START_HUE_MAGENTA}};


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

hsv hsv_color;
rgb rgb_color;


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
char buff[288];
uint8_t motorMinSpeed = MIN_MOTOR_SPEED;
uint8_t motorMaxSpeed = MAX_MOTOR_SPEED;
static int lastMeasuredHue[4] = {50,50,50,50};
long sensorReadOutTime[4] = {-1,-1,-1,-1};
int lastNote[4] = {-1,-1,-1,-1};


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
  digitalWrite(MULTIPLEX_A_PIN, HIGH);  // for MIDI: HIGH
  digitalWrite(MULTIPLEX_B_PIN, LOW);   // for MIDI: LOW
  digitalWrite(MULTIPLEX_INH_PIN, LOW); // LOW to turn Output on


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
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE); 
  }
  pixel1.setChannel(1);

  pixel2.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 2"); }
  else { 
    //if(VEML_TRIGGER_MODE) RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);   
  }
  pixel2.setChannel(1);

  pixel3.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 3"); }
  else { 
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);   
  }
  pixel3.setChannel(1);

  pixel4.setChannel(0);
  if(!RGBWSensor.begin()) { debugln("ERROR: couldn't detect the sensor 4"); }
  else { 
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);   
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

  readHallSensor(0);
  readHallSensor(1);
  readHallSensor(2);
  readHallSensor(3);

  updateColorSensor(0);
  updateColorSensor(1);
  updateColorSensor(2);
  updateColorSensor(3);

  updateEncoder();
  updateButtons();
  updateNoteOffQueue();

  // read any new MIDI messages
  while (MIDI_USB.read()) {}
  while (MIDI.read()) {}
}


void setup1()
{
  setupMotor();
}


void loop1()
{
  updateMotor();
}


////////////////////////////////////////////////////////



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






bool readHallSensor(uint8_t track)
{
  // Check if pin 4 is pulled low (magnet event)
  if (isHallSensorHigh(track)) {
    if(!hallSensorState[track]) { //Sensor state was not High before
      hallSensorState[track] = true;
      
      if(trackIsOn[track]) 
      {
        debug("Magnet T"); debugln(track);

        turnGateOnOff(track, true);

        // SET Color Sensor to Trigger Mode 
        selectColorSensor(track, true);
        RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
        selectColorSensor(track, false);
        sensorReadOutTime[track]=millis()+42;   // delay window of 42ms for measurements
      
      } 
      else {
        /*
        if(lastNote[track] != -1) {
          sendNoteOff(lastNote[track]);
          turnGateOnOff(track, false);
          lastNote[track] = -1;
        }
        */
      }
    }
    return true;
  } else if(hallSensorState[track]){
    hallSensorState[track] = false;
    if(trackIsOn[track]) {

      // Turn Gate Off
      turnGateOnOff(track, false);

      // Send MIDI Note Off
      /*
      if(lastNote[track] != -1) {
        sendNoteOff(lastNote[track]);
        lastNote[track] = -1;
      }
      */

    }
    return false;
  }
  return false;
}











void updateColorSensor(uint8_t track) 
{
  if(sensorReadOutTime[track] == -1)        // we are not in Trigger Mode
  {
    readColorSensor(track);
  } 
  else if(sensorReadOutTime[track] != -1){  // we are in trigger mode and 
    if (sensorReadOutTime[track] <= millis()){
      // we waited long enough, doing measurement now
      sensorReadOutTime[track] = -1;
      readColorSensor(track);

      ////////////////////////////////////
      playNoteOn(track);
      ////////////////////////////////////

      setColorSensorAutoMode(track);
    } else {
      // we wait for 40ms sensor window and do nothing
    }
  } 
}




//////////////////////////////////////////////////////////////////////////////


typedef struct{
    uint8_t pitch;          // MIDI note/pitch
    uint32_t end_time;      // time when note off will be send
    uint8_t channel;        // MIDI Channel?
} NoteStruct_t;



#define MAX_QUEUE_SIZE 25
NoteStruct_t notes[MAX_QUEUE_SIZE];
uint8_t activeNotes = 0;


int isQueueFull() {
  if (activeNotes >= (MAX_QUEUE_SIZE-1)) return 1;
  else return 0;
}

int isQueueEmpty() {
  if (activeNotes == 0) return 1;
  else return 0;
}




void deleteNoteAtIndex(uint8_t _index) 
{
  if (!isQueueEmpty()) 
  {
    // send Note Off
    sendNoteOff(notes[_index].pitch); //, notes[_index].channel);
    activeNotes = activeNotes-1;

    //debug2("%u | ", millis());
    //printf("Note deleted: pitch=%d, end_time=%d, channel=%d, Index:%d, ActiveNotes:%d\n", last_note.pitch, last_note.end_time, last_note.channel, _index, activeNotes);
    notes[_index].end_time = 0;
  }
}




void updateNoteOffQueue() 
{
  if (isQueueEmpty()) {
    //debug("MIDI queue is empty \n");
  } else {
    uint8_t checkedNotes = 0;
    for(int i=0; i<MAX_QUEUE_SIZE; i++){
      if(notes[i].end_time != 0){   // Index is not used
        if (millis() >= (notes[i].end_time)) deleteNoteAtIndex(i);
        checkedNotes++;
        if(checkedNotes == activeNotes) return;
      }
    }
  }
}



void addNoteToMIDIQueue(uint8_t pitch, uint32_t end_time, uint8_t channel=1) 
{
  if (isQueueFull()) {
    debug("MIDI queue is full\n");
  }
  else {

    uint8_t newIndex = returnFreeQueueIndex();
    notes[newIndex].pitch = pitch;
    notes[newIndex].end_time = end_time;
    notes[newIndex].channel = channel;
    activeNotes = activeNotes+1;

    //uint8_t note_on[3] = {0x90 | notes[newIndex].channel, notes[newIndex].pitch, 127};
    //tud_midi_stream_write(cable_num, note_on, 3);

    //uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    //printf("\n%u | ", current_time_ms);
    //printf("Note added: pitch=%d, end_time=%d, channel=%d, to Index:%d, active Notes:%d\n", pitch, end_time, channel, newIndex, activeNotes);
  }
}


int returnFreeQueueIndex()
{
  if(isQueueFull()){
    return -1;
  }
  else if (isQueueEmpty()){
    return 0;
  } 
  else {
    for(int index=0; index<MAX_QUEUE_SIZE; index++)
    {
      if ((notes[index].end_time) == 0) // Index is not in use
      {
        return index;
      }
    }
  }
  return -1;
}



void playNoteOn(uint8_t track)
{
    uint8_t note;
    note = detect_color_fromHue(track);
    sendNoteOn(midiNotes[track][note]);
    addNoteToMIDIQueue(midiNotes[track][note], (millis()+ MIDI_NOTE_LENGTH));
    if(currentCVMode == 0) playCV_Note(track, cvNotes[track][note]);
}