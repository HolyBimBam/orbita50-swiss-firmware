# orbita50-swiss-firmware
Orbita50 - Swiss Edition - Arduino Firmware

![Product Render](/Hardware_Setup/orbita_50_swiss_render9.png?raw=true)


## HARDWARE

###Orbita ERIN Mainboard 

###4 Pixel Sensor Boards with VEML6040 Ambient Light Sensor & 3 Neopixel per Board
- PIXEL1_ADDR 0x71  *outer circle
- PIXEL2_ADDR 0x72  
- PIXEL3_ADDR 0x73  
- PIXEL4_ADDR 0x74  

###Adafruit Stemma Encoder Module 
- ENCODER_ADDR  0x36

###Adafruit Stemma 7 Segment Display
- DISPLAY_ADDR 0x70 

###Custom 4 Button + Neopixel Board based on TCA9
- TCA_ADDR 0x27

###Custom Modified CV & Gate Board
- MCP_ADDR 0x60
- Gate Signals direct via ERIN Pins 27 / 1 / 23 / 25


## I/0

- USB C Connector for Power & MIDI
- Analog MIDI via 3.5mm TRS Type A Connector (on ERIN)
- 4x CV & 4x Gate via 3.5mm TS Connector (on Expander)




## ARDUINO SETTINGS

- Board: Generic RP4020 (change Serial1 Pins in variants.h File for now)
- USB: Adafruit Tiny USB