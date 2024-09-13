# orbita50-swiss-firmware
Orbita50 - Swiss Edition - Arduino Firmware




# HARDWARE

- Orbita ERIN Mainboard 

- 4 Pixel Sensor Boards with VEML6040 Ambient Light Sensor
PIXEL1_ADDR 0x71
PIXEL2_ADDR 0x72
PIXEL3_ADDR 0x73
PIXEL4_ADDR 0x74

- Adafruit Stemma Encoder Module 
ENCODER_ADDR  0x36

- Adafruit Stemma 7 Segment Display
DISPLAY_ADDR 0x70 


- Custom 4 Button + Neopixel Board based on TCA9
TCA_ADDR 0x27

- Custom Modified CV & Gate Board
MCP_ADDR 0x60





PIXEL JUMPER SETTINGS:


Button Board 

- JUMPER
- i2c Address




# ARDUINO SETTINGS

- Board: Generic RP4020 (change Serial1 Pins in variants.h File for now)
- USB: Adafruit Tiny USB