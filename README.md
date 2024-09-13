# orbita50-swiss-firmware
Orbita50 - Swiss Edition - Arduino Firmware

[![Product Render][product-screenshot]](/Hardware_Setup/orbita_50_swiss_render9.png?raw=true)


# HARDWARE

Orbita ERIN Mainboard 

4 Pixel Sensor Boards with VEML6040 Ambient Light Sensor & 3 Neopixel per Board
- PIXEL1_ADDR 0x71 (Jumper: )	*outer circle
- PIXEL2_ADDR 0x72 (Jumper: )
- PIXEL3_ADDR 0x73 (Jumper: )
- PIXEL4_ADDR 0x74 (Jumper: )	*inner circle
PIXEL JUMPER SETTINGS:

Adafruit Stemma Encoder Module 
- ENCODER_ADDR  0x36

Adafruit Stemma 7 Segment Display
- DISPLAY_ADDR 0x70 

Custom 4 Button + Neopixel Board based on TCA9
- TCA_ADDR 0x27

Custom Modified CV & Gate Board
- MCP_ADDR 0x60
- Gate Signals direct via ERIN Pins 27 / 1 / 23 / 25







Button Board 

- JUMPER
- i2c Address




# ARDUINO SETTINGS

- Board: Generic RP4020 (change Serial1 Pins in variants.h File for now)
- USB: Adafruit Tiny USB