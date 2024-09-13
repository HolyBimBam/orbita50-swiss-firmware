#include <Adafruit_NeoPixel.h>


#define NEOPIXEL_PIN        21 
#define NUMPIXELS 4 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  pixels.clear(); // Set all pixel colors to 'off'

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, pixels.Color(200, 110, 20));
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(500); // Pause before next pass through loop
  }
}
