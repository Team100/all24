#include "Adafruit_NeoKey_1x4.h"
#include "seesaw_neopixel.h"

Adafruit_NeoKey_1x4 neokey;

void setup() {
  if (! neokey.begin(0x30)) {     // begin with I2C address, default is 0x30
    while(1) delay(10);
  }
  
  // Pulse all the LEDs on to show we're working
  for (uint16_t i=0; i<neokey.pixels.numPixels(); i++) {
    neokey.pixels.setPixelColor(i, 0xffffff);
    neokey.pixels.show();
    delay(50);
  }
  for (uint16_t i=0; i<neokey.pixels.numPixels(); i++) {
    neokey.pixels.setPixelColor(i, 0x000000);
    neokey.pixels.show();
    delay(50);
  }
}

void loop() {
  uint8_t buttons = neokey.read();
  
  // going for woodside orange here, ff0f00 is pretty good.
  if (buttons & (1<<0)) {
    neokey.pixels.setPixelColor(0, 0xff0f00);
  } else {
    neokey.pixels.setPixelColor(0, 0);
  }

  if (buttons & (1<<1)) {
    neokey.pixels.setPixelColor(1, 0xff0f00);
  } else {
    neokey.pixels.setPixelColor(1, 0);
  }
  
  if (buttons & (1<<2)) {
    neokey.pixels.setPixelColor(2, 0xff0f00);
  } else {
    neokey.pixels.setPixelColor(2, 0);
  }

  if (buttons & (1<<3)) {
    neokey.pixels.setPixelColor(3, 0xff0000); // like for "stop"
  } else {
    neokey.pixels.setPixelColor(3, 0);
  }  

  neokey.pixels.show();
}
