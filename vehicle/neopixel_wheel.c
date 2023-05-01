#include "neopixel_wheel.h"
#include "vehicle_pin_layout.h"

#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
 // Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

static Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, VEHICLE_PIN_NEOPIXEL_LEFT_PIN, NEO_GRB + NEO_KHZ800);

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
static uint32_t wheel(byte wheel_pos) {
  wheel_pos = 255 - wheel_pos;
  if(wheel_pos < 85) {
    return strip.Color(255 - wheel_pos * 3, 0, wheel_pos * 3);
  }
  if(wheel_pos < 170) {
    wheel_pos -= 85;
    return strip.Color(0, wheel_pos * 3, 255 - wheel_pos * 3);
  }
  wheel_pos -= 170;
  return strip.Color(wheel_pos * 3, 255 - wheel_pos * 3, 0);
}



void 
neopixel_wheel_animation_setup()
{
  strip.begin();
  
  strip.setBrightness(50);

  // Initialize all pixels to 'off'
  strip.show(); 
}

void 
neopixel_wheel_animation_tick()
{

   static uint16_t i = 0;
   static uint16_t j = 0;

   for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, wheel((i+j) & 255));
   }
   strip.show();

  j++;
  if (j == 256) {
    j = 0;
  }
}