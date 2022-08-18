#include <math.h>
#include "rgb_led.h"
#include "drv_rgb_led.h"
#include "drv_time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

extern int ledcommand;

// maximum sequence length for LED patterns 14bytes/sequence for 1400bytes total
#define LED_MAX_SEQUENCE_LEN 100
#define LED_MAX_SEQUENCE_COLORS 3
// maximum colors for rainbow wave
#define LED_MAX_WAVE_COLORS 6

// normal flight rgb colour - LED switch OFF
#define RGB_VALUE_INFLIGHT_OFF RGB(0, 0, 0)

//  colour before bind
#define RGB_VALUE_BEFORE_BIND RGB(0, 128, 128)

// fade from one color to another when changed
#define RGB_FILTER_TIME_MICROSECONDS 50e3

// runs the update once every 16 loop times ( 16 mS )
#define DOWNSAMPLE 16
#define RGB_FILTER_TIME FILTERCALC(1000 * DOWNSAMPLE, RGB_FILTER_TIME_MICROSECONDS)

// array with individual led brightnesses
int rgb_led_value[RGB_LED_MAX];

typedef struct {
  uint32_t led_mask1; // 2 bits per led to allow up to 3 colors (and off)
  uint32_t led_mask2; // one bitmask for each group of 16 LEDs
  uint16_t colors[LED_MAX_SEQUENCE_COLORS];   // 5 bit colors is still 32k available colors!
} led_map_t;

typedef struct {
  led_map_t led_map[LED_MAX_SEQUENCE_LEN];
  uint8_t num_steps;
  uint32_t duration;       // duration of each pattern step
  uint8_t pattern_reverse; // does the pattern reverse at the end, or go back to the start
} led_sequence_t;

led_sequence_t led_sequence;
uint32_t current_step = 0;
uint32_t last_micros = 0;
int pattern_rev = 0;

// Variables for rainbow wave - to be overwritten by configuration
uint32_t rainbow_colors[LED_MAX_WAVE_COLORS] = {RGB(255,0,0),RGB(128,128,0),RGB(0,255,0),RGB(0,128,128),RGB(0,0,255),RGB(128,0,128)};
uint8_t rainbow_num_colors = 6;
uint8_t fade_steps = 64;  // steps between each color - more = smoother, slower rainbow / less = faster
uint8_t rainbow_width_between_colors = 6; // how many leds till the next color (0 = all leds in sync)
uint8_t rainbow_reverse = 1;              // reverse direction of wave motion

// Variables for Solid colour setting
uint8_t modifier_channel = 3;
uint32_t solid_color1 = RGB(0,255,0);
uint32_t solid_color2 = RGB(255,0,0);

// loop count for downsampling
int rgb_loopcount = 0;

// rgb low pass filter variables
float r_filt, g_filt, b_filt;

// sets all leds to a brightness
void rgb_led_set_all(int rgb, int fade) {
  if (fade) {
    // deconstruct the colour into components
    int g = rgb >> 16;
    int r = (rgb & 0x0000FF00) >> 8;
    int b = rgb & 0xff;

    // filter individual colors
    lpf(&r_filt, r, RGB_FILTER_TIME);
    lpf(&g_filt, g, RGB_FILTER_TIME);
    lpf(&b_filt, b, RGB_FILTER_TIME);
  //*out = (*out) * coeff + in * (1 - coeff);
    int temp = RGB(r_filt, g_filt, b_filt);

    for (int i = 0; i < RGB_LED_NUMBER; i++)
      rgb_led_value[i] = temp;
  }
  else {
    for (int i = 0; i < RGB_LED_NUMBER; i++)
      rgb_led_value[i] = rgb;
  }
}

// set an individual led brightness
void rgb_led_set_one(int led_number, int rgb) {
  rgb_led_value[led_number] = rgb;
}

// flashes between 2 colours, duty cycle 1 - 15
void rgb_ledflash(int color1, int color2, uint32_t period, int duty) {
  if (time_micros() % period > (period * duty) >> 4) {
    rgb_led_set_all(color1,1);
  } else {
    rgb_led_set_all(color2,1);
  }
}

void rgb_leds_off()
{
  rgb_led_set_all(RGB_VALUE_INFLIGHT_OFF,0);
  current_step = 0;
  last_micros = 0;
  pattern_rev = 0;
}

void rgb_pattern_sequence()
{
  if (time_micros() > last_micros + led_sequence.duration) {
    last_micros = time_micros();
    for(uint16_t j=0;j<RGB_LED_NUMBER;j++) {
      if (j < 16) {
        int temp_color = led_sequence.led_map[current_step].led_mask1 & (3 << (j*2) >> (j*2));
        rgb_led_set_one(j,temp_color);
      }
      else {
        int temp_color = led_sequence.led_map[current_step].led_mask2 & (3 << ((j-16)*2) >> ((j-16)*2));
        rgb_led_set_one(j,temp_color);
      }
    }

    if (pattern_rev){
      if (current_step == 0){
        pattern_rev = !pattern_rev;
        current_step++;
      }
      else {
        current_step--;
      }
    }
    else {
      if (current_step == led_sequence.num_steps - 1) {
        if (led_sequence.pattern_reverse) {
          pattern_rev = !pattern_rev;
          current_step++;
        }
        else {
          current_step = 0;
        }
      }
      else {
        current_step++;
      }
    }
  }
}


void rgb_rainbow()
{
  float factor1, factor2;
  uint16_t ind;
  uint16_t col1,col2,r1,g1,b1,r2,g2,b2;
  for(uint16_t j=0;j<RGB_LED_NUMBER;j++) {
    uint16_t k = j;
    ind = current_step; 
    if (rainbow_width_between_colors){
      if (rainbow_reverse)
        k = (RGB_LED_NUMBER - 1) - j;
      ind += j * fade_steps / rainbow_width_between_colors;
    }
    col1 = (int)((ind % (fade_steps * rainbow_num_colors)) / fade_steps);
    col2 = (col1 + 1) % rainbow_num_colors;
    // deconstruct the colour into components
    g1 = rainbow_colors[col1] >> 16;
    r1 = (rainbow_colors[col1] & 0x0000FF00) >> 8;
    b1 = rainbow_colors[col1] & 0xff;
    g2 = rainbow_colors[col2] >> 16;
    r2 = (rainbow_colors[col2] & 0x0000FF00) >> 8;
    b2 = rainbow_colors[col2] & 0xff;

    factor1 = 1.0 - ((float)(ind % (fade_steps * rainbow_num_colors) - col1 * fade_steps) / fade_steps);
    factor2 = (float)((int)(ind - (col1 * fade_steps)) % (fade_steps * rainbow_num_colors)) / fade_steps;
    rgb_led_set_one(k, RGB((int)(r1 * factor1 + r2 * factor2), (int)(g1 * factor1 + g2 * factor2), (int)(b1 * factor1 + b2 * factor2)));
  }
  current_step++;
  if(current_step > (fade_steps * rainbow_num_colors)) {current_step = 0; }
}


int rgb_fade(int current, int new, float coeff)
{
  // deconstruct the colour into components
  int g1 = current >> 16;
  int r1 = (current & 0x0000FF00) >> 8;
  int b1 = current & 0xff;
  int g2 = new >> 16;
  int r2 = (new & 0x0000FF00) >> 8;
  int b2 = new & 0xff;

  return RGB((r1 + (r2-r1)*coeff),(g1 + (g2-g1)*coeff),(b1 + (b2-b1)*coeff));
}

void rgb_solid_color()
{
  if (solid_color2 != 0) {
    switch (modifier_channel) {
      case 0:
        rgb_led_set_all(rgb_fade(solid_color1, solid_color2,(float)((state.rx_filtered.roll + 1) / 2)),0);
        break;
      case 1:
        rgb_led_set_all(rgb_fade(solid_color1, solid_color2,(float)((state.rx_filtered.pitch + 1) / 2)),0);
        break;
      case 2:
        rgb_led_set_all(rgb_fade(solid_color1, solid_color2,state.rx_filtered.throttle),0);
        break;
      case 3:
        rgb_led_set_all(rgb_fade(solid_color1, solid_color2,(float)((state.rx_filtered.yaw + 1) / 2)),0);
        break;
      default: // one of the aux channels
        rgb_led_set_all(state.aux[modifier_channel] ? solid_color2 : solid_color1,1);
        break;
    }
  }
  else {
    rgb_led_set_all(solid_color1,0);
  }
}


// speed of movement
float KR_SPEED = 0.005f * DOWNSAMPLE;

float kr_position = 0;
int kr_dir = 0;

// knight rider style led movement
void rgb_knight_rider() {
  if (kr_dir) {
    kr_position += KR_SPEED;
    if (kr_position > RGB_LED_NUMBER - 1)
      kr_dir = !kr_dir;
  } else {
    kr_position -= KR_SPEED;
    if (kr_position < 0)
      kr_dir = !kr_dir;
  }

  // calculate led value
  for (int i = 0; i < RGB_LED_NUMBER; i++) {
    float led_bright = fabsf((float)i - kr_position);
    if (led_bright > 1.0f)
      led_bright = 1.0f;
    led_bright = 1.0f - led_bright;

    // set a green background as well, 32 brightness
    rgb_led_set_one(i, RGB((led_bright * 255.0f), (32.0f - led_bright * 32.0f), 0));
  }
}

// 2 led flasher
void rgb_ledflash_twin(int color1, int color2, uint32_t period) {
  if (time_micros() % period > (period / 2)) {
    for (int i = 0; i < RGB_LED_NUMBER; i++) {
      if ((i / 2) * 2 == i)
        rgb_led_set_one(i, color1);
      else
        rgb_led_set_one(i, color2);
    }
  } else {
    for (int i = 0; i < RGB_LED_NUMBER; i++) {
      if ((i / 2) * 2 == i)
        rgb_led_set_one(i, color2);
      else
        rgb_led_set_one(i, color1);
    }
  }
}


void rgb_led_pattern() {
  switch (profile.rgb.active_pattern) {
    case RGB_PATTERN_SOLID:
      rgb_solid_color();
      break;
    case RGB_PATTERN_SEQUENCE:
      rgb_pattern_sequence();
      //rgb_led_set_all(RGB_VALUE_INFLIGHT_ON,1);
      //rgb_ledflash ( RGB( 0 , 0 , 255 ), RGB( 0 , 128 , 0 ), 1000000, 12);
      //rgb_ledflash_twin( RGB( 0 , 0 , 255 ), RGB( 0 , 128 , 0 ), 1000000);
      //rgb_knight_rider();
      break;
    case RGB_PATTERN_RAINBOW:
      rgb_rainbow();
      break;
    default:
      rgb_led_set_all(RGB_VALUE_INFLIGHT_OFF,1);
      break;
  }
}

// main function
void rgb_led_lvc() {
  rgb_loopcount++;
  if (rgb_loopcount > DOWNSAMPLE) {
    rgb_loopcount = 0;
    // led flash logic
    if (flags.lowbatt) {
      rgb_led_pattern();
      //rgb_ledflash(RGB(255, 0, 0), RGB(255, 32, 0), 500000, 8);
    } else {
      if (flags.rx_mode == RXMODE_BIND) {
        // bind mode
        //rgb_led_pattern();
        rgb_led_set_all(RGB_VALUE_BEFORE_BIND,1);
      } else { // non bind
        if (flags.failsafe) {
          // failsafe flash
          rgb_ledflash(RGB(0, 128, 0), RGB(0, 0, 128), 500000, 8);
        } else {
          if (rx_aux_on(AUX_LEDS))
		  	    rgb_led_pattern();
          else
            rgb_leds_off();
        }
      }
    }
#ifdef RGB_LED_DMA
    // send dma start signal
    rgb_send(0);
#else
    // send data to leds
    for (int i = 0; i < RGB_LED_NUMBER; i++) {
      rgb_send(rgb_led_value[i]);
    }
#endif
  }
}

