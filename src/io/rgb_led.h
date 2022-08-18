#pragma once
void rgb_led_lvc();
#define RGB(r, g, b) ((((int)g & 0xff) << 16) | (((int)r & 0xff) << 8) | ((int)b & 0xff))
#define RGB5BIT(r, g, b) ((((int)(g>>3) & 0x1f) << 10) | (((int)(r>>3) & 0x1f) << 5) | ((int)(b>>3) & 0x1f))