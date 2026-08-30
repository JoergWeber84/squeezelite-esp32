/* 
 *  (c) Philippe G. 2019, philippe_44@outlook.com
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#pragma once

#include "infrared.h"
 
// button type (pressed = LOW or HIGH, matches GPIO level)
#define BUTTON_LOW 		0
#define BUTTON_HIGH		1

typedef enum { BUTTON_PRESSED, BUTTON_RELEASED } button_event_e; 
typedef enum { BUTTON_NORMAL, BUTTON_SHIFTED } button_press_e; 
typedef void (*button_handler)(void *id, button_event_e event, button_press_e mode, bool long_press);

/* 
set debounce to 0 for default (50ms)
set long_press to 0 for no long-press
set shifter_gpio to -1 for no shift
NOTE: shifter buttons *must* be created before shiftee
*/

void button_create(void *client, int gpio, int type, bool pull, int debounce, button_handler handler, int long_press, int shifter_gpio);

/*
Same as button_create, but on one of the esp32's capacitive touch pads. The gpio must be
touch-capable (0, 2, 4, 12, 13, 14, 15, 27, 32 or 33) and touching it always counts as
"pressed", whatever type the other buttons use. Set threshold to 0 to derive it from the
idle value measured at creation, which assumes nobody touches the pad while booting.
Touch pads are polled every 50ms, debounce only sets the long-press timer base.

That derived threshold is a starting point, not a measurement: how far a finger moves
the reading depends on the size of the pad and on what covers it. Use button_touch_report
to see the real numbers and pass a threshold of your own when they do not fit.
*/
void button_create_touch(void *client, int gpio, int threshold, int debounce, button_handler handler, int long_press, int shifter_gpio);

/*
Log what every touch pad reads right now, next to the threshold it is judged against.
The auto-calibrated threshold assumes a finger moves the reading by a third, which not
every pad manages, so this is how you find a value to put in "threshold" by hand.
*/
void button_touch_report(void);

void *button_remap(void *client, int gpio, button_handler handler, int long_press, int shifter_gpio);
void *button_get_client(int gpio);
bool button_is_pressed(int gpio, void *client);

typedef enum { ROTARY_LEFT, ROTARY_RIGHT, ROTARY_PRESSED, ROTARY_RELEASED } rotary_event_e; 
typedef void (*rotary_handler)(void *id, rotary_event_e event, bool long_press);

bool create_rotary(void *id, int A, int B, int SW, int long_press, rotary_handler handler);
bool create_volume_rotary(void *id, int A, int B, int SW, rotary_handler handler);
bool create_infrared(int gpio, infrared_handler handler, infrared_mode_t mode);
