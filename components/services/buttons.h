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
"pressed", whatever type the other buttons use. Touch pads are polled every 50ms,
debounce only sets the long-press timer base.

A pad is judged against where it has been sitting, not against a fixed level: the idle
reading wanders with temperature, humidity and anything that changes the stray
capacitance, so it is followed and a touch is a dip below it. Nothing needs calibrating
for that to keep working.

A dip is not taken at face value. It has to last 200ms before it counts, which no noise
seen here does, and only the pad that dips furthest may be pressed - the others are
ignored for as long as they stay down, so crosstalk into a neighbouring pad cannot press
two buttons at once.

"delta" is how far below the idle level counts as a finger, in raw counts, 0 for the
default. It only wants changing when a pad is unusually insensitive - use
button_touch_report or the touch debug topic to see what a real finger produces. Values
above 100 are rejected with a warning: those are absolute levels from when the threshold
was fixed, and they no longer mean anything.

Assumes these are tap buttons. A pad held down for minutes is eventually absorbed into
the idle level and then reads as released.
*/
void button_create_touch(void *client, int gpio, int delta, int debounce, button_handler handler, int long_press, int shifter_gpio);

/*
Log what every touch pad reads right now, next to the idle level it is judged against.
*/
void button_touch_report(void);

/*
The same numbers, for publishing somewhere they can be watched without a console - which
matters, because reading a pad while touching it is awkward with one pair of hands. Fills
up to "max" entries and returns how many. min and max span the time since the previous
call and are reset by it, so both the noise band and a dip too brief to catch otherwise
become visible.
*/
struct button_touch_s {
	int gpio;
	int value;			// what the pad reads now
	int baseline;		// what it has been idling at
	int delta;			// how far below the baseline counts as touched
	int min, max;		// of value, since the previous probe
	bool touched;		// the state reported, after the persistence and the single winner
	int presses;		// accepted since boot: a dip that did not raise this was rejected,
					// which is otherwise indistinguishable from one this probe just missed
};

int button_touch_probe(struct button_touch_s *out, int max);

void *button_remap(void *client, int gpio, button_handler handler, int long_press, int shifter_gpio);
void *button_get_client(int gpio);
bool button_is_pressed(int gpio, void *client);

typedef enum { ROTARY_LEFT, ROTARY_RIGHT, ROTARY_PRESSED, ROTARY_RELEASED } rotary_event_e; 
typedef void (*rotary_handler)(void *id, rotary_event_e event, bool long_press);

bool create_rotary(void *id, int A, int B, int SW, int long_press, rotary_handler handler);
bool create_volume_rotary(void *id, int A, int B, int SW, rotary_handler handler);
bool create_infrared(int gpio, infrared_handler handler, infrared_mode_t mode);
