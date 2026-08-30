/* 
 *  a crude button press/long-press/shift management based on GPIO
 *
 *  (c) Philippe G. 2019, philippe_44@outlook.com
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/touch_pad.h"
#include "gpio_exp.h"
#include "buttons.h"
#include "services.h"
#include "rotary_encoder.h"
#include "globdefs.h"

static const char * TAG = "buttons";

static EXT_RAM_ATTR int n_buttons;
static EXT_RAM_ATTR uint32_t buttons_idle_since;

#define BUTTON_STACK_SIZE	4096
#define MAX_BUTTONS			32
#define DEBOUNCE			50
#define BUTTON_QUEUE_LEN	10

static EXT_RAM_ATTR struct button_s {
	void *client;
	int gpio;
	int debounce;
	button_handler handler;
	struct button_s *self, *shifter;
	int shifter_gpio;	// this one is just for post-creation						
	int	long_press;
	bool long_timer, shifted, shifting;
	int type, level;	
	bool touch;
	int touch_channel, touch_threshold;
	TimerHandle_t timer;
} buttons[MAX_BUTTONS];

// can't use EXT_RAM_ATTR for initialized structure
static struct {
	int gpio, level;
	struct button_s *button;
} polled_gpio[] = { {36, -1, NULL}, {39, -1, NULL}, {-1, -1, NULL} };

static TimerHandle_t polled_timer;

#if CONFIG_IDF_TARGET_ESP32
/*
The esp32 touch sensor reads *lower* when a finger loads the pad, so a pad counts as
touched below its threshold. Which channel sits on which GPIO is fixed by the silicon.
*/
#define TOUCH_POLL			50
#define TOUCH_FILTER		10
#define TOUCH_THRESHOLD_PCT	90

static const int touch_gpio[TOUCH_PAD_MAX] = { 4, 0, 2, 15, 13, 12, 14, 27, 33, 32 };
static TimerHandle_t touch_timer;
static bool touch_started;
#endif

static EXT_RAM_ATTR struct encoder {
	QueueHandle_t queue;
	void *client;
	rotary_encoder_info_t info;
	int A, B, SW;
	rotary_handler handler;
} rotary, volume;

static EXT_RAM_ATTR struct {
	RingbufHandle_t rb;
	infrared_handler handler;
} infrared;

static EXT_RAM_ATTR QueueHandle_t button_queue;
static EXT_RAM_ATTR QueueSetHandle_t common_queue_set;

static void buttons_task(void* arg);
static void buttons_handler(struct button_s *button, int level);

/****************************************************************************************
 * Start task needed by button,s rotaty and infrared
 */
static void common_task_init(void) {
	static DRAM_ATTR StaticTask_t xTaskBuffer __attribute__ ((aligned (4)));
	static EXT_RAM_ATTR StackType_t xStack[BUTTON_STACK_SIZE] __attribute__ ((aligned (4)));
	
	if (!common_queue_set) {
		common_queue_set = xQueueCreateSet(BUTTON_QUEUE_LEN + 1);
		xTaskCreateStatic( (TaskFunction_t) buttons_task, "buttons", BUTTON_STACK_SIZE, NULL, ESP_TASK_PRIO_MIN + 2, xStack, &xTaskBuffer);
	}
 }	

/****************************************************************************************
 * GPIO low-level ISR handler
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	struct button_s *button = (struct button_s*) arg;
	BaseType_t woken = pdFALSE;

	if (xTimerGetPeriod(button->timer) > pdMS_TO_TICKS(button->debounce)) {
		if (button->gpio < GPIO_NUM_MAX) xTimerChangePeriodFromISR(button->timer, pdMS_TO_TICKS(button->debounce), &woken); 
		else xTimerChangePeriod(button->timer, pdMS_TO_TICKS(button->debounce), pdMS_TO_TICKS(10)); 
	} else {
		if (button->gpio < GPIO_NUM_MAX) xTimerResetFromISR(button->timer, &woken);
		else xTimerReset(button->timer, portMAX_DELAY);
	}

	if (woken) portYIELD_FROM_ISR();

	ESP_EARLY_LOGD(TAG, "INT gpio %u level %u", button->gpio, button->level);
}

/****************************************************************************************
 * Buttons debounce/longpress timer
 */
static void buttons_timer_handler( TimerHandle_t xTimer ) {
	struct button_s *button = (struct button_s*) pvTimerGetTimerID (xTimer);
	// touch pads are polled, so their level is already up to date
	if (button->touch) buttons_handler(button, button->level);
	// if this is an expanded GPIO, must give cache a chance
	else buttons_handler(button, gpio_exp_get_level(button->gpio, (button->debounce * 3) / 2, NULL));
}

/****************************************************************************************
 * Buttons polling timer
 */
static void buttons_polling( TimerHandle_t xTimer ) {
	for (int i = 0; polled_gpio[i].gpio != -1; i++) {
		if (!polled_gpio[i].button) continue;
		
		int level = gpio_get_level(polled_gpio[i].gpio);
	
		if (level != polled_gpio[i].level) {
			polled_gpio[i].level = level;
			buttons_handler(polled_gpio[i].button, level);
		}	
	}	
}

#if CONFIG_IDF_TARGET_ESP32
/****************************************************************************************
 * Touch pads polling timer
 */
static void touch_polling( TimerHandle_t xTimer ) {
	for (int i = 0; i < n_buttons; i++) {
		uint16_t value;
		int level;

		if (!buttons[i].touch) continue;
		if (touch_pad_read_filtered(buttons[i].touch_channel, &value) != ESP_OK) continue;

		// a loaded pad reads below the threshold and that always means 'pressed'
		level = value < buttons[i].touch_threshold ? buttons[i].type : !buttons[i].type;

		if (level != buttons[i].level) {
			ESP_LOGD(TAG, "touch pad gpio:%u value:%u level:%u", buttons[i].gpio, value, level);
			buttons_handler(buttons + i, level);
		}
	}
}
#endif

/****************************************************************************************
 * Buttons timer handler for press/longpress
 */
static void buttons_handler(struct button_s *button, int level) {
	button->level = level;

	if (button->shifter && button->shifter->type == button->shifter->level) button->shifter->shifting = true;

	if (button->long_press && !button->long_timer && button->level == button->type) {
		// detect a long press, so hold event generation
		ESP_LOGD(TAG, "setting long timer gpio:%u level:%u", button->gpio, button->level);
		xTimerChangePeriod(button->timer, button->long_press / portTICK_RATE_MS, 0);
		button->long_timer = true;
	} else {
		// send a button pressed/released event (content is copied in queue)
		ESP_LOGD(TAG, "sending event for gpio:%u level:%u", button->gpio, button->level);
		// queue will have a copy of button's context
		xQueueSend(button_queue, button, 0);
		button->long_timer = false;
	}
}

/****************************************************************************************
 * Get inactivity callback
 */
static uint32_t buttons_idle_callback(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount()) - buttons_idle_since;
}    

/****************************************************************************************
 * Tasks that calls the appropriate functions when buttons are pressed
 */
static void buttons_task(void* arg) {
	ESP_LOGI(TAG, "starting button tasks");   
    
    buttons_idle_since = pdTICKS_TO_MS(xTaskGetTickCount());    
    services_sleep_setsleeper(buttons_idle_callback);
	
    while (1) {
		QueueSetMemberHandle_t xActivatedMember;
        bool active = true;

		// wait on button, rotary and infrared queues 
		if ((xActivatedMember = xQueueSelectFromSet( common_queue_set, portMAX_DELAY )) == NULL) continue;
        	
		if (xActivatedMember == button_queue) {
			struct button_s button;
			button_event_e event;
			button_press_e press;
			
			// received a button event
			xQueueReceive(button_queue, &button, 0);

			event = (button.level == button.type) ? BUTTON_PRESSED : BUTTON_RELEASED;		

			ESP_LOGD(TAG, "received event:%u from gpio:%u level:%u (timer %u shifting %u)", event, button.gpio, button.level, button.long_timer, button.shifting);

			// find if shifting is activated
			if (button.shifter && button.shifter->type == button.shifter->level) press = BUTTON_SHIFTED;
			else press = BUTTON_NORMAL;
	
			/* 
			long_timer will be set either because we truly have a long press 
			or we have a release before the long press timer elapsed, so two 
			events shall be sent
			*/
			if (button.long_timer) {
				if (event == BUTTON_RELEASED) {
					// early release of a long-press button, send press/release
					if (!button.shifting) {
						button.handler(button.client, BUTTON_PRESSED, press, false);		
						button.handler(button.client, BUTTON_RELEASED, press, false);		
					}
					// button is a copy, so need to go to real context
					button.self->shifting = false;
				} else if (!button.shifting) {
					// normal long press and not shifting so don't discard
					button.handler(button.client, BUTTON_PRESSED, press, true);
				}  
			} else {
				// normal press/release of a button or release of a long-press button
				if (!button.shifting) button.handler(button.client, event, press, button.long_press);
				// button is a copy, so need to go to real context
				button.self->shifting = false;
			}
		} else if (xActivatedMember == rotary.queue) {
			rotary_encoder_event_t event = { 0 };
			
			// received a rotary event
		    xQueueReceive(rotary.queue, &event, 0);

			ESP_LOGD(TAG, "Rotary event: position %d, direction %s", event.state.position,
					event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
			
			rotary.handler(rotary.client, event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? 
											ROTARY_RIGHT : ROTARY_LEFT, false);   
		} else if (xActivatedMember == volume.queue) {
			rotary_encoder_event_t event = { 0 };
			
			// received a volume rotary event
		    xQueueReceive(volume.queue, &event, 0);

			ESP_LOGD(TAG, "Volume event: position %d, direction %s", event.state.position,
					event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
			
			volume.handler(volume.client, event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? 
											ROTARY_RIGHT : ROTARY_LEFT, false);   											
		} else {
			// this is IR
			active = infrared_receive(infrared.rb, infrared.handler);
		}	
        
        // mark the last activity
        if (active) buttons_idle_since = pdTICKS_TO_MS(xTaskGetTickCount());
    }
}	
	
/****************************************************************************************
 * dummy button handler
 */	
void dummy_handler(void *id, button_event_e event, button_press_e press) {
	ESP_LOGW(TAG, "should not be here");
}

/****************************************************************************************
 * Reserve a button slot and wire it to its shifter. This is the part that is common to
 * GPIO and capacitive touch buttons, the caller sets up the pin itself.
 */
static struct button_s *button_register(void *client, int gpio, int type, int debounce, button_handler handler, int long_press, int shifter_gpio) { 
	if (n_buttons >= MAX_BUTTONS) return NULL;

	if (!n_buttons) {
		button_queue = xQueueCreate(BUTTON_QUEUE_LEN, sizeof(struct button_s));
		common_task_init();
		xQueueAddToSet( button_queue, common_queue_set );
	}
	
	// just in case this structure is allocated in a future release
	memset(buttons + n_buttons, 0, sizeof(struct button_s));

	// set mandatory parameters
	buttons[n_buttons].client = client;
 	buttons[n_buttons].gpio = gpio;
 	buttons[n_buttons].debounce = debounce ? debounce: DEBOUNCE;
	buttons[n_buttons].handler = handler;
	buttons[n_buttons].long_press = long_press;
	buttons[n_buttons].shifter_gpio = shifter_gpio;
	buttons[n_buttons].type = type;
	buttons[n_buttons].timer = xTimerCreate("buttonTimer", buttons[n_buttons].debounce / portTICK_RATE_MS, pdFALSE, (void *) &buttons[n_buttons], buttons_timer_handler);
	buttons[n_buttons].self = buttons + n_buttons;

	for (int i = 0; i < n_buttons; i++) {
		// first try to find our shifter
		if (buttons[i].gpio == shifter_gpio) {
			buttons[n_buttons].shifter = buttons + i;
			// a shifter must have a long-press handler
			if (!buttons[i].long_press) buttons[i].long_press = -1;
		}
		// then try to see if we are a non-assigned shifter
		if (buttons[i].shifter_gpio == gpio) {
			buttons[i].shifter = buttons + n_buttons;
			ESP_LOGI(TAG, "post-assigned shifter gpio %u", buttons[i].gpio);			
		}	
	}

	n_buttons++;

	return buttons + n_buttons - 1;
}

/****************************************************************************************
 * Create buttons 
 */
void button_create(void *client, int gpio, int type, bool pull, int debounce, button_handler handler, int long_press, int shifter_gpio) { 
	struct button_s *button = button_register(client, gpio, type, debounce, handler, long_press, shifter_gpio);

	if (!button) return;

	ESP_LOGI(TAG, "Creating button using GPIO %u, type %u, pull-up/down %u, long press %u shifter %d", gpio, type, pull, long_press, shifter_gpio);

	gpio_pad_select_gpio_x(gpio);
	gpio_set_direction_x(gpio, GPIO_MODE_INPUT);

	// do we need pullup or pulldown
	if (pull) {
		if (GPIO_IS_VALID_OUTPUT_GPIO(gpio) || gpio >= GPIO_NUM_MAX) {
			if (type == BUTTON_LOW) gpio_set_pull_mode_x(gpio, GPIO_PULLUP_ONLY);
			else gpio_set_pull_mode_x(gpio, GPIO_PULLDOWN_ONLY);
		} else {	
			ESP_LOGW(TAG, "cannot set pull up/down for gpio %u", gpio);
		}
	}
	
	// and initialize level ...
	button->level = gpio_get_level_x(gpio);
	
	// nasty ESP32 bug: fire-up constantly INT on GPIO 36/39 if ADC1, AMP, Hall used which WiFi does when PS is activated
	for (int i = 0; polled_gpio[i].gpio != -1; i++) if (polled_gpio[i].gpio == gpio) {
		if (!polled_timer) {
			polled_timer = xTimerCreate("buttonsPolling", 100 / portTICK_RATE_MS, pdTRUE, polled_gpio, buttons_polling);		
			xTimerStart(polled_timer, portMAX_DELAY);
		}	
	
		polled_gpio[i].button = button;					
		polled_gpio[i].level = gpio_get_level(gpio);
		ESP_LOGW(TAG, "creating polled gpio %u, level %u", gpio, polled_gpio[i].level);		
	
		gpio = -1;
		break;
	}
	
	// only create ISR if this is not a polled gpio
	if (gpio != -1) {
		// we need any edge detection
		gpio_set_intr_type_x(gpio, GPIO_INTR_ANYEDGE);
		gpio_isr_handler_add_x(gpio, gpio_isr_handler, button);
		gpio_intr_enable_x(gpio);
	}	
}	

/****************************************************************************************
 * Create a button on a capacitive touch pad
 */
void button_create_touch(void *client, int gpio, int threshold, int debounce, button_handler handler, int long_press, int shifter_gpio) {
#if CONFIG_IDF_TARGET_ESP32
	struct button_s *button;
	int channel = -1;

	for (int i = 0; i < TOUCH_PAD_MAX; i++) if (touch_gpio[i] == gpio) channel = i;

	if (channel == -1) {
		ESP_LOGE(TAG, "GPIO %u is not a capacitive touch pad", gpio);
		return;
	}

	// the touch controller and its software filter are shared by all pads
	if (!touch_started) {
		touch_pad_init();
		touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
		touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
		touch_pad_filter_start(TOUCH_FILTER);
		touch_started = true;
	}

	touch_pad_config(channel, 0);

	// give the filter a few rounds before trusting what it reads
	vTaskDelay(pdMS_TO_TICKS(TOUCH_FILTER * 5));

	if (!threshold) {
		uint16_t idle = 0;
		touch_pad_read_filtered(channel, &idle);
		threshold = (idle * TOUCH_THRESHOLD_PCT) / 100;
		ESP_LOGI(TAG, "touch pad GPIO %u idles at %u, threshold set to %d - use the 'touch' command to check it against a real finger", gpio, idle, threshold);
	}

	if ((button = button_register(client, gpio, BUTTON_LOW, debounce, handler, long_press, shifter_gpio)) == NULL) return;

	ESP_LOGI(TAG, "Creating touch button using GPIO %u, threshold %d, long press %u shifter %d", gpio, threshold, long_press, shifter_gpio);

	button->touch = true;
	button->touch_channel = channel;
	button->touch_threshold = threshold;
	button->level = !button->type;

	if (!touch_timer) {
		touch_timer = xTimerCreate("touchPolling", pdMS_TO_TICKS(TOUCH_POLL), pdTRUE, NULL, touch_polling);
		xTimerStart(touch_timer, portMAX_DELAY);
	}
#else
	ESP_LOGE(TAG, "capacitive touch buttons are only supported on the esp32");
#endif
}	

/****************************************************************************************
 * Report what the touch pads read, for setting a threshold by hand
 */
void button_touch_report(void) {
#if CONFIG_IDF_TARGET_ESP32
	bool any = false;

	for (int i = 0; i < n_buttons; i++) {
		uint16_t value = 0;

		if (!buttons[i].touch) continue;
		any = true;

		if (touch_pad_read_filtered(buttons[i].touch_channel, &value) != ESP_OK) {
			ESP_LOGW(TAG, "touch pad GPIO %u cannot be read", buttons[i].gpio);
			continue;
		}

		ESP_LOGI(TAG, "touch pad GPIO %u reads %u, threshold %d -> %s", buttons[i].gpio,
				 value, buttons[i].touch_threshold,
				 value < buttons[i].touch_threshold ? "touched" : "idle");
	}

	if (!any) ESP_LOGI(TAG, "no touch pads configured");
#else
	ESP_LOGI(TAG, "capacitive touch is only supported on the esp32");
#endif
}

/****************************************************************************************
 * Get stored id
 */
void *button_get_client(int gpio) {
	 for (int i = 0; i < n_buttons; i++) {
		 if (buttons[i].gpio == gpio) return buttons[i].client;
	 }
	 return NULL;
}

/****************************************************************************************
 * Get stored id
 */
bool button_is_pressed(int gpio, void *client) {
	for (int i = 0; i < n_buttons; i++) {
		if (gpio != -1 && buttons[i].gpio == gpio) return buttons[i].level == buttons[i].type;
		else if (client && buttons[i].client == client) return buttons[i].level == buttons[i].type;
	}
	return false; 
}

/****************************************************************************************
 * Update buttons 
 */
void *button_remap(void *client, int gpio, button_handler handler, int long_press, int shifter_gpio) { 
	int i;
	struct button_s *button = NULL;
	void *prev_client;
	
	ESP_LOGI(TAG, "remapping GPIO %u, long press %u shifter %u", gpio, long_press, shifter_gpio);

	// find button
	for (i = 0; i < n_buttons; i++) {
		if (buttons[i].gpio == gpio) {
			button = buttons + i;
			break;
		}	
	}	
	
	// don't know what we are doing here
	if (!button) return NULL;	
	
	prev_client = button->client;
	button->client = client;
 	button->handler = handler;
	button->long_press = long_press;
	button->shifter_gpio = shifter_gpio;

	// find our shifter	(if any)	
	for (i = 0; shifter_gpio != -1 && i < n_buttons; i++) {
		if (buttons[i].gpio == shifter_gpio) {
			button->shifter = buttons + i;
			// a shifter must have a long-press handler
			if (!buttons[i].long_press) buttons[i].long_press = -1;
			break;
		}
	}
	
	return prev_client;
}

/****************************************************************************************
 * Create rotary encoder
 */
static bool create_rotary_encoder(struct encoder *encoder, void *id, int A, int B, int SW, int long_press, rotary_handler handler, button_handler button) {
	// nasty ESP32 bug: fire-up constantly INT on GPIO 36/39 if ADC1, AMP, Hall used which WiFi does when PS is activated
	if (A == -1 || B == -1 || A == 36 || A == 39 || B == 36 || B == 39) {
		ESP_LOGI(TAG, "Cannot create rotary %d %d", A, B);
		return false;
	}

	encoder->A = A;
	encoder->B = B;
	encoder->SW = SW;
	encoder->client = id;
	encoder->handler = handler;
	
    // Initialise the rotary encoder device with the GPIOs for A and B signals
    rotary_encoder_init(&encoder->info, A, B);
		
    // Create a queue for events from the rotary encoder driver.
    encoder->queue = rotary_encoder_create_queue();
    rotary_encoder_set_queue(&encoder->info, encoder->queue);
	
	common_task_init();
	xQueueAddToSet( encoder->queue, common_queue_set );

	// create companion button if rotary has a switch
	if (SW != -1) button_create(id, SW, BUTTON_LOW, true, 0, button, long_press, -1);
	
	return true;
}

/****************************************************************************************
 * Volume button encoder handler
 */
static void volume_button_handler(void *id, button_event_e event, button_press_e mode, bool long_press) {
	ESP_LOGI(TAG, "Volume encoder push-button %d", event);
	volume.handler(id, event == BUTTON_PRESSED ? ROTARY_PRESSED : ROTARY_RELEASED, long_press);
}	

/****************************************************************************************
 * Create volume encoder
 */
bool create_volume_rotary(void *id, int A, int B, int SW, rotary_handler handler) {
	ESP_LOGI(TAG, "Created volume encoder A:%d B:%d, SW:%d", A, B, SW);
	return create_rotary_encoder(&volume, id, A, B, SW, false, handler, volume_button_handler);
}	

/****************************************************************************************
 * Rotary button encoder handler
 */
static void rotary_button_handler(void *id, button_event_e event, button_press_e mode, bool long_press) {
	ESP_LOGI(TAG, "Rotary push-button %d", event);
	rotary.handler(id, event == BUTTON_PRESSED ? ROTARY_PRESSED : ROTARY_RELEASED, long_press);
}

/****************************************************************************************
 * Create rotary encoder
 */
bool create_rotary(void *id, int A, int B, int SW, int long_press, rotary_handler handler) {
	ESP_LOGI(TAG, "Created rotary encoder A:%d B:%d, SW:%d", A, B, SW);
	return create_rotary_encoder(&rotary, id, A, B, SW, long_press, handler, rotary_button_handler);
}	

/****************************************************************************************
 * Create Infrared
 */
bool create_infrared(int gpio, infrared_handler handler, infrared_mode_t mode) {
	// initialize IR infrastructure
	infrared_init(&infrared.rb, gpio, mode);
	infrared.handler = handler;
	
	// join the queue set
	common_task_init();
	xRingbufferAddToQueueSetRead(infrared.rb, common_queue_set);
	
	ESP_LOGI(TAG, "Created infrared receiver using GPIO %u", gpio);	
	
	return (infrared.rb != NULL);
}	
