/*
 *  Send audio to a bluetooth headset when it is there, to the dac when it is not
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task.h"
#include "esp_timer.h"
#include "platform_config.h"
#include "bt_app_core.h"
#include "bt_headphone.h"

static const char *TAG = "bt_headphone";

extern void hal_bluetooth_init(const char *options);

#define HEADPHONE_STACK_SIZE	3072
#define POLL_MS					500

// long enough that a headset losing its link for a moment does not cost a restart
#define SETTLE_MS				4000

// a headset that was there when we rebooted into bluetooth has this long to come back
#define REJOIN_MS				60000

// nothing is allowed to restart the player before it has finished starting
#define GRACE_MS				20000

#define OUTPUT_I2S				"i2s"
#define OUTPUT_BT				"BT"

static EXT_RAM_ATTR struct {
	char name[64];
	bool bt_mode;			// what the running squeezelite was started with
	bool connected;			// what the headset is doing now
	bool seen;				// whether it has been here at all since we started
	int64_t wanted_since;	// when connected last disagreed with bt_mode, 0 when it agrees
	int64_t started;
} headphone;

/****************************************************************************************
 * Locate the value of the "-o" argument. Shared so that reading the current output and
 * rewriting it cannot disagree about what counts as the argument.
 */
static const char *find_output(const char *cmd, const char **rest) {
	const char *opt = cmd;

	// "-o" has to be an argument of its own, not the tail of something else
	while ((opt = strstr(opt, "-o")) != NULL) {
		bool starts = (opt == cmd) || isspace((unsigned char) opt[-1]);
		if (starts && isspace((unsigned char) opt[2])) break;
		opt += 2;
	}

	if (!opt) return NULL;

	const char *value = opt + 2;
	while (isspace((unsigned char) *value)) value++;

	const char *end = value;
	while (*end && !isspace((unsigned char) *end)) end++;

	if (rest) *rest = end;
	return value;
}

/****************************************************************************************
 * Replace the value of the "-o" argument, leaving the rest of the command line alone.
 * Returns a new string, or NULL when there is no "-o" to replace.
 */
static char *rewrite_output(const char *cmd, const char *device) {
	const char *rest = NULL;
	const char *value = find_output(cmd, &rest);

	if (!value) return NULL;

	size_t head = value - cmd, len = strlen(device);
	char *out = malloc(head + len + strlen(rest) + 1);

	// built by hand rather than with snprintf: the length is computed, and this project
	// compiles with -Werror=format-truncation, which cannot see that it fits
	if (out) {
		memcpy(out, cmd, head);
		memcpy(out + head, device, len);
		memcpy(out + head + len, rest, strlen(rest) + 1);
	}

	return out;
}

/****************************************************************************************
 * Change the output squeezelite will use, and restart into it. There is no way to do
 * this while it runs: the backend is chosen once at startup and squeezelite refuses to
 * be started a second time.
 */
static void switch_output(bool to_bt) {
	char *cmd = config_alloc_get_default(NVS_TYPE_STR, "autoexec1", "", 0);
	char *changed = cmd ? rewrite_output(cmd, to_bt ? OUTPUT_BT : OUTPUT_I2S) : NULL;

	if (!changed) {
		ESP_LOGE(TAG, "no -o argument in autoexec1, leaving the output alone");
		if (cmd) free(cmd);
		return;
	}

	ESP_LOGW(TAG, "headset %s, restarting on %s", to_bt ? "connected" : "gone",
			 to_bt ? "bluetooth" : "the dac");
	ESP_LOGI(TAG, "%s", changed);

	esp_err_t err = config_set_value(NVS_TYPE_STR, "autoexec1", changed);

	free(cmd);
	free(changed);

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "could not store the new command line: %s", esp_err_to_name(err));
		return;
	}

	// the value is only in the cache until this returns, and a restart would lose it
	wait_for_commit();
	esp_restart();
}

/****************************************************************************************
 *
 */
static void headphone_task(void *arg) {
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(POLL_MS));

		/*
		Asked for rather than pushed to us: a callback registered at startup would miss a
		headset that connected before we got there, and we would then wrongly conclude
		that it never came back.
		*/
		bool connected = bt_app_source_get_a2d_state() == APP_AV_STATE_CONNECTED;

		if (connected != headphone.connected) {
			headphone.connected = connected;
			ESP_LOGI(TAG, "headset %s", connected ? "connected" : "disconnected");
		}
		if (connected) headphone.seen = true;

		int64_t now = esp_timer_get_time() / 1000;
		if (now - headphone.started < GRACE_MS) continue;

		/*
		Started on bluetooth and the headset has not turned up yet - it was switched off
		while we rebooted, or is still finding its way back. Give it the whole rejoin
		window before falling back to the dac, and in particular do not let the settling
		rule below cut that short: a headset absent since boot is not the same thing as
		one that has just walked away.
		*/
		if (headphone.bt_mode && !headphone.seen) {
			if (now - headphone.started > REJOIN_MS) {
				ESP_LOGW(TAG, "no headset within %d s, falling back to the dac", REJOIN_MS / 1000);
				switch_output(false);
			}
			continue;
		}

		if (headphone.connected == headphone.bt_mode) {
			headphone.wanted_since = 0;
			continue;
		}

		// disagreement has to hold still for a while before it costs a restart
		if (!headphone.wanted_since) {
			headphone.wanted_since = now;
		} else if (now - headphone.wanted_since >= SETTLE_MS) {
			switch_output(headphone.connected);
		}
	}
}

/****************************************************************************************
 *
 */
void bt_headphone_svc_init(void) {
	char *name = config_alloc_get_default(NVS_TYPE_STR, "bt_headphone", "", 0);

	if (!name || !*name) {
		ESP_LOGI(TAG, "no headset configured");
		if (name) free(name);
		return;
	}

	strncpy(headphone.name, name, sizeof(headphone.name) - 1);
	free(name);

	char *cmd = config_alloc_get_default(NVS_TYPE_STR, "autoexec1", "", 0);
	const char *rest = NULL;
	const char *device = cmd ? find_output(cmd, &rest) : NULL;

	if (!device) {
		ESP_LOGE(TAG, "no -o argument in autoexec1, cannot follow a headset");
		if (cmd) free(cmd);
		return;
	}

	// the same test output_embedded.c makes when it picks the backend
	headphone.bt_mode = strncasecmp(device, "BT", 2) == 0 && rest == device + 2;
	free(cmd);

	headphone.started = esp_timer_get_time() / 1000;

	/*
	The name travels through nvs rather than through the option string. hal_bluetooth_init
	falls back to a2dp_sink_name when it is given no -n, which keeps the "-o" argument a
	single word - and a single word is what the rewriting above can safely replace.
	*/
	char *sink = config_alloc_get_default(NVS_TYPE_STR, "a2dp_sink_name", "", 0);
	if (!sink || strcmp(sink, headphone.name)) {
		config_set_value(NVS_TYPE_STR, "a2dp_sink_name", headphone.name);
		ESP_LOGI(TAG, "a2dp_sink_name set to '%s'", headphone.name);
	}
	if (sink) free(sink);

	/*
	On bluetooth, output_init_bt brings the stack up for us and keeps its discovery, so a
	headset in pairing mode can still be found the first time. On the dac we bring it up
	ourselves, connectable but silent - see the header for why we do not scan.
	*/
	if (!headphone.bt_mode) {
		bt_app_source_set_auto_discover(false);

		// "BT" stands in for the program name the option parser expects, nothing more
		hal_bluetooth_init("BT");
	}

	static DRAM_ATTR StaticTask_t task_buffer __attribute__ ((aligned (4)));
	static EXT_RAM_ATTR StackType_t task_stack[HEADPHONE_STACK_SIZE] __attribute__ ((aligned (4)));

	xTaskCreateStatic(headphone_task, "bt_headphone", HEADPHONE_STACK_SIZE, NULL,
					  ESP_TASK_PRIO_MIN + 1, task_stack, &task_buffer);

	ESP_LOGI(TAG, "watching for '%s', currently on %s", headphone.name,
			 headphone.bt_mode ? "bluetooth" : "the dac");
}
