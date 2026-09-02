/*
 *  Periodic device telemetry over MQTT
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_task.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "platform_config.h"
#include "monitor.h"
#include "mqtt_svc.h"
#include "esp_netif.h"
#include "network_ethernet.h"
#include "network_manager.h"
#include "network_wifi.h"
#include "bt_headphone.h"
#include "buttons.h"
#include "rfid.h"
#include "telemetry.h"

static const char *TAG = "telemetry";

static void publish_state(void);

#define TELEMETRY_STACK_SIZE	4096
#define DEFAULT_INTERVAL_S		60
#define STATE_TOPIC				"state"

// how often the switch is looked at, which is what makes "immediately" mean anything
#define TICK_MS					250

// one announcement is the largest thing we build, the state object stays well below it
#define PAYLOAD_LEN				640
#define TOPIC_LEN				160

static EXT_RAM_ATTR struct {
	char device_id[64];
	int interval_s;
	bool has_battery;
	int switch_gpio;		// -1 when no switch is configured
	bool switch_invert;
	bool switch_on;
	int channel;			// last published audio channel, -1 when not applicable
	char last_tag[32];		// last published tags, for spotting a change
	char prev_tag[32];
} telemetry;

/*
Every field of the state object becomes one sensor in Home Assistant. Giving them a
device class is what makes the difference between a number and something the interface
knows how to draw, and marking them diagnostic keeps them out of the main device card.
*/
static const struct {
	const char *key;
	const char *name;
	const char *unit;
	const char *device_class;
	bool measurement;
} fields[] = {
	{ "rssi",     "Signal",           "dBm", "signal_strength", true  },
	{ "bssid",    "Access point",     NULL,  NULL,              false },
	{ "channel",  "WiFi channel",     NULL,  NULL,              false },
	{ "ip",       "IP address",       NULL,  NULL,              false },
	{ "uptime",   "Uptime",           "s",   "duration",        true  },
	{ "reset",    "Reset reason",     NULL,  NULL,              false },
	{ "heap",     "Free memory",      "B",   NULL,              true  },
	{ "psram",    "Free PSRAM",       "B",   NULL,              true  },
	{ "version",  "Firmware",         NULL,  NULL,              false },
};

// battery is only announced when one is actually wired up, see telemetry_svc_init
static const struct {
	const char *key;
	const char *name;
	const char *unit;
	const char *device_class;
} battery_fields[] = {
	{ "battery",  "Battery voltage",  "V",   "voltage" },
	{ "level",    "Battery",          "%",   "battery" },
};

/****************************************************************************************
 * Why we last started, in words rather than in numbers
 */
static const char *reset_reason(void) {
	switch (esp_reset_reason()) {
	case ESP_RST_POWERON:  return "power on";
	case ESP_RST_EXT:      return "external pin";
	case ESP_RST_SW:       return "software";
	case ESP_RST_PANIC:    return "panic";
	case ESP_RST_INT_WDT:  return "interrupt watchdog";
	case ESP_RST_TASK_WDT: return "task watchdog";
	case ESP_RST_WDT:      return "watchdog";
	case ESP_RST_DEEPSLEEP:return "deep sleep";
	case ESP_RST_BROWNOUT: return "brownout";
	case ESP_RST_SDIO:     return "sdio";
	default:               return "unknown";
	}
}

/****************************************************************************************
 * Announce one sensor. Retained, because Home Assistant only reads these when it starts
 * or when they change, and a restarting server has to find them again.
 */
static void announce(const char *key, const char *name, const char *unit,
					 const char *device_class, bool measurement) {
	char topic[TOPIC_LEN], payload[PAYLOAD_LEN], extra[128] = "";
	const char *prefix = mqtt_svc_discovery_prefix();
	const char *base = mqtt_svc_topic_base();
	size_t used = 0;

	if (!prefix || !base) return;

	if (unit) {
		used += snprintf(extra + used, sizeof(extra) - used, ",\"unit_of_measurement\":\"%s\"", unit);
	}
	if (device_class && used < sizeof(extra)) {
		used += snprintf(extra + used, sizeof(extra) - used, ",\"device_class\":\"%s\"", device_class);
	}
	if (measurement && used < sizeof(extra)) {
		snprintf(extra + used, sizeof(extra) - used, ",\"state_class\":\"measurement\"");
	}

	mqtt_svc_format(topic, sizeof(topic), "/%s/sensor/%s_%s/config", prefix,
					telemetry.device_id, key);

	mqtt_svc_format(payload, sizeof(payload),
					"{\"name\":\"%s\",\"state_topic\":\"%s/%s\","
					"\"value_template\":\"{{ value_json.%s }}\","
					"\"unique_id\":\"%s_%s\",\"availability_topic\":\"%s/availability\","
					"\"entity_category\":\"diagnostic\"%s,"
					"\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\","
					"\"manufacturer\":\"squeezelite-esp32\",\"model\":\"SqueezeESP32\"}}",
					name, base, STATE_TOPIC, key, telemetry.device_id, key, base, extra,
					telemetry.device_id, telemetry.device_id);

	mqtt_svc_publish(topic, payload, 1, true);
}

/****************************************************************************************
 * The switch is a state, not a measurement, so it gets a binary sensor rather than the
 * sensor the others use - which means its own announcement rather than a flag on theirs.
 */
static void announce_switch(void) {
	char topic[TOPIC_LEN], payload[PAYLOAD_LEN];
	const char *prefix = mqtt_svc_discovery_prefix();
	const char *base = mqtt_svc_topic_base();

	if (!prefix || !base) return;

	mqtt_svc_format(topic, sizeof(topic), "/%s/binary_sensor/%s_switch/config", prefix,
					telemetry.device_id);

	mqtt_svc_format(payload, sizeof(payload),
					"{\"name\":\"Play switch\",\"state_topic\":\"%s/%s\","
					"\"value_template\":\"{{ value_json.switch }}\","
					"\"payload_on\":\"on\",\"payload_off\":\"off\","
					"\"unique_id\":\"%s_switch\",\"availability_topic\":\"%s/availability\","
					"\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\","
					"\"manufacturer\":\"squeezelite-esp32\",\"model\":\"SqueezeESP32\"}}",
					base, STATE_TOPIC, telemetry.device_id, base,
					telemetry.device_id, telemetry.device_id);

	mqtt_svc_publish(topic, payload, 1, true);
}

/****************************************************************************************
 * Called on every connect: the announcements are retained on the broker, but a broker
 * that lost its session has to be told again.
 */
static void publish_discovery(void) {
	if (!mqtt_svc_discovery_prefix()) return;

	for (int i = 0; i < sizeof(fields) / sizeof(*fields); i++) {
		announce(fields[i].key, fields[i].name, fields[i].unit,
				 fields[i].device_class, fields[i].measurement);
	}

	if (telemetry.has_battery) {
		for (int i = 0; i < sizeof(battery_fields) / sizeof(*battery_fields); i++) {
			announce(battery_fields[i].key, battery_fields[i].name, battery_fields[i].unit,
					 battery_fields[i].device_class, true);
		}
	}

	if (telemetry.switch_gpio >= 0) announce_switch();

	// not "channel": that key is the wifi channel, and has been since the first version
	if (bt_headphone_channel() >= 0) announce("output", "Audio output", NULL, NULL, false);

	/*
	The scan stays an event - retained, it would replay as a fresh scan every time Home
	Assistant restarts. These two are the state that the event cannot answer: what is on
	the reader now, and what was on it before.
	*/
	if (rfid_last_uid()) {
		announce("tag", "Last tag", NULL, NULL, false);
		announce("tag_prev", "Previous tag", NULL, NULL, false);
	}

	ESP_LOGI(TAG, "announced %d sensors to Home Assistant",
			 (int) (sizeof(fields) / sizeof(*fields) +
					(telemetry.has_battery ? sizeof(battery_fields) / sizeof(*battery_fields) : 0)));

	// send values straight away, otherwise the sensors sit at unknown for a whole interval
	publish_state();
}

/****************************************************************************************
 * One JSON object with everything, so Home Assistant needs a single message for all of it
 */
static void publish_state(void) {
	char payload[PAYLOAD_LEN], bssid[18] = "", battery[64] = "", sw[24] = "";
	char chan[24] = "", tags[96] = "";
	char ip[24] = "";
	wifi_ap_record_t ap;
	int rssi = 0, channel = 0;

	if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
		rssi = ap.rssi;
		channel = ap.primary;
		snprintf(bssid, sizeof(bssid), "%02x:%02x:%02x:%02x:%02x:%02x",
				 ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5]);
	}

	/*
	Deliberately not network_status_get_sta_ip_string(): its setter is only ever called
	once, at init, with NULL, so that string stays "0.0.0.0" forever. Ask the interface
	that is actually up instead, wifi first because that is the common case.
	*/
	esp_netif_t *netifs[] = { network_wifi_get_interface(), network_ethernet_get_interface() };

	for (int i = 0; i < 2 && !*ip; i++) {
		esp_netif_ip_info_t info;

		if (netifs[i] && network_is_interface_connected(netifs[i]) &&
			esp_netif_get_ip_info(netifs[i], &info) == ESP_OK) {
			snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.ip));
		}
	}

	if (telemetry.has_battery) {
		mqtt_svc_format(battery, sizeof(battery), ",\"battery\":%.2f,\"level\":%u",
						battery_value_svc(), battery_level_svc());
	}

	if (telemetry.switch_gpio >= 0) {
		mqtt_svc_format(sw, sizeof(sw), ",\"switch\":\"%s\"", telemetry.switch_on ? "on" : "off");
	}

	int out = bt_headphone_channel();
	if (out >= 0) {
		mqtt_svc_format(chan, sizeof(chan), ",\"output\":\"%s\"", out ? "bluetooth" : "dac");
	}

	const char *tag = rfid_last_uid(), *prev = rfid_previous_uid();
	if (tag && prev) {
		mqtt_svc_format(tags, sizeof(tags), ",\"tag\":\"%s\",\"tag_prev\":\"%s\"", tag, prev);
	}

	mqtt_svc_format(payload, sizeof(payload),
					"{\"rssi\":%d,\"bssid\":\"%s\",\"channel\":%d,\"ip\":\"%s\","
					"\"uptime\":%lld,\"reset\":\"%s\",\"heap\":%u,\"psram\":%u,"
					"\"version\":\"%s\"%s%s%s%s}",
					rssi, bssid, channel, ip,
					esp_timer_get_time() / 1000000, reset_reason(),
					(unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
					(unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
					esp_ota_get_app_description()->version, battery, sw, chan, tags);

	// retained, so a restarting Home Assistant sees the last values straight away
	mqtt_svc_publish(STATE_TOPIC, payload, 0, true);

	ESP_LOGD(TAG, "%s", payload);
}

/****************************************************************************************
 *
 */
static void telemetry_task(void *arg) {
	ESP_LOGI(TAG, "publishing telemetry every %d s", telemetry.interval_s);

	int elapsed = 0;

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(TICK_MS));
		elapsed += TICK_MS;

		bool due = elapsed >= telemetry.interval_s * 1000;
		bool changed = false;

		/*
		Looked at rather than subscribed to: the button code keeps this level current for
		us, and reading it here leaves the path that turns the same switch into play/pause
		completely alone. A quarter second is well below what anyone notices.
		*/
		if (telemetry.switch_gpio >= 0) {
			bool on = button_is_pressed(telemetry.switch_gpio, NULL) != telemetry.switch_invert;

			if (on != telemetry.switch_on) {
				telemetry.switch_on = on;
				changed = true;
				ESP_LOGI(TAG, "play switch %s", on ? "on" : "off");
			}
		}

		int out = bt_headphone_channel();
		if (out != telemetry.channel) {
			telemetry.channel = out;
			changed = true;
			ESP_LOGI(TAG, "audio output is now %s", out ? "bluetooth" : "the dac");
		}

		const char *tag = rfid_last_uid();
		if (tag && strcmp(tag, telemetry.last_tag)) {
			strncpy(telemetry.last_tag, tag, sizeof(telemetry.last_tag) - 1);
			changed = true;
		}

		const char *prev = rfid_previous_uid();
		if (prev && strcmp(prev, telemetry.prev_tag)) {
			strncpy(telemetry.prev_tag, prev, sizeof(telemetry.prev_tag) - 1);
			changed = true;
		}

		if (due) elapsed = 0;

		// a change goes out at once, without disturbing the periodic cadence
		if ((due || changed) && mqtt_svc_connected()) publish_state();
	}
}

/****************************************************************************************
 *
 */
void telemetry_svc_init(void) {
	char *config = config_alloc_get_default(NVS_TYPE_STR, "mqtt_interval", "", 0);

	telemetry.interval_s = DEFAULT_INTERVAL_S;
	if (config) {
		if (*config) telemetry.interval_s = atoi(config);
		free(config);
	}

	if (telemetry.interval_s <= 0) {
		ESP_LOGI(TAG, "telemetry disabled");
		return;
	}

	// a battery that is not wired up reads zero, and announcing it would be a lie
	telemetry.has_battery = battery_value_svc() > 0.1;

	/*
	Which gpio carries the play switch, empty for none. Not derived from the button
	configuration: that is json, and one number is not worth parsing it for.

	A leading "!" inverts it. Which way round the switch reads depends on how it is
	wired, and that is not something to bake in: what the button code calls pressed is
	whichever level was configured as active, which need not be the position the person
	looking at the entity would call on.
	*/
	telemetry.switch_gpio = -1;
	char *gpio = config_alloc_get_default(NVS_TYPE_STR, "mqtt_switch", "", 0);
	if (gpio) {
		const char *num = gpio;
		if (*num == '!') {
			telemetry.switch_invert = true;
			num++;
		}
		if (*num) telemetry.switch_gpio = atoi(num);
		free(gpio);
	}
	if (telemetry.switch_gpio >= 0) {
		telemetry.switch_on = button_is_pressed(telemetry.switch_gpio, NULL) != telemetry.switch_invert;
	}

	// seeded so the first tick does not report a change that nobody made
	telemetry.channel = bt_headphone_channel();

	char *name = config_alloc_get_str("host_name", NULL, "squeezelite");
	strncpy(telemetry.device_id, name ? name : "squeezelite", sizeof(telemetry.device_id) - 1);
	if (name) free(name);
	for (char *p = telemetry.device_id; *p; p++) {
		if (*p == ' ' || *p == '/' || *p == '+' || *p == '#') *p = '_';
	}

	mqtt_svc_set_connect_hook(publish_discovery);

	static DRAM_ATTR StaticTask_t task_buffer __attribute__ ((aligned (4)));
	static EXT_RAM_ATTR StackType_t task_stack[TELEMETRY_STACK_SIZE] __attribute__ ((aligned (4)));

	xTaskCreateStatic(telemetry_task, "telemetry", TELEMETRY_STACK_SIZE, NULL,
					  ESP_TASK_PRIO_MIN + 1, task_stack, &task_buffer);

	if (telemetry.switch_gpio >= 0) {
		ESP_LOGI(TAG, "telemetry every %d s, battery %s, play switch on gpio %d%s",
				 telemetry.interval_s, telemetry.has_battery ? "included" : "not configured",
				 telemetry.switch_gpio, telemetry.switch_invert ? " inverted" : "");
	} else {
		ESP_LOGI(TAG, "telemetry every %d s, battery %s, no play switch",
				 telemetry.interval_s, telemetry.has_battery ? "included" : "not configured");
	}
}
