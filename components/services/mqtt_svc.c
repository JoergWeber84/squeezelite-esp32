/*
 *  MQTT client service - publishes device events to a broker (Home Assistant & co)
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "platform_config.h"
#include "mqtt_svc.h"

static const char *TAG = "mqtt";

#define MAX_CONNECT_HOOKS	4
#define TOPIC_LEN			128
#define VALUE_LEN			96

static EXT_RAM_ATTR struct {
	esp_mqtt_client_handle_t client;
	char base[TOPIC_LEN];
	char availability[TOPIC_LEN + 16];
	char discovery[32];
	bool connected;
	void (*connect_hook[MAX_CONNECT_HOOKS])(void);
} mqtt_context;

/****************************************************************************************
 * Topics are assembled from strings of unknown length, which snprintf cannot be talked
 * out of warning about. Going through vsnprintf keeps the truncation check ours.
 */
static bool format_topic(char *dst, size_t size, const char *fmt, ...) {
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(dst, size, fmt, args);
	va_end(args);

	if (len < 0 || (size_t) len >= size) {
		ESP_LOGW(TAG, "topic does not fit, truncated to %s", dst);
		return false;
	}

	return true;
}

/****************************************************************************************
 * Topics are used verbatim by brokers and Home Assistant, so keep them boring
 */
static void sanitize_topic(char *topic) {
	for (char *p = topic; *p; p++) {
		bool ok = (*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z') ||
				  (*p >= '0' && *p <= '9') || *p == '_' || *p == '-' || *p == '/';
		if (!ok) *p = '_';
	}
}

/****************************************************************************************
 *
 */
static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
	esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) data;

	switch ((esp_mqtt_event_id_t) id) {
	case MQTT_EVENT_CONNECTED:
		mqtt_context.connected = true;
		ESP_LOGI(TAG, "connected to broker, base topic %s", mqtt_context.base);
		esp_mqtt_client_publish(event->client, mqtt_context.availability, "online", 0, 1, true);
		for (int i = 0; i < MAX_CONNECT_HOOKS && mqtt_context.connect_hook[i]; i++) {
			mqtt_context.connect_hook[i]();
		}
		break;
	case MQTT_EVENT_DISCONNECTED:
		mqtt_context.connected = false;
		ESP_LOGW(TAG, "disconnected from broker, will retry");
		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGW(TAG, "mqtt error type %d", event->error_handle ? event->error_handle->error_type : -1);
		break;
	default:
		break;
	}
}

/****************************************************************************************
 *
 */
void mqtt_svc_set_connect_hook(void (*hook)(void)) {
	for (int i = 0; i < MAX_CONNECT_HOOKS; i++) {
		if (!mqtt_context.connect_hook[i]) {
			mqtt_context.connect_hook[i] = hook;
			return;
		}
	}

	ESP_LOGE(TAG, "too many connect hooks");
}

/****************************************************************************************
 *
 */
void mqtt_svc_init(void) {
	char *config = config_alloc_get_str("mqtt_config", CONFIG_MQTT_CONFIG, "");
	char host[VALUE_LEN] = "", user[VALUE_LEN] = "", password[VALUE_LEN] = "";
	char topic[TOPIC_LEN] = "";

	if (!config) return;

	// the widths below are stringified by the macro, so they have to stay literals
	PARSE_PARAM_STR(config, "host", '=', host, 95);			// VALUE_LEN - 1
	PARSE_PARAM_STR(config, "user", '=', user, 95);			// VALUE_LEN - 1
	PARSE_PARAM_STR(config, "password", '=', password, 95);	// VALUE_LEN - 1
	PARSE_PARAM_STR(config, "topic", '=', topic, 127);		// TOPIC_LEN - 1
	strcpy(mqtt_context.discovery, "homeassistant");
	PARSE_PARAM_STR(config, "discovery", '=', mqtt_context.discovery, 31);

	free(config);

	if (!*host) {
		ESP_LOGI(TAG, "no MQTT broker configured");
		return;
	}

	// default the base topic to the device's own name, which is what the user renames
	if (!*topic) {
		char *name = config_alloc_get_str("host_name", NULL, "squeezelite");
		format_topic(topic, TOPIC_LEN, "squeezelite/%s", name ? name : "esp32");
		if (name) free(name);
	}

	sanitize_topic(topic);
	strncpy(mqtt_context.base, topic, TOPIC_LEN - 1);
	format_topic(mqtt_context.availability, sizeof(mqtt_context.availability), "%s/availability", mqtt_context.base);

	// a single '-' as prefix is how discovery is switched off
	if (!strcmp(mqtt_context.discovery, "-")) *mqtt_context.discovery = '\0';

	/*
	The client's task goes straight for the TCP/IP stack, so lwIP has to exist by now
	even though no interface may be up yet - failing to connect is fine, a missing stack
	is not. Both calls below are no-ops when the network manager got there first, which
	it normally has.
	*/
	esp_netif_init();
	esp_event_loop_create_default();

	esp_mqtt_client_config_t client_config = {
		.uri = host,
		.username = *user ? user : NULL,
		.password = *password ? password : NULL,
		.lwt_topic = mqtt_context.availability,
		.lwt_msg = "offline",
		.lwt_qos = 1,
		.lwt_retain = true,
		.keepalive = 60,
	};

	mqtt_context.client = esp_mqtt_client_init(&client_config);
	if (!mqtt_context.client) {
		ESP_LOGE(TAG, "cannot create MQTT client for %s", host);
		return;
	}

	esp_mqtt_client_register_event(mqtt_context.client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

	if (esp_mqtt_client_start(mqtt_context.client) != ESP_OK) {
		ESP_LOGE(TAG, "cannot start MQTT client for %s", host);
		esp_mqtt_client_destroy(mqtt_context.client);
		mqtt_context.client = NULL;
		return;
	}

	ESP_LOGI(TAG, "MQTT client started for %s, base topic %s", host, mqtt_context.base);
}

/****************************************************************************************
 *
 */
bool mqtt_svc_connected(void) {
	return mqtt_context.connected;
}

/****************************************************************************************
 *
 */
const char *mqtt_svc_topic_base(void) {
	return *mqtt_context.base ? mqtt_context.base : NULL;
}

/****************************************************************************************
 *
 */
const char *mqtt_svc_discovery_prefix(void) {
	return *mqtt_context.discovery ? mqtt_context.discovery : NULL;
}

/****************************************************************************************
 *
 */
bool mqtt_svc_publish(const char *subtopic, const char *payload, int qos, bool retain) {
	char topic[TOPIC_LEN];

	if (!mqtt_context.client || !mqtt_context.connected) return false;

	// a leading '/' escapes the device's base topic
	if (*subtopic == '/') format_topic(topic, TOPIC_LEN, "%s", subtopic + 1);
	else format_topic(topic, TOPIC_LEN, "%s/%s", mqtt_context.base, subtopic);

	if (esp_mqtt_client_publish(mqtt_context.client, topic, payload, 0, qos, retain) < 0) {
		ESP_LOGW(TAG, "cannot publish on %s", topic);
		return false;
	}

	ESP_LOGD(TAG, "published %s on %s", payload, topic);

	return true;
}
