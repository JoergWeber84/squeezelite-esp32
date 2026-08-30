/*
 *  RFID reader service - reports scanned tags over MQTT
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task.h"
#include "platform_config.h"
#include "globdefs.h"
#include "messaging.h"
#include "mqtt_svc.h"
#include "rc522.h"
#include "rfid.h"

static const char *TAG = "rfid";

#define RFID_STACK_SIZE		4096
#define TOPIC_LEN			128
#define UID_STR_LEN			(RC522_UID_MAX_LEN * 2 + 1)

#define DEFAULT_POLL_MS		200
#define DEFAULT_HOLD_MS		5000
#define DEFAULT_SPEED_HZ	5000000

static EXT_RAM_ATTR struct {
	rc522_handle_t reader;
	char topic[TOPIC_LEN];
	char device_id[64];
	char last_uid[UID_STR_LEN];
	uint32_t last_seen;
	int poll_ms, hold_ms;
} rfid_context;

/****************************************************************************************
 * Topics and payloads embed strings of unknown length, which snprintf cannot be talked
 * out of warning about. Going through vsnprintf keeps the truncation check ours.
 */
static bool format_buffer(char *dst, size_t size, const char *fmt, ...) {
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(dst, size, fmt, args);
	va_end(args);

	if (len < 0 || (size_t) len >= size) {
		ESP_LOGW(TAG, "value does not fit, truncated to %s", dst);
		return false;
	}

	return true;
}

/****************************************************************************************
 * Announce the reader as a Home Assistant tag scanner. HA turns anything published on
 * our topic into a tag_scanned event that automations can trigger on.
 */
static void publish_discovery(void) {
	const char *prefix = mqtt_svc_discovery_prefix();
	char topic[TOPIC_LEN], payload[512];

	if (!prefix) return;

	format_buffer(topic, sizeof(topic), "/%s/tag/%s/config", prefix, rfid_context.device_id);
	format_buffer(payload, sizeof(payload),
			 "{\"topic\":\"%s/%s\",\"value_template\":\"{{ value_json.uid }}\","
			 "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\","
			 "\"manufacturer\":\"squeezelite-esp32\",\"model\":\"RC522\"}}",
			 mqtt_svc_topic_base(), rfid_context.topic,
			 rfid_context.device_id, rfid_context.device_id);

	mqtt_svc_publish(topic, payload, 1, true);

	ESP_LOGI(TAG, "published Home Assistant tag discovery on %s", topic + 1);
}

/****************************************************************************************
 *
 */
static void report_tag(const rc522_uid_t *uid) {
	char uid_str[UID_STR_LEN], payload[128];

	rc522_uid_to_string(uid, uid_str, sizeof(uid_str));

	/*
	A tag left in the field answers every poll, so only report it again once the hold
	delay has passed. A different tag is always reported right away.
	*/
	uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
	bool same = !strcmp(uid_str, rfid_context.last_uid);

	if (same && now - rfid_context.last_seen < (uint32_t) rfid_context.hold_ms) {
		rfid_context.last_seen = now;
		return;
	}

	strcpy(rfid_context.last_uid, uid_str);
	rfid_context.last_seen = now;

	ESP_LOGI(TAG, "tag %s (sak 0x%02x)", uid_str, uid->sak);

	format_buffer(payload, sizeof(payload), "{\"uid\":\"%s\",\"sak\":%u,\"len\":%u}",
				  uid_str, uid->sak, uid->len);

	// tag scans are events, publishing them retained would replay them on every restart
	if (!mqtt_svc_publish(rfid_context.topic, payload, 0, false)) {
		ESP_LOGW(TAG, "tag %s could not be published, broker not connected", uid_str);
	}
}

/****************************************************************************************
 *
 */
static void rfid_task(void *arg) {
	ESP_LOGI(TAG, "starting rfid task, polling every %d ms", rfid_context.poll_ms);

	while (1) {
		rc522_uid_t uid;

		if (rc522_poll(rfid_context.reader, &uid)) report_tag(&uid);

		vTaskDelay(pdMS_TO_TICKS(rfid_context.poll_ms));
	}
}

/****************************************************************************************
 *
 */
void rfid_svc_init(void) {
	char *config = config_alloc_get_str("rfid_config", CONFIG_RFID_CONFIG, "");
	char model[16] = "", topic[TOPIC_LEN] = "";
	int cs = -1, rst = -1, speed = DEFAULT_SPEED_HZ;

	if (!config) return;

	// the widths below are stringified by the macro, so they have to stay literals
	PARSE_PARAM_STR(config, "model", '=', model, 15);
	PARSE_PARAM_STR(config, "topic", '=', topic, 127);	// TOPIC_LEN - 1
	PARSE_PARAM(config, "cs", '=', cs);
	PARSE_PARAM(config, "rst", '=', rst);
	PARSE_PARAM(config, "speed", '=', speed);

	rfid_context.poll_ms = DEFAULT_POLL_MS;
	rfid_context.hold_ms = DEFAULT_HOLD_MS;
	PARSE_PARAM(config, "poll", '=', rfid_context.poll_ms);
	PARSE_PARAM(config, "hold", '=', rfid_context.hold_ms);

	free(config);

	if (!*model) {
		ESP_LOGI(TAG, "no RFID reader configured");
		return;
	}

	if (strcasecmp(model, "RC522")) {
		ESP_LOGE(TAG, "unknown RFID reader model %s", model);
		return;
	}

	if (cs < 0) {
		ESP_LOGE(TAG, "RC522 needs a chip select GPIO (cs=<gpio>)");
		return;
	}

	if (spi_system_host == -1) {
		ESP_LOGE(TAG, "RC522 needs the shared SPI bus, set spi_config first");
		return;
	}

	strcpy(rfid_context.topic, *topic ? topic : "rfid");

	// the device id ties the discovery message to this player, so reuse its name
	char *name = config_alloc_get_str("host_name", NULL, "squeezelite");
	strncpy(rfid_context.device_id, name ? name : "squeezelite", sizeof(rfid_context.device_id) - 1);
	if (name) free(name);
	for (char *p = rfid_context.device_id; *p; p++) {
		if (*p == ' ' || *p == '/' || *p == '+' || *p == '#') *p = '_';
	}

	rfid_context.reader = rc522_create(spi_system_host, cs, rst, speed);
	if (!rfid_context.reader) {
		messaging_post_message(MESSAGING_ERROR, MESSAGING_CLASS_SYSTEM,
							   "RFID reader not responding on cs %d", cs);
		return;
	}

	// discovery is retained and must be re-sent whenever the broker session restarts
	mqtt_svc_set_connect_hook(publish_discovery);

	static DRAM_ATTR StaticTask_t task_buffer __attribute__ ((aligned (4)));
	static EXT_RAM_ATTR StackType_t task_stack[RFID_STACK_SIZE] __attribute__ ((aligned (4)));

	xTaskCreateStatic(rfid_task, "rfid", RFID_STACK_SIZE, NULL, ESP_TASK_PRIO_MIN + 1,
					  task_stack, &task_buffer);

	ESP_LOGI(TAG, "RC522 on cs %d rst %d, publishing tags on %s", cs, rst, rfid_context.topic);
}
