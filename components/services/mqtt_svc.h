/*
 *  MQTT client service - publishes device events to a broker (Home Assistant & co)
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

/*
Start the MQTT client from the "mqtt_config" NVS entry:

	host=mqtt://<ip|name>[:port][,user=<user>][,password=<password>]
	[,topic=<base topic>][,discovery=<prefix|->]

The client reconnects on its own, so the broker being unreachable is not a problem.
The network stack does have to be initialized by the time this is called, though:
the client's task talks to it immediately. Nothing happens when no host is
configured. Note that values cannot contain a comma, that is the separator of the
configuration string itself.
*/
void mqtt_svc_init(void);

bool mqtt_svc_connected(void);

// base topic of this device, e.g. "squeezelite/kitchen" - NULL when not configured
const char *mqtt_svc_topic_base(void);

// Home Assistant discovery prefix, e.g. "homeassistant" - NULL when discovery is off
const char *mqtt_svc_discovery_prefix(void);

/*
Format into dst, reporting truncation rather than letting it pass unnoticed. Topics and
payloads are built from strings whose length is not known at compile time, which snprintf
cannot be talked out of warning about under -Werror=format-truncation; going through
vsnprintf keeps the check ours. Returns false when the result did not fit.
*/
bool mqtt_svc_format(char *dst, size_t size, const char *fmt, ...);

/*
Publish under the device's base topic, or on an absolute topic when subtopic starts
with a '/'. Returns false when the client is not connected or the broker refused.
*/
bool mqtt_svc_publish(const char *subtopic, const char *payload, int qos, bool retain);

/*
Register a callback invoked every time the client connects, which is where retained
announcements such as Home Assistant discovery messages should be (re)published.
Hooks must be registered before mqtt_svc_init().
*/
void mqtt_svc_set_connect_hook(void (*hook)(void));
