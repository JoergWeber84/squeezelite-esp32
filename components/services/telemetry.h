/*
 *  Periodic device telemetry over MQTT
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#pragma once

/*
Publish what the device knows about itself on <base topic>/state as one JSON object,
every "mqtt_interval" seconds (NVS, 60 by default, 0 switches it off), and announce each
field to Home Assistant as its own sensor. All of them hang off the same device as the
tag reader and reference the availability topic, so they go unavailable with it.

Start this before mqtt_svc_init(): the announcements are retained and have to be resent
on every reconnect, which is what the client's connect hook is for.
*/
void telemetry_svc_init(void);
