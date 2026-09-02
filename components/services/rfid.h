/*
 *  RFID reader service - reports scanned tags over MQTT
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#pragma once

/*
Start the RFID reader from the "rfid_config" NVS entry:

	model=RC522,cs=<gpio>[,rst=<gpio>][,speed=<hz>][,poll=<ms>][,hold=<ms>][,topic=<sub>]

The reader sits on the shared SPI bus, so spi_config must be set as well. Every tag
that enters the field is published once on <base topic>/<sub topic>. A tag left lying
on the reader is not published again: the same tag is only reported anew once it has
been out of the field for "hold" milliseconds. A different tag is always reported
right away.

The reader starts several seconds before the broker connects, so a tag that is already
on the reader at boot is announced once the connection is up rather than being lost to
that repeat guard. This happens on the first connection only.
*/
void rfid_svc_init(void);

/*
The last tag seen and the one before it, as strings that are empty until there have been
that many. The scan itself stays an event: these are for reading the state at any time,
which an event cannot answer.
*/
const char *rfid_last_uid(void);
const char *rfid_previous_uid(void);   // both NULL when no reader is configured
