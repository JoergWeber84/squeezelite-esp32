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
