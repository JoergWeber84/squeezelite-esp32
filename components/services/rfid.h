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
that enters the field is published once on <base topic>/<sub topic>; holding the same
tag in front of the reader only repeats after "hold" milliseconds.
*/
void rfid_svc_init(void);
