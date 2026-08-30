/*
 *  MFRC522 (RC522) contactless reader driver - SPI only
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// a UID is 4, 7 or 10 bytes (ISO/IEC 14443-3 single/double/triple size)
#define RC522_UID_MAX_LEN	10

typedef struct {
	uint8_t bytes[RC522_UID_MAX_LEN];
	uint8_t len;
	uint8_t sak;
} rc522_uid_t;

typedef struct rc522_s *rc522_handle_t;

/*
Attach a RC522 on the shared SPI bus. The bus must already be initialized (see
services_init) and cs_gpio must be a free output. rst_gpio may be -1, in which
case only the soft reset is used. Returns NULL when the chip does not answer.
*/
rc522_handle_t rc522_create(int spi_host, int cs_gpio, int rst_gpio, int speed_hz);
void rc522_destroy(rc522_handle_t dev);

// version register of the chip, 0x91/0x92 for genuine parts, 0x00/0xff when mute
uint8_t rc522_version(rc522_handle_t dev);

/*
Look for a card in the field and return its UID. Returns false when no card is
present, when several cards collide or on any transmission error.
*/
bool rc522_poll(rc522_handle_t dev, rc522_uid_t *uid);

// hex representation of a UID, uppercase and without separator
void rc522_uid_to_string(const rc522_uid_t *uid, char *buf, size_t size);
