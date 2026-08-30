/*
 *  MFRC522 (RC522) contactless reader driver - SPI only
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rc522.h"

static const char *TAG = "rc522";

// MFRC522 registers (datasheet section 9.2), addresses are given unshifted
#define REG_COMMAND			0x01
#define REG_COM_IRQ			0x04
#define REG_DIV_IRQ			0x05
#define REG_ERROR			0x06
#define REG_FIFO_DATA		0x09
#define REG_FIFO_LEVEL		0x0a
#define REG_CONTROL			0x0c
#define REG_BIT_FRAMING		0x0d
#define REG_COLL			0x0e
#define REG_MODE			0x11
#define REG_TX_CONTROL		0x14
#define REG_TX_ASK			0x15
#define REG_CRC_RESULT_H	0x21
#define REG_CRC_RESULT_L	0x22
#define REG_T_MODE			0x2a
#define REG_T_PRESCALER		0x2b
#define REG_T_RELOAD_H		0x2c
#define REG_T_RELOAD_L		0x2d
#define REG_VERSION			0x37

// MFRC522 commands
#define CMD_IDLE			0x00
#define CMD_CALC_CRC		0x03
#define CMD_TRANSCEIVE		0x0c
#define CMD_SOFT_RESET		0x0f

// PICC commands (ISO/IEC 14443-3)
#define PICC_REQA			0x26
#define PICC_HLTA			0x50
#define PICC_SEL_CL1		0x93
#define PICC_SEL_CL2		0x95
#define PICC_SEL_CL3		0x97
#define PICC_CASCADE_TAG	0x88

// the chip's own timer is set for 25ms, give the driver a bit more than that
#define TRANSCEIVE_TIMEOUT_US	40000

// largest burst we ever push through the FIFO is a 9 byte SELECT plus the address byte
#define SPI_BURST_MAX		16

struct rc522_s {
	spi_device_handle_t spi;
	int rst_gpio;
};

/****************************************************************************************
 * Register access. The RC522 wants the address in bits 6..1 and the read flag in bit 7,
 * repeated for every byte that is clocked out of the FIFO.
 */
static void write_reg_n(rc522_handle_t dev, uint8_t reg, const uint8_t *data, size_t len) {
	uint8_t buf[SPI_BURST_MAX];

	if (len + 1 > sizeof(buf)) return;

	buf[0] = (reg << 1) & 0x7e;
	memcpy(buf + 1, data, len);

	spi_transaction_t t = {
		.length = (len + 1) * 8,
		.tx_buffer = buf,
	};

	spi_device_polling_transmit(dev->spi, &t);
}

static void write_reg(rc522_handle_t dev, uint8_t reg, uint8_t value) {
	write_reg_n(dev, reg, &value, 1);
}

static void read_reg_n(rc522_handle_t dev, uint8_t reg, uint8_t *data, size_t len) {
	uint8_t tx[SPI_BURST_MAX], rx[SPI_BURST_MAX];

	if (!len || len + 1 > sizeof(tx)) return;

	// the address is repeated for every byte, a trailing 0 closes the burst
	memset(tx, ((reg << 1) & 0x7e) | 0x80, len);
	tx[len] = 0;

	spi_transaction_t t = {
		.length = (len + 1) * 8,
		.rxlength = (len + 1) * 8,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};

	spi_device_polling_transmit(dev->spi, &t);
	memcpy(data, rx + 1, len);
}

static uint8_t read_reg(rc522_handle_t dev, uint8_t reg) {
	uint8_t value = 0;
	read_reg_n(dev, reg, &value, 1);
	return value;
}

static void set_bits(rc522_handle_t dev, uint8_t reg, uint8_t mask) {
	write_reg(dev, reg, read_reg(dev, reg) | mask);
}

static void clear_bits(rc522_handle_t dev, uint8_t reg, uint8_t mask) {
	write_reg(dev, reg, read_reg(dev, reg) & ~mask);
}

/****************************************************************************************
 * CRC_A, computed by the chip itself
 */
static bool calculate_crc(rc522_handle_t dev, const uint8_t *data, size_t len, uint8_t *crc) {
	int64_t deadline = esp_timer_get_time() + TRANSCEIVE_TIMEOUT_US;

	write_reg(dev, REG_COMMAND, CMD_IDLE);
	write_reg(dev, REG_DIV_IRQ, 0x04);
	write_reg(dev, REG_FIFO_LEVEL, 0x80);
	write_reg_n(dev, REG_FIFO_DATA, data, len);
	write_reg(dev, REG_COMMAND, CMD_CALC_CRC);

	while (esp_timer_get_time() < deadline) {
		// CRCIRq is raised once the calculation is done
		if (read_reg(dev, REG_DIV_IRQ) & 0x04) {
			write_reg(dev, REG_COMMAND, CMD_IDLE);
			crc[0] = read_reg(dev, REG_CRC_RESULT_L);
			crc[1] = read_reg(dev, REG_CRC_RESULT_H);
			return true;
		}
	}

	ESP_LOGD(TAG, "CRC calculation timed out");

	return false;
}

/****************************************************************************************
 * Send to and receive from the card. back_len is in/out (buffer size in, bytes received
 * out) and valid_bits returns the significant bits of the last received byte.
 */
static bool transceive(rc522_handle_t dev, const uint8_t *send, size_t send_len, uint8_t tx_last_bits,
					   uint8_t *back, size_t *back_len, uint8_t *valid_bits) {
	int64_t deadline = esp_timer_get_time() + TRANSCEIVE_TIMEOUT_US;
	uint8_t irq = 0, error, received;

	write_reg(dev, REG_COMMAND, CMD_IDLE);
	write_reg(dev, REG_COM_IRQ, 0x7f);
	write_reg(dev, REG_FIFO_LEVEL, 0x80);
	write_reg_n(dev, REG_FIFO_DATA, send, send_len);
	write_reg(dev, REG_BIT_FRAMING, tx_last_bits & 0x07);
	write_reg(dev, REG_COMMAND, CMD_TRANSCEIVE);
	set_bits(dev, REG_BIT_FRAMING, 0x80);

	// wait for RxIRq or IdleIRq, the chip raises TimerIRq on its own when nobody answers
	do {
		irq = read_reg(dev, REG_COM_IRQ);
		if (irq & 0x30) break;
		if (irq & 0x01) {
			ESP_LOGV(TAG, "no card answered");
			clear_bits(dev, REG_BIT_FRAMING, 0x80);
			return false;
		}
	} while (esp_timer_get_time() < deadline);

	clear_bits(dev, REG_BIT_FRAMING, 0x80);

	if (!(irq & 0x30)) {
		ESP_LOGD(TAG, "transceive timed out (irq 0x%02x)", irq);
		return false;
	}

	// BufferOvfl, ParityErr and ProtocolErr are fatal, CollErr is checked further down
	error = read_reg(dev, REG_ERROR);
	if (error & 0x13) {
		ESP_LOGD(TAG, "transceive error 0x%02x", error);
		return false;
	}

	received = read_reg(dev, REG_FIFO_LEVEL);
	if (received > *back_len) {
		ESP_LOGD(TAG, "answer of %u bytes does not fit in %u", received, (unsigned) *back_len);
		return false;
	}

	read_reg_n(dev, REG_FIFO_DATA, back, received);
	*back_len = received;

	if (valid_bits) {
		uint8_t bits = read_reg(dev, REG_CONTROL) & 0x07;
		*valid_bits = bits ? bits : 8;
	}

	// a collision means more than one card sits in the field, let the caller retry
	if (error & 0x08) {
		ESP_LOGD(TAG, "collision detected");
		return false;
	}

	return true;
}

/****************************************************************************************
 * REQA - is there an idle card in the field
 */
static bool request_a(rc522_handle_t dev) {
	uint8_t command = PICC_REQA, atqa[2], valid_bits = 0;
	size_t len = sizeof(atqa);

	// REQA is a 7 bit short frame and the answer must be a complete 2 byte ATQA
	write_reg(dev, REG_COLL, 0x80);
	if (!transceive(dev, &command, 1, 7, atqa, &len, &valid_bits)) return false;

	return len == 2 && valid_bits == 8;
}

/****************************************************************************************
 * Anticollision + select over up to three cascade levels, yielding the full UID
 */
static bool select_card(rc522_handle_t dev, rc522_uid_t *uid) {
	static const uint8_t cascade_levels[] = { PICC_SEL_CL1, PICC_SEL_CL2, PICC_SEL_CL3 };

	uid->len = 0;

	for (int level = 0; level < (int) sizeof(cascade_levels); level++) {
		uint8_t buffer[9], answer[5], bcc = 0;
		size_t len;

		// anticollision: NVB 0x20 means we send no part of the UID and want all of it
		buffer[0] = cascade_levels[level];
		buffer[1] = 0x20;

		len = sizeof(answer);
		write_reg(dev, REG_COLL, 0x00);
		if (!transceive(dev, buffer, 2, 0, answer, &len, NULL) || len != 5) return false;

		for (int i = 0; i < 4; i++) bcc ^= answer[i];
		if (bcc != answer[4]) {
			ESP_LOGD(TAG, "bad BCC on cascade level %d", level + 1);
			return false;
		}

		// select: NVB 0x70 means the full UID of this level follows, plus a CRC_A
		memcpy(buffer + 2, answer, 5);
		buffer[1] = 0x70;
		if (!calculate_crc(dev, buffer, 7, buffer + 7)) return false;

		len = sizeof(answer);
		write_reg(dev, REG_COLL, 0x80);
		if (!transceive(dev, buffer, 9, 0, answer, &len, NULL) || len != 3) return false;

		uid->sak = answer[0];

		/*
		A cascade tag as first byte means this level only carries 3 UID bytes and another
		level follows. Otherwise all 4 bytes belong to the UID.
		*/
		if (buffer[2] == PICC_CASCADE_TAG) {
			memcpy(uid->bytes + uid->len, buffer + 3, 3);
			uid->len += 3;
		} else {
			memcpy(uid->bytes + uid->len, buffer + 2, 4);
			uid->len += 4;
		}

		// bit 2 of SAK is the "cascade continues" flag
		if (!(uid->sak & 0x04)) return true;
	}

	ESP_LOGD(TAG, "UID longer than three cascade levels");

	return false;
}

/****************************************************************************************
 * Put the card back to halt so that it answers REQA again on the next poll
 */
static void halt_card(rc522_handle_t dev) {
	uint8_t buffer[4] = { PICC_HLTA, 0x00 }, answer[4];
	size_t len = sizeof(answer);

	if (!calculate_crc(dev, buffer, 2, buffer + 2)) return;

	// a card that accepts HLTA stays silent, so the timeout here is the success case
	transceive(dev, buffer, 4, 0, answer, &len, NULL);
}

/****************************************************************************************
 * Bring the chip in a known state
 */
static void chip_init(rc522_handle_t dev) {
	write_reg(dev, REG_COMMAND, CMD_SOFT_RESET);
	vTaskDelay(pdMS_TO_TICKS(50));

	// timer runs at ~40kHz and reloads at 1000, giving the 25ms answer timeout
	write_reg(dev, REG_T_MODE, 0x80);
	write_reg(dev, REG_T_PRESCALER, 0xa9);
	write_reg(dev, REG_T_RELOAD_H, 0x03);
	write_reg(dev, REG_T_RELOAD_L, 0xe8);

	write_reg(dev, REG_TX_ASK, 0x40);		// force 100% ASK
	write_reg(dev, REG_MODE, 0x3d);			// CRC preset 0x6363

	set_bits(dev, REG_TX_CONTROL, 0x03);	// antenna on
}

/****************************************************************************************
 * Public interface
 */
rc522_handle_t rc522_create(int spi_host, int cs_gpio, int rst_gpio, int speed_hz) {
	rc522_handle_t dev = calloc(1, sizeof(struct rc522_s));
	uint8_t version;

	if (!dev) return NULL;

	dev->rst_gpio = rst_gpio;

	spi_device_interface_config_t config = {
		.clock_speed_hz = speed_hz,
		.mode = 0,
		.spics_io_num = cs_gpio,
		.queue_size = 1,
	};

	if (spi_bus_add_device(spi_host, &config, &dev->spi) != ESP_OK) {
		ESP_LOGE(TAG, "cannot add RC522 to SPI host %d", spi_host);
		free(dev);
		return NULL;
	}

	if (rst_gpio >= 0) {
		gpio_pad_select_gpio(rst_gpio);
		gpio_set_direction(rst_gpio, GPIO_MODE_OUTPUT);
		gpio_set_level(rst_gpio, 0);
		vTaskDelay(pdMS_TO_TICKS(5));
		gpio_set_level(rst_gpio, 1);
		vTaskDelay(pdMS_TO_TICKS(50));
	}

	chip_init(dev);

	version = read_reg(dev, REG_VERSION);
	if (version == 0x00 || version == 0xff) {
		ESP_LOGE(TAG, "no RC522 answering on cs %d (version 0x%02x)", cs_gpio, version);
		spi_bus_remove_device(dev->spi);
		free(dev);
		return NULL;
	}

	ESP_LOGI(TAG, "RC522 found on cs %d, version 0x%02x", cs_gpio, version);

	return dev;
}

void rc522_destroy(rc522_handle_t dev) {
	if (!dev) return;

	clear_bits(dev, REG_TX_CONTROL, 0x03);
	spi_bus_remove_device(dev->spi);
	free(dev);
}

uint8_t rc522_version(rc522_handle_t dev) {
	return dev ? read_reg(dev, REG_VERSION) : 0;
}

bool rc522_poll(rc522_handle_t dev, rc522_uid_t *uid) {
	bool found;

	if (!dev || !request_a(dev)) return false;

	found = select_card(dev, uid);
	halt_card(dev);

	return found;
}

void rc522_uid_to_string(const rc522_uid_t *uid, char *buf, size_t size) {
	size_t offset = 0;

	if (!size) return;

	for (int i = 0; i < uid->len && offset + 3 <= size; i++) {
		offset += snprintf(buf + offset, size - offset, "%02X", uid->bytes[i]);
	}

	buf[offset < size ? offset : size - 1] = '\0';
}
