#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <app_version.h>

#include "lis3dhtr.h"
#include "dps368.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define LED0_NODE DT_NODELABEL(led0)
#define LED1_NODE DT_NODELABEL(led1)
#define LED2_NODE DT_NODELABEL(led2)
#define LED3_NODE DT_NODELABEL(led3)

#define SPI_NODE DT_NODELABEL(spi0)

#define DPS368_NODE DT_NODELABEL(dps368)
#define LIS3DHTR_NODE DT_NODELABEL(lis3dhtr)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

#define DPS368_OP  SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE
#define LIS3DHTR_OP SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

struct spi_dt_spec dps368 = SPI_DT_SPEC_GET(DPS368_NODE, DPS368_OP);
struct spi_dt_spec lis3dhtr = SPI_DT_SPEC_GET(LIS3DHTR_NODE, LIS3DHTR_OP);

int spi_write_uint8(const struct spi_dt_spec *spec, uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2] = { reg & 0x3F, val }; /* ensure WRITE bit is 0 */
    
    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    const struct spi_buf_set txs = { .buffers = &tx, .count = 1 };
    
    return spi_write_dt(spec, &txs);
}

uint8_t spi_read_uint8(const struct spi_dt_spec *spec, uint8_t reg)
{
    uint8_t tx_buf[2] = { reg | 0x80, 0 }; /* set READ bit */
    uint8_t rx_buf[2] = { 0xFF, 0xFF }; /* Initialize to detect no-response */
    
    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf rx = { .buf = rx_buf, .len = 2 };
    const struct spi_buf_set txs = { .buffers = &tx, .count = 1 };
    const struct spi_buf_set rxs = { .buffers = &rx, .count = 1 };

    spi_transceive_dt(spec, &txs, &rxs);

	return rx_buf[1];
}

uint16_t spi_read_int16(const struct spi_dt_spec *spec, uint8_t reg)
{
	uint8_t tx_buf[3] = { reg | 0x80 | 0x40, 0, 0 }; /* set READ and MULTI bits */
	uint8_t rx_buf[3] = { 0xFF, 0xFF, 0xFF }; /* Initialize to detect no-response */
	
	struct spi_buf tx = { .buf = tx_buf, .len = 3 };
	struct spi_buf rx = { .buf = rx_buf, .len = 3 };
	const struct spi_buf_set txs = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rx, .count = 1 };

	spi_transceive_dt(spec, &txs, &rxs);

	return (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
}

void spi_read_buf(const struct spi_dt_spec *spec, uint8_t reg, uint8_t length, uint8_t *output)
{
    uint8_t cmd = reg | 0x80 | 0x40;
    uint8_t tx_buf[16]; // Max reasonable read size
    uint8_t rx_buf[16];
    
    if (length > 15) {
        return;
    }
    
    tx_buf[0] = cmd;
    for (int i = 1; i <= length; i++) {
        tx_buf[i] = 0x00; // Dummy bytes for reading
    }
    
    struct spi_buf tx = { .buf = tx_buf, .len = length + 1 };
    struct spi_buf rx = { .buf = rx_buf, .len = length + 1 };
    const struct spi_buf_set txs = { .buffers = &tx, .count = 1 };
    const struct spi_buf_set rxs = { .buffers = &rx, .count = 1 };

    spi_transceive_dt(spec, &txs, &rxs);

	for (int i = 0; i < length; i++) {
		output[i] = rx_buf[i + 1];
	}
}

int main(void)
{
	k_msleep(500); /* settle power */

	LOG_INF("KartCam Tire Sensor bring-up");

	if(!device_is_ready(spi_dev)) {
		LOG_WRN("SPI master device not ready!\n");
	}

	lis3dhtr_init();
	dps368_init();

	int n = 0;

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);

	while (1) {
		if (n % 4 == 0) gpio_pin_toggle_dt(&led0);
		if (n % 4 == 1) gpio_pin_toggle_dt(&led1);
		if (n % 4 == 2) gpio_pin_toggle_dt(&led2);
		if (n % 4 == 3) gpio_pin_toggle_dt(&led3);

		int16_t dx, dy, dz;
		lis3dhtr_get_acceleration(&dx, &dy, &dz);
		LOG_INF("LIS3DHTR Accel X: %d Y: %d Z: %d", dx, dy, dz);

		float temp, pressure;
		dps368_read(&temp, &pressure);
		float temp_f = temp * 9.0f/5.0f + 32.0f;
		LOG_INF("DPS368 Temp: %f C (%f F) Pressure: %f", (double)temp, (double)temp_f, (double)pressure);

		k_msleep(100);

		if (n % 4 == 0) gpio_pin_toggle_dt(&led0);
		if (n % 4 == 1) gpio_pin_toggle_dt(&led1);
		if (n % 4 == 2) gpio_pin_toggle_dt(&led2);
		if (n % 4 == 3) gpio_pin_toggle_dt(&led3);

		n++;
	}

	return 0;
}
