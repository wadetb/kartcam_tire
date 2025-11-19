#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
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

//
// TODO:
// - For lis3dhtr and dps368, use threads with sleep loop to drain fifo at the expected time
// + Make DPS368 use its FIFO to sleep longer, increase rate.
// + Make sampling rate, accuracy, etc. configurable through a config structure
// + Use the lis3dhtr interrupt to wake from Deep Sleep.
// + Implement Deep Sleep for all the peripherals.
// + Use Zephyr DTS to disable unwanted nRF units - extra SPI etc.
// + Figure out how much RAM we actually have.
// + Use the physical button for something
// + Use the RTT console to emulate BLE commands
//

//
// SPI example:
//
// https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-5-serial-peripheral-interface-spi/topic/zephyr-spi-api/
//

#define SPI_NODE DT_NODELABEL(spi0)

#define DPS368_NODE DT_NODELABEL(dps368)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

#define DPS368_OP  SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

struct spi_dt_spec dps368 = SPI_DT_SPEC_GET(DPS368_NODE, DPS368_OP);

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

// /* NUS-like UUIDs (Nordic UART Service) */
// #define BT_UUID_NUS_SERVICE_VAL BT_UUID_128_ENCODE(0x6E400001,0xB5A3,0xF393,0xE0A9,0xE50E24DCCA9E)
// #define BT_UUID_NUS_RX_VAL      BT_UUID_128_ENCODE(0x6E400002,0xB5A3,0xF393,0xE0A9,0xE50E24DCCA9E)
// #define BT_UUID_NUS_TX_VAL      BT_UUID_128_ENCODE(0x6E400003,0xB5A3,0xF393,0xE0A9,0xE50E24DCCA9E)


// static struct bt_uuid_128 nus_uuid     = BT_UUID_INIT_128(BT_UUID_NUS_SERVICE_VAL);
// static struct bt_uuid_128 nus_rx_uuid  = BT_UUID_INIT_128(BT_UUID_NUS_RX_VAL);
// static struct bt_uuid_128 nus_tx_uuid  = BT_UUID_INIT_128(BT_UUID_NUS_TX_VAL);

// /* TX CCC state */
// static void tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
// 	ARG_UNUSED(attr);
// 	LOG_INF("TX notify %s", (value == BT_GATT_CCC_NOTIFY) ? "enabled" : "disabled");
// }

// /* RX write handler: receive bytes from central */
// static ssize_t rx_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 			   const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
// {
// 	ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(offset); ARG_UNUSED(flags);
// 	LOG_INF("RX %u bytes", len);
// 	/* echo back via TX notify if enabled */
// 	bt_gatt_notify(conn, attr - 1 /* TX value attr is just before CCC */,
// 	               buf, len);
// 	return len;
// }

// /* GATT: Service with RX (Write Without Resp) and TX (Notify) */
// BT_GATT_SERVICE_DEFINE(nus_svc,
// 	BT_GATT_PRIMARY_SERVICE(&nus_uuid),
// 	/* TX characteristic: notify to central */
// 	BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
// 	BT_GATT_CCC(tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// 	/* RX characteristic: central writes here */
// 	BT_GATT_CHARACTERISTIC(&nus_rx_uuid.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
// 			       BT_GATT_PERM_WRITE, NULL, rx_write_cb, NULL)
// );

// static void connected(struct bt_conn *conn, uint8_t err)
// {
// 	if (err) { LOG_INF("Conn failed (err %u)", err); return; }
// 	LOG_INF("Connected");
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
// 	LOG_INF("Disconnected (reason 0x%02x)", reason);
// }

// BT_CONN_CB_DEFINE(conn_cbs) = {
// 	.connected = connected,
// 	.disconnected = disconnected,
// };

// #define COMPANY_ID_CODE 0x0059

// typedef struct adv_mfg_data {
// 	uint16_t company_code; /* Company Identifier Code. */
// 	uint16_t number_press; /* Number of times Button 1 is pressed */
// } adv_mfg_data_type;

// static adv_mfg_data_type adv_mfg_data = { COMPANY_ID_CODE, 0x00 };

// static const struct bt_data ad[] = {
// 	/* STEP 4.1.2 - Set the advertising flags */
// 	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
// 	/* STEP 4.1.3 - Set the advertising packet data  */
// 	BT_DATA(BT_DATA_NAME_COMPLETE, "KartCam Tire", sizeof("KartCam Tire")-1),
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data)),
// };

// static unsigned char url_data[] ={0x17,'/','/','a','c','a','d','e','m','y','.',
//                                  'n','o','r','d','i','c','s','e','m','i','.',
//                                  'c','o','m'};

// static const struct bt_data sd[] = {
// 	/* 4.2.3 Include the URL data in the scan response packet*/
// 	BT_DATA(BT_DATA_URI, url_data,sizeof(url_data)),
// };

// static const struct bt_le_adv_param *adv_param =
// 	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_NONE, /* No options specified */
// 			800, /* Min Advertising Interval 500ms (800*0.625ms) */
// 			801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
// 			NULL); /* Set to NULL for undirected advertising */

int main(void)
{
	k_msleep(500); /* settle power */

	LOG_INF("KartCam Tire Sensor bring-up");

	// int err;

	// err = bt_enable(NULL);
	// if (err) {
	// 	LOG_ERR("Bluetooth init failed (err %d)\n", err);
	// 	return -1;
	// }
	// LOG_INF("Bluetooth initialized\n");

	// err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	// if (err) {
	// 	LOG_ERR("Advertising failed to start (err %d)\n", err);
	// 	return -1;
	// }

	// for (;;)
	// {
	// 	k_msleep(100);
	// }
	
	// /* Advertise: connectable + UUID in scan response */
	// const struct bt_data ad[] = {
	// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	// 	BT_DATA(BT_DATA_NAME_COMPLETE, "KartCam Tire", sizeof("KartCam Tire")-1),
	// };
	// const struct bt_data sd[] = {
	// 	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SERVICE_VAL),
	// };

	// err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	// if (err) {
	// 	LOG_ERR("Adv start failed (%d)", err);
	// }
	// LOG_INF("Advertising 'KartCam Tire'");

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
		lis3dhtr_read(&dx, &dy, &dz);
		// LOG_INF("LIS3DHTR Accel X: %d Y: %d Z: %d", dx, dy, dz);

		// float temp, pressure;
		// dps368_read(&temp, &pressure);
		// float temp_f = temp * 9.0f/5.0f + 32.0f;
		// LOG_INF("DPS368 Temp: %f C (%f F) Pressure: %f", (double)temp, (double)temp_f, (double)pressure);

		k_msleep(1000);

		if (n % 4 == 0) gpio_pin_toggle_dt(&led0);
		if (n % 4 == 1) gpio_pin_toggle_dt(&led1);
		if (n % 4 == 2) gpio_pin_toggle_dt(&led2);
		if (n % 4 == 3) gpio_pin_toggle_dt(&led3);

		n++;
	}

	return 0;
}
