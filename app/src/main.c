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

#include "common.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define LED0_NODE DT_NODELABEL(led0)
#define LED1_NODE DT_NODELABEL(led1)
#define LED2_NODE DT_NODELABEL(led2)
#define LED3_NODE DT_NODELABEL(led3)

//
// TODO:
// + Use the RTT console to emulate BLE commands
// + Make sampling rate, accuracy, etc. configurable through a command syntax
// + Save settings to NVM
// + Use "at least" values as input and map to hardware.
// + Refactor packet recording code into a common "channel" module
// + Add huffman compression with static dictionaries
// + Test different quantization for temperature and pressure - instead of min / max just use step size
// + Follow Micropython framing format with timestamps, etc.
// + Split accel into 3 separate "channels" so they can be toggled independently
// + Use the lis3dhtr interrupt to wake from Deep Sleep.
// + Implement Deep Sleep for all the peripherals.
// + Use Zephyr DTS to disable unwanted nRF units - extra SPI etc.
// + Figure out how much RAM we actually have and implement that in the device tree.
// + Use the physical button for something, like clearing nvram
//

//
// SPI example:
//
// https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-5-serial-peripheral-interface-spi/topic/zephyr-spi-api/
//

#define SPI_NODE DT_NODELABEL(spi0)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

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

int cmd_table_lookup(const struct shell *shell, const char **table, size_t table_size, const char *value)
{
    for (size_t i = 0; i < table_size; i++)
    {
        if (strcmp(table[i], value) == 0)
        {
            return (int)i;
        }
    }
    shell_fprintf(shell, SHELL_ERROR, "Invalid argument value: %s\n", value);
    shell_fprintf(shell, SHELL_ERROR, "Valid options are:\n");
    for (size_t i = 0; i < table_size; i++)
    {
        shell_fprintf(shell, SHELL_ERROR, " %s\n", table[i]);
    }
    return -1;
}

int main(void)
{
	k_msleep(500); // settle power

	LOG_INF("KartCam Tire Sensor bring-up");

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);

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

	lis3dh_init();
	dps368_init();

	int n = 0;

	while (1) {
		// read rtt console for commands to start/stop logging, set rates, etc.



		if (n % 4 == 0) gpio_pin_toggle_dt(&led0);
		if (n % 4 == 1) gpio_pin_toggle_dt(&led1);
		if (n % 4 == 2) gpio_pin_toggle_dt(&led2);
		if (n % 4 == 3) gpio_pin_toggle_dt(&led3);

		k_msleep(1000);

		if (n % 4 == 0) gpio_pin_toggle_dt(&led0);
		if (n % 4 == 1) gpio_pin_toggle_dt(&led1);
		if (n % 4 == 2) gpio_pin_toggle_dt(&led2);
		if (n % 4 == 3) gpio_pin_toggle_dt(&led3);

		n++;
	}

	return 0;
}
