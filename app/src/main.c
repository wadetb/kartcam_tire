#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>

#include <app_version.h>

#include "common.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define LED0_NODE DT_NODELABEL(led0)
#define LED1_NODE DT_NODELABEL(led1)
// #define LED2_NODE DT_NODELABEL(led2)
// #define LED3_NODE DT_NODELABEL(led3)

#define BTN0_NODE DT_NODELABEL(btn0)

//
// TODO:
// + Drop packets unless recording.
// + Expose per-channel sample rates via shell commands.
// + Expose recording duration and status via shell command.
// + Explicit sleep command.
// + Save settings to NVM.
// + Reset NVRAM if button held at startup?
// + Use "at least" values as input and map to hardware.
// + Add huffman compression with static dictionaries
// + Follow Micropython framing format with timestamps, etc.
// + Use Zephyr DTS to disable unwanted nRF units - extra SPI etc.
//

//
// SPI example:
//
// https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-5-serial-peripheral-interface-spi/topic/zephyr-spi-api/
//

#define SPI_NODE  DT_NODELABEL(spi0)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
// static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
// static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

// /* NUS-like UUIDs (Nordic UART Service) */
// #define BT_UUID_NUS_SERVICE_VAL
// BT_UUID_128_ENCODE(0x6E400001,0xB5A3,0xF393,0xE0A9,0xE50E24DCCA9E) #define BT_UUID_NUS_RX_VAL
// BT_UUID_128_ENCODE(0x6E400002,0xB5A3,0xF393,0xE0A9,0xE50E24DCCA9E) #define BT_UUID_NUS_TX_VAL
// BT_UUID_128_ENCODE(0x6E400003,0xB5A3,0xF393,0xE0A9,0xE50E24DCCA9E)

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
// 	BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
// NULL, NULL), 	BT_GATT_CCC(tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
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

int cmd_table_lookup(const struct shell *shell, const char **table, size_t table_size,
		     const char *value)
{
	for (size_t i = 0; i < table_size; i++) {
		if (strcmp(table[i], value) == 0) {
			return (int)i;
		}
	}
	shell_fprintf(shell, SHELL_ERROR, "Invalid argument value: %s\n", value);
	shell_fprintf(shell, SHELL_ERROR, "Valid options are:\n");
	for (size_t i = 0; i < table_size; i++) {
		shell_fprintf(shell, SHELL_ERROR, " %s\n", table[i]);
	}
	return -1;
}

int print_reset_cause(uint32_t reset_cause)
{
	int32_t ret;
	uint32_t supported;

	ret = hwinfo_get_supported_reset_cause((uint32_t *) &supported);

	if (ret || !(reset_cause & supported)) {
		return -ENOTSUP;
	}

	if (reset_cause & RESET_DEBUG) {
		LOG_INF("Reset by debugger.\n");
	} else if (reset_cause & RESET_CLOCK) {
		LOG_INF("Wakeup from System OFF by GRTC.\n");
	} else if (reset_cause & RESET_LOW_POWER_WAKE) {
		LOG_INF("Wakeup from System OFF by GPIO.\n");
	} else  {
		LOG_INF("Other wake up cause 0x%08X.\n", reset_cause);
	}

	return 0;
}

const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(BTN0_NODE, gpios);
struct gpio_callback btn0_gpio_cb;
K_EVENT_DEFINE(btn0_event);

static void btn0_int_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_event_post(&btn0_event, 1);
}

int main(void)
{
	k_msleep(100); // settle power

	LOG_INF("KartCam Tire Sensor bring-up");

	uint32_t reset_cause;
	hwinfo_get_reset_cause(&reset_cause);
	print_reset_cause(reset_cause);

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

	gpio_pin_configure_dt(&btn0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn0, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&btn0_gpio_cb, btn0_int_handler, BIT(btn0.pin));
	gpio_add_callback(btn0.port, &btn0_gpio_cb);

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

	if (!device_is_ready(spi_dev)) {
		LOG_WRN("SPI master device not ready!\n");
	}

	lis3dh_init();
	dps368_init();

	for (;;)
	{
		gpio_pin_set_dt(&led0, 0);
		gpio_pin_set_dt(&led1, 0);

		bool record = false;

		for (int n = 10; n >= 0; n--) {
			gpio_pin_toggle_dt(&led0);

			LOG_INF("sleep in %d seconds...", n);

			// float x, y, z;
			// lis3dh_latest(&x, &y, &z);
			// LOG_INF("  accel: x=%f mg, y=%f mg, z=%f mg", (double)x, (double)y, (double)z);

			uint32_t events = k_event_wait(&btn0_event, 0xFFF, true, K_MSEC(1000));
			if (events & 1) {
				LOG_INF("button pressed!");
				k_msleep(10);
				record = true;
			}

			if (record) {
				break;
			}
		}

		if (!record) {
			break;
		}

		LOG_INF("recording...");

		gpio_pin_set_dt(&led0, 0);
		gpio_pin_set_dt(&led1, 1);

		uint32_t record_ms = 15 * 60 * 1000;
		uint32_t events = k_event_wait(&btn0_event, 0xFFF, true, K_MSEC(record_ms));
		if (events & 1) {
			LOG_INF("button pressed!");
			k_msleep(10);
		}
	}

	gpio_pin_set_dt(&led0, 0);
	gpio_pin_set_dt(&led1, 0);
	dps368_stop();

	LOG_INF("entering deep sleep, wake on interrupt");
	k_msleep(100);

	lis3dh_wake_on_z();
	hwinfo_clear_reset_cause();
	sys_poweroff();	

	return 0;
}
