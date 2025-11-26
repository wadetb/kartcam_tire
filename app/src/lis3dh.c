#include "common.h"

LOG_MODULE_REGISTER(lis3dh);

#define LIS3DH_REG_WHO_AM_I            0x0F

#define LIS3DH_REG_CTRL_REG1           0x20
#define LIS3DH_REG_CTRL_REG1_LPEN      BIT(3)
#define LIS3DH_REG_CTRL_REG1_ZEN       BIT(2)
#define LIS3DH_REG_CTRL_REG1_YEN       BIT(1)
#define LIS3DH_REG_CTRL_REG1_XEN       BIT(0)

#define LIS3DH_REG_CTRL_REG2           0x21
#define LIS3DH_REG_CTRL_REG3           0x22

#define LIS3DH_REG_CTRL_REG4           0x23
#define LIS3DH_REG_CTRL_REG4_HR        BIT(3)

#define LIS3DH_REG_CTRL_REG5           0x24
#define LIS3DH_REG_CTRL_REG5_FIFO_EN   BIT(6)

#define LIS3DH_REG_OUT_X_L             0x28

#define LIS3DH_REG_FIFO_CTRL           0x2E
#define LIS3DH_REG_FIFO_CTRL_FM_BYPASS 0
#define LIS3DH_REG_FIFO_CTRL_FM_FIFO   BIT(6)

#define LIS3DH_REG_FIFO_SRC            0x2F
#define LIS3DH_REG_INT1_SRC            0x31

enum lis3dh_rate {
	LIS3DH_RATE_0_HZ, // Power-down mode
	LIS3DH_RATE_1_HZ,
	LIS3DH_RATE_10_HZ,
	LIS3DH_RATE_25_HZ,
	LIS3DH_RATE_50_HZ,
	LIS3DH_RATE_100_HZ,
	LIS3DH_RATE_200_HZ,
	LIS3DH_RATE_400_HZ,
	LIS3DH_RATE_1_6_KHZ,
	LIS3DH_RATE_5_KHZ
};

enum lis3dh_mode {
	LIS3DH_MODE_LOW_POWER,
	LIS3DH_MODE_NORMAL,
	LIS3DH_MODE_HIGH_RES
};

enum lis3dh_scale {
	LIS3DH_SCALE_2G,
	LIS3DH_SCALE_4G,
	LIS3DH_SCALE_8G,
	LIS3DH_SCALE_16G
};

#define LIS3DH_NODE DT_NODELABEL(lis3dh)

#define LIS3DH_OP                                                                                  \
	SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

const struct spi_dt_spec lis3dh = SPI_DT_SPEC_GET(LIS3DH_NODE, LIS3DH_OP);
const struct gpio_dt_spec lis3dh_int = GPIO_DT_SPEC_GET(LIS3DH_NODE, int_gpios);

struct gpio_callback lis3dh_gpio_cb;

enum lis3dh_mode lis3dh_mode = LIS3DH_MODE_HIGH_RES;
enum lis3dh_scale lis3dh_scale = LIS3DH_SCALE_2G;
enum lis3dh_rate lis3dh_rate = LIS3DH_RATE_100_HZ;
enum quantize lis3dh_quant = QUANTIZE_0_1;
uint8_t lis3dh_watermark = 30;

static float lis3dh_mg_per_lsb_table[] = {0.0625f, 0.125f, 0.25f, 0.75f};
static uint16_t lis3dh_samples_per_sec_table[] = {0, 1, 10, 25, 50, 100, 200, 400, 1600, 5000};

static void lis3dh_read_fifo(int samples, bool log)
{
	// LOG_INF("lis3dh_read_fifo: samples=%d", samples);

	uint8_t len = 1 + samples * 3 * sizeof(int16_t);
	uint8_t tx_buf[len];
	uint8_t rx_buf[len];

	tx_buf[0] = LIS3DH_REG_OUT_X_L | 0x80 | 0x40;

	struct spi_buf tx0 = {.buf = tx_buf, .len = len};
	struct spi_buf rx0 = {.buf = rx_buf, .len = len};
	struct spi_buf_set txs = {.buffers = &tx0, .count = 1};
	struct spi_buf_set rxs = {.buffers = &rx0, .count = 1};

	spi_transceive_dt(&lis3dh, &txs, &rxs);

	spi_write_uint8(&lis3dh, LIS3DH_REG_FIFO_CTRL, LIS3DH_REG_FIFO_CTRL_FM_BYPASS);
	spi_write_uint8(&lis3dh, LIS3DH_REG_FIFO_CTRL,
			LIS3DH_REG_FIFO_CTRL_FM_FIFO | lis3dh_watermark);

	// LOG_INF("reading %d samples from fifo, log_size=%d", samples);

	float mg_scale = lis3dh_mg_per_lsb_table[lis3dh_scale];
	// LOG_INF("mg_scale %f", (double)mg_scale);

	uint64_t timestamp = channel_timestamp();
	uint32_t samples_per_sec = lis3dh_samples_per_sec_table[lis3dh_rate];

	// Errata: we skip the first sample of each FIFO as it's consistently invalid.

	if (channel_start_packet(CHANNEL_ACCEL_X, lis3dh_quant, timestamp, samples_per_sec, samples - 1)) {
		for (int i = 1; i < samples; i++) {
			int offset = 1 + i * 6;
			int16_t value = (int16_t)(rx_buf[offset + 1] << 8 | rx_buf[offset + 0]);
			channel_add_packet_sample(value * mg_scale);
		}
		channel_finish_packet();
	}

	if (channel_start_packet(CHANNEL_ACCEL_Y, lis3dh_quant, timestamp, samples_per_sec, samples - 1)) {
		for (int i = 1; i < samples; i++) {
			int offset = 1 + i * 6;
			int16_t value = (int16_t)(rx_buf[offset + 3] << 8 | rx_buf[offset + 2]);
			channel_add_packet_sample(value * mg_scale);
		}
		channel_finish_packet();
	}

	if (channel_start_packet(CHANNEL_ACCEL_Z, lis3dh_quant, timestamp, samples_per_sec, samples - 1)) {
		for (int i = 1; i < samples; i++) {
			int offset = 1 + i * 6;
			int16_t value = (int16_t)(rx_buf[offset + 5] << 8 | rx_buf[offset + 4]);
			channel_add_packet_sample(value * mg_scale);
		}
		channel_finish_packet();
	}
}

static void lis3dh_thread_main(void *, void *, void *)
{
	while (1) {
		uint32_t samples_per_sec = lis3dh_samples_per_sec_table[lis3dh_rate];
		// LOG_INF("samples_per_sec: %d", samples_per_sec);
		uint32_t fifo_fill_usec = samples_per_sec ? lis3dh_watermark * 1000000 / samples_per_sec : 1000000;
		// LOG_INF("fifo_fill_usec: %d", fifo_fill_usec);
		k_usleep(fifo_fill_usec);

		uint8_t fifo_src = spi_read_uint8(&lis3dh, LIS3DH_REG_FIFO_SRC);
		uint8_t ovrn_fifo = fifo_src & 0x40;
		uint8_t fss = fifo_src & 0x1F;
		// uint8_t wtm = fifo_src & 0x80;
		// uint8_t empty = fifo_src & 0x20;
		// LOG_INF("FIFO_SRC: 0x%02x wtm=%x ovrn_fifo=%x empty=%x, fss=%x", fifo_src, wtm,
		// ovrn_fifo, empty, fss);

		if (ovrn_fifo) {
			LOG_WRN("fifo overrun");
		}

		if (fss > 0) {
			lis3dh_read_fifo(fss, true);
		}
	}
}

K_THREAD_STACK_DEFINE(lis3dh_thread_stack, 1024);
struct k_thread lis3dh_thread;

static void lis3dh_int_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	uint8_t src = spi_read_uint8(&lis3dh, LIS3DH_REG_INT1_SRC);
	(void)src;

	// LOG_INF("INT triggered by %x", src);
}

static void lis3dh_config(void)
{
	uint8_t config1 = (lis3dh_rate << 4) | LIS3DH_REG_CTRL_REG1_ZEN | LIS3DH_REG_CTRL_REG1_YEN |
			  LIS3DH_REG_CTRL_REG1_XEN |
			  (lis3dh_mode == LIS3DH_MODE_LOW_POWER ? LIS3DH_REG_CTRL_REG1_LPEN : 0);
	uint8_t config2 = 0;
	uint8_t config3 = 0;
	uint8_t config4 = (lis3dh_scale << 4) |
			  (lis3dh_mode == LIS3DH_MODE_HIGH_RES ? LIS3DH_REG_CTRL_REG4_HR : 0);
	uint8_t config5 = LIS3DH_REG_CTRL_REG5_FIFO_EN;
	uint8_t fifoctrl = LIS3DH_REG_FIFO_CTRL_FM_FIFO | lis3dh_watermark;

	spi_write_uint8(&lis3dh, LIS3DH_REG_CTRL_REG1, config1);
	spi_write_uint8(&lis3dh, LIS3DH_REG_CTRL_REG2, config2);
	spi_write_uint8(&lis3dh, LIS3DH_REG_CTRL_REG3, config3);
	spi_write_uint8(&lis3dh, LIS3DH_REG_CTRL_REG4, config4);
	spi_write_uint8(&lis3dh, LIS3DH_REG_CTRL_REG5, config5);
	spi_write_uint8(&lis3dh, LIS3DH_REG_FIFO_CTRL, fifoctrl);

	// Drain and discard any existing samples in FIFO
	lis3dh_read_fifo(32, false);
}

void lis3dh_init(void)
{
	uint8_t id = spi_read_uint8(&lis3dh, LIS3DH_REG_WHO_AM_I);
	if (id != 0x33) {
		LOG_ERR("device not detected!");
		return;
	}

	spi_read_uint8(&lis3dh, LIS3DH_REG_INT1_SRC); // Clear any pending interrupts

	gpio_pin_configure_dt(&lis3dh_int, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&lis3dh_int, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&lis3dh_gpio_cb, lis3dh_int_handler, BIT(lis3dh_int.pin));
	gpio_add_callback(lis3dh_int.port, &lis3dh_gpio_cb);

	lis3dh_config();

	k_thread_create(&lis3dh_thread, lis3dh_thread_stack,
			K_THREAD_STACK_SIZEOF(lis3dh_thread_stack), lis3dh_thread_main, NULL, NULL,
			NULL, 7, 0, K_NO_WAIT);

	LOG_INF("initialized");
}

void lis3dh_latest(int16_t *x, int16_t *y, int16_t *z)
{
	// *x = log_x[log_accel_count - 1];
	// *y = log_y[log_accel_count - 1];
	// *z = log_z[log_accel_count - 1];
}

static const char *lis3dh_mode_names[] = {"low_power", "normal", "high_res"};

static const char *lis3dh_scale_names[] = {"2g", "4g", "8g", "16g"};

static const char *lis3dh_rate_names[] = {"0hz",   "1hz",   "10hz",  "25hz",   "50hz",
					  "100hz", "200hz", "400hz", "1.6khz", "5khz"};

static int cmd_lis3dh_mode(const struct shell *shell, size_t argc, char *argv[])
{
	int index =
		cmd_table_lookup(shell, lis3dh_mode_names, ARRAY_SIZE(lis3dh_mode_names), argv[1]);
	if (index < 0) {
		return -1;
	}
	lis3dh_mode = (enum lis3dh_mode)index;
	lis3dh_config();
	return 0;
}

static int cmd_lis3dh_scale(const struct shell *shell, size_t argc, char *argv[])
{
	int index = cmd_table_lookup(shell, lis3dh_scale_names, ARRAY_SIZE(lis3dh_scale_names),
				     argv[1]);
	if (index < 0) {
		return -1;
	}
	lis3dh_scale = (enum lis3dh_scale)index;
	lis3dh_config();
	return 0;
}

static int cmd_lis3dh_rate(const struct shell *shell, size_t argc, char *argv[])
{
	int index =
		cmd_table_lookup(shell, lis3dh_rate_names, ARRAY_SIZE(lis3dh_rate_names), argv[1]);
	if (index < 0) {
		return -1;
	}
	lis3dh_rate = (enum lis3dh_rate)index;
	lis3dh_config();
	return 0;
}

static int cmd_lis3dh_quant(const struct shell *shell, size_t argc, char *argv[])
{
    int index =
        cmd_table_lookup(shell, quantize_names, QUANTIZE_COUNT, argv[1]);
    if (index < 0) {
        return -1;
    }
    lis3dh_quant = (enum quantize)index;
    return 0;
}

static int cmd_lis3dh_status(const struct shell *shell, size_t argc, char *argv[])
{
	shell_fprintf(shell, SHELL_NORMAL, "LIS3DH status:\n");
	shell_fprintf(shell, SHELL_NORMAL, " mode: %s\n", lis3dh_mode_names[lis3dh_mode]);
	shell_fprintf(shell, SHELL_NORMAL, " scale: %s\n", lis3dh_scale_names[lis3dh_scale]);
	shell_fprintf(shell, SHELL_NORMAL, " rate: %s\n", lis3dh_rate_names[lis3dh_rate]);
	shell_fprintf(shell, SHELL_NORMAL, " quant: %s\n", quantize_names[lis3dh_quant]);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	lis3dh_cmds, SHELL_CMD_ARG(mode, NULL, "low_power|normal|high_res", cmd_lis3dh_mode, 2, 0),
	SHELL_CMD_ARG(scale, NULL, "2g|4g|8g|16g", cmd_lis3dh_scale, 2, 0),
	SHELL_CMD_ARG(rate, NULL, "0hz|1hz|10hz|25hz|50hz|100hz|200hz|400hz|1.6khz|5khz",
		      cmd_lis3dh_rate, 2, 0),
	SHELL_CMD_ARG(quant, NULL, QUANTIZE_HELP, cmd_lis3dh_quant, 2, 0),
	SHELL_CMD_ARG(status, NULL, "print device status", cmd_lis3dh_status, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lis3dh, &lis3dh_cmds, "LIS3DH MEMS 3-axis \"nano\" accelerometer", NULL);
