#include "common.h"

//
// TODO:
// + Fix fifo overrun logging
// + Use P and T shift
// + Adjust sleep timers for oversampling?
//

LOG_MODULE_REGISTER(dps368);

#define DPS368_REG_PRS_B2                       0x00
#define DPS368_REG_PRS_B1                       0x01
#define DPS368_REG_PRS_B0                       0x02

#define DPS368_REG_PRS_CFG                      0x06

#define DPS368_REG_TMP_CFG                      0x07
#define DPS368_REG_TMP_CFG_TMP_EXT              BIT(7)

#define DPS368_REG_MEAS_CFG                     0x08
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_CONT_BOTH BIT_MASK(3)
#define DPS368_REG_MEAS_CFG_COEFF_RDY           BIT(7)

#define DPS368_REG_CFG_REG                      0x09
#define DPS368_REG_CFG_REG_FIFO_EN              0x02
#define DPS368_REG_CFG_REG_P_SHIFT              0x04
#define DPS368_REG_CFG_REG_T_SHIFT              0x08

#define DPS368_REG_FIFO_STS                     0x0B
#define DPS368_REG_FIFO_STS_FIFO_FULL           BIT(1)

#define DPS368_REG_PRODUCT_ID                   0x0D
#define DPS368_REG_COEF                         0x10

enum dps368_rate {
	DPS368_RATE_1_HZ,
	DPS368_RATE_2_HZ,
	DPS368_RATE_4_HZ,
	DPS368_RATE_8_HZ,
	DPS368_RATE_16_HZ,
	DPS368_RATE_32_HZ,
	DPS368_RATE_64_HZ,
	DPS368_RATE_128_HZ
};

enum dps368_oversampling {
	DPS368_OSR_1,
	DPS368_OSR_2,
	DPS368_OSR_4,
	DPS368_OSR_8,
	DPS368_OSR_16,
	DPS368_OSR_32,
	DPS368_OSR_64,
	DPS368_OSR_128
};

#define DPS368_NODE DT_NODELABEL(dps368)

#define DPS368_OP                                                                                  \
	SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

struct spi_dt_spec dps368 = SPI_DT_SPEC_GET(DPS368_NODE, DPS368_OP);

static float c0Half, c1;
static float c00, c10, c20, c30;
static float c01, c11, c21;

const float dps368_scaling_facts[] = {524288.0f, 1572864.0f, 3670016.0f, 7864320.0f,
				      253952.0f, 516096.0f,  1040384.0f, 2088960.0f};

enum dps368_rate dps368_prs_rate = DPS368_RATE_64_HZ;
enum dps368_rate dps368_tmp_rate = DPS368_RATE_8_HZ;

enum dps368_oversampling dps368_tmp_osr = DPS368_OSR_1;
enum dps368_oversampling dps368_prs_osr = DPS368_OSR_1;

enum quantize dps368_tmp_quant = QUANTIZE_0_1;
enum quantize dps368_prs_quant = QUANTIZE_0_1;

uint8_t dps368_watermark = 30;

float dps368_latest_tmp_sc;
float dps368_latest_tmp_comp;
float dps368_latest_prs_comp;

static uint32_t dps368_samples_per_sec(enum dps368_rate rate)
{
	return 1 << rate;
}

static int32_t twoc(uint32_t value, uint8_t bits)
{
	if (value & (1u << (bits - 1))) {
		value = value - (1u << bits);
	}
	return (int32_t)value;
}

static void dps368_read_fifo()
{
	// LOG_INF("dps368_read_fifo");

	// FIXME- not working?
	// uint8_t fifo_sts = spi_read_uint8(&dps368, DPS368_REG_FIFO_STS);
	// if (fifo_sts & DPS368_REG_FIFO_STS_FIFO_FULL)
	// {
	//     LOG_WRN("fifo overrun");
	// }

	int prs_count = 0;
	int tmp_count = 0;
	float prs_buf[32];
	float tmp_buf[32];

	for (;;) {
		uint8_t value[3];
		value[0] = spi_read_uint8(&dps368, DPS368_REG_PRS_B2);
		value[1] = spi_read_uint8(&dps368, DPS368_REG_PRS_B1);
		value[2] = spi_read_uint8(&dps368, DPS368_REG_PRS_B0);

		// LOG_INF("raw bytes: %02x %02x %02x", value[0], value[1], value[2]);

		if (value[0] == 0x80 && value[1] == 0x00 && value[2] == 0x00) {
			break;
		}

		uint32_t mode = value[2] & 1;

		int32_t raw = 0;
		raw |= (uint32_t)value[0] << 16;
		raw |= (uint32_t)value[1] << 8;
		raw |= (uint32_t)value[2] & 0xfe;
		raw = twoc(raw, 24);
		// LOG_INF("raw: %x", raw);

		if (mode) {
			int32_t prs_raw = raw;
			float prs_sc = (float)prs_raw / dps368_scaling_facts[dps368_prs_osr];
			float prs_comp =
				c00 + prs_sc * (c10 + prs_sc * (c20 + prs_sc * c30)) +
				dps368_latest_tmp_sc * (c01 + prs_sc * (c11 + prs_sc * c21));
			// LOG_INF("prs_raw: %x prs_sc: %f prs_comp: %f", prs_raw, (double)prs_sc,
			// (double)prs_comp);
			prs_buf[prs_count] = prs_comp;
			prs_count++;
			dps368_latest_prs_comp = prs_comp;
		} else {
			int32_t tmp_raw = raw;
			float tmp_sc = (float)tmp_raw / dps368_scaling_facts[dps368_tmp_osr];
			float tmp_comp = c0Half + c1 * tmp_sc;
			// LOG_INF("tmp_raw: %x tmp_sc: %f tmp_comp: %f", tmp_raw, (double)tmp_sc,
			// (double)tmp_comp);
			tmp_buf[tmp_count] = tmp_comp;
			tmp_count++;
			dps368_latest_tmp_sc = tmp_sc;
			dps368_latest_tmp_comp = tmp_comp;
		}
	}

	uint64_t timestamp = channel_timestamp();

	if (prs_count > 0) {
		uint32_t samples_per_sec = dps368_samples_per_sec(dps368_prs_rate);
		if (channel_start_packet(CHANNEL_PRESSURE, dps368_prs_quant, timestamp, samples_per_sec,
				 prs_count)) {
			for (int i = 0; i < prs_count; i++) {
				channel_add_packet_sample(prs_buf[i]);
			}
			channel_finish_packet();
		}
	}

	if (tmp_count > 0) {
		uint32_t samples_per_sec = dps368_samples_per_sec(dps368_tmp_rate);
		if (channel_start_packet(CHANNEL_TEMPERATURE, dps368_tmp_quant, timestamp,
				 samples_per_sec, tmp_count)) {
			for (int i = 0; i < tmp_count; i++) {
				channel_add_packet_sample(tmp_buf[i]);
			}
			channel_finish_packet();
		}
	}
}

K_THREAD_STACK_DEFINE(dps368_thread_stack, 1024);
struct k_thread dps368_thread;

static void dps368_thread_main(void *, void *, void *)
{
	while (1) {
		uint32_t prs_samples_per_sec = dps368_samples_per_sec(dps368_prs_rate);
		uint32_t tmp_samples_per_sec = dps368_samples_per_sec(dps368_tmp_rate);
		uint32_t sleep_usec =
			dps368_watermark * 1000000 / (prs_samples_per_sec + tmp_samples_per_sec);
		// LOG_INF("sleep_usec: %d", sleep_usec);
		k_usleep(sleep_usec);

		dps368_read_fifo();
	}
}

static void dps368_read_coefs(void)
{
	uint8_t coef[18];
	for (int i = 0; i < 18; i++) {
		coef[i] = spi_read_uint8(&dps368, DPS368_REG_COEF + i);
	}
	// spi_read_buf(&dps368, DPS368_REG_COEF, 18, coef);

	c0Half = twoc(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12) / 2;
	c1 = twoc((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);

	c00 = twoc(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) |
			   (((uint32_t)coef[5] >> 4) & 0x0F),
		   20);
	c10 = twoc((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) |
			   (uint32_t)coef[7],
		   20);

	c01 = twoc(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
	c11 = twoc(((uint32_t)coef[10] << 8) | (uint32_t)coef[11], 16);
	c20 = twoc(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);
	c21 = twoc(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);
	c30 = twoc(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

	LOG_INF("c0Half: %f c1: %f", (double)c0Half, (double)c1);
	LOG_INF("c00: %f c10: %f c20: %f c30: %f", (double)c00, (double)c10, (double)c20,
		(double)c30);
	LOG_INF("c01: %f c11: %f c21: %f", (double)c01, (double)c11, (double)c21);
}

static void dps368_config(void)
{
	uint8_t tmpcfg = dps368_tmp_osr | (dps368_tmp_rate << 4) | DPS368_REG_TMP_CFG_TMP_EXT;
	uint8_t prscfg = dps368_prs_osr | (dps368_prs_rate << 4);
	uint8_t cfg_reg = DPS368_REG_CFG_REG_FIFO_EN;
	uint8_t meas_cfg = DPS368_REG_MEAS_CFG_MEAS_CTRL_CONT_BOTH;

	spi_write_uint8(&dps368, DPS368_REG_TMP_CFG, tmpcfg);
	spi_write_uint8(&dps368, DPS368_REG_PRS_CFG, prscfg);
	spi_write_uint8(&dps368, DPS368_REG_CFG_REG, cfg_reg);
	spi_write_uint8(&dps368, DPS368_REG_MEAS_CFG, meas_cfg);
}

void dps368_init(void)
{
	uint8_t id = spi_read_uint8(&dps368, DPS368_REG_PRODUCT_ID);
	if (id != 0x10) {
		LOG_ERR("device not detected!");
		return;
	}

	dps368_read_coefs();
	dps368_config();

	k_thread_create(&dps368_thread, dps368_thread_stack,
			K_THREAD_STACK_SIZEOF(dps368_thread_stack), dps368_thread_main, NULL, NULL,
			NULL, 7, 0, K_NO_WAIT);

	LOG_INF("initialized");
}

void dps368_latest(float *temperature, float *pressure)
{
	*temperature = dps368_latest_tmp_comp;
	*pressure = dps368_latest_prs_comp;
}

void dps368_stop(void)
{
	k_thread_abort(&dps368_thread);

	spi_write_uint8(&dps368, DPS368_REG_TMP_CFG, 0);
	spi_write_uint8(&dps368, DPS368_REG_PRS_CFG, 0);
	spi_write_uint8(&dps368, DPS368_REG_CFG_REG, 0);
	spi_write_uint8(&dps368, DPS368_REG_MEAS_CFG, 0);
}

static const char *dps368_rate_names[] = {
	"1hz", "2hz", "4hz", "8hz", "16hz", "32hz", "64hz", "128hz",
};

static const char *dps368_osr_names[] = {
	"1", "2", "4", "8", "16", "32", "64", "128",
};

static int cmd_dps368_tmp_rate(const struct shell *shell, size_t argc, char *argv[])
{
	int index =
		cmd_table_lookup(shell, dps368_rate_names, ARRAY_SIZE(dps368_rate_names), argv[1]);
	if (index < 0) {
		return -1;
	}
	dps368_tmp_rate = (enum dps368_rate)index;
	dps368_config();
	return 0;
}

static int cmd_dps368_prs_rate(const struct shell *shell, size_t argc, char *argv[])
{
	int index =
		cmd_table_lookup(shell, dps368_rate_names, ARRAY_SIZE(dps368_rate_names), argv[1]);
	if (index < 0) {
		return -1;
	}
	dps368_prs_rate = (enum dps368_rate)index;
	dps368_config();
	return 0;
}

static int cmd_dps368_tmp_osr(const struct shell *shell, size_t argc, char *argv[])
{
	int index =
		cmd_table_lookup(shell, dps368_osr_names, ARRAY_SIZE(dps368_osr_names), argv[1]);
	if (index < 0) {
		return -1;
	}
	dps368_tmp_osr = (enum dps368_oversampling)index;
	dps368_config();
	return 0;
}

static int cmd_dps368_prs_osr(const struct shell *shell, size_t argc, char *argv[])
{
	int index =
		cmd_table_lookup(shell, dps368_osr_names, ARRAY_SIZE(dps368_osr_names), argv[1]);
	if (index < 0) {
		return -1;
	}
	dps368_prs_osr = (enum dps368_oversampling)index;
	dps368_config();
	return 0;
}

static int cmd_dps368_tmp_quant(const struct shell *shell, size_t argc, char *argv[])
{
	int index = cmd_table_lookup(shell, quantize_names, QUANTIZE_COUNT, argv[1]);
	if (index < 0) {
		return -1;
	}
	dps368_tmp_quant = (enum quantize)index;
	return 0;
}

static int cmd_dps368_prs_quant(const struct shell *shell, size_t argc, char *argv[])
{
	int index = cmd_table_lookup(shell, quantize_names, QUANTIZE_COUNT, argv[1]);
	if (index < 0) {
		return -1;
	}
	dps368_prs_quant = (enum quantize)index;
	return 0;
}

static int cmd_dps368_status(const struct shell *shell, size_t argc, char *argv[])
{
	shell_fprintf(shell, SHELL_NORMAL, "DPS368 status:\n");
	shell_fprintf(shell, SHELL_NORMAL, " tmp_rate: %s\n", dps368_rate_names[dps368_tmp_rate]);
	shell_fprintf(shell, SHELL_NORMAL, " prs_rate: %s\n", dps368_rate_names[dps368_prs_rate]);
	shell_fprintf(shell, SHELL_NORMAL, " tmp_osr: %s\n", dps368_osr_names[dps368_tmp_osr]);
	shell_fprintf(shell, SHELL_NORMAL, " prs_osr: %s\n", dps368_osr_names[dps368_prs_osr]);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	dps368_cmds,
	SHELL_CMD_ARG(tmp_rate, NULL, "1hz|2hz|4hz|8hz|16hz|32hz|64hz|128hz", cmd_dps368_tmp_rate,
		      2, 0),
	SHELL_CMD_ARG(prs_rate, NULL, "1hz|2hz|4hz|8hz|16hz|32hz|64hz|128hz", cmd_dps368_prs_rate,
		      2, 0),
	SHELL_CMD_ARG(tmp_osr, NULL, "1|2|4|8|16|32|64|128", cmd_dps368_tmp_osr, 2, 0),
	SHELL_CMD_ARG(prs_osr, NULL, "1|2|4|8|16|32|64|128", cmd_dps368_prs_osr, 2, 0),
	SHELL_CMD_ARG(tmp_quant, NULL, QUANTIZE_HELP, cmd_dps368_tmp_quant, 2, 0),
	SHELL_CMD_ARG(prs_quant, NULL, QUANTIZE_HELP, cmd_dps368_prs_quant, 2, 0),
	SHELL_CMD_ARG(status, NULL, "print device status", cmd_dps368_status, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dps368, &dps368_cmds, "DPS368 Digital XENSIV(TM) barometric pressure sensor",
		   NULL);
