#include "common.h"

#include "dps368.h"

extern struct spi_dt_spec dps368;

LOG_MODULE_REGISTER(dps368);

float c0Half;
float c1;
float c00;
float c10;
float c20;
float c30;
float c01;
float c11;
float c21;

const float scaling_facts[] = {524288.0f, 1572864.0f, 3670016.0f, 7864320.0f, 253952.0f, 516096.0f, 1040384.0f, 2088960.0f};

uint32_t tmp_osr = DPS368_REG_TMP_CFG_PRC_1;
uint32_t prs_osr = DPS368_REG_TMP_CFG_PRC_1;

uint8_t prs_rate = DPS368_REG_PRS_CFG_RATE_64;
uint8_t tmp_rate = DPS368_REG_TMP_CFG_RATE_8;
uint8_t prs_tmp_watermark = 30;

float tmp_step = 0.001f; // degrees C
float prs_step = 0.01f;  // Pa

int32_t tmp_raw;
int32_t prs_raw;
float tmp_sc;
float prs_sc;
float tmp_comp;
float prs_comp;

#define LOG_SIZE 40

int log_prs_count;
int log_tmp_count;
float log_prs[LOG_SIZE];
float log_tmp[LOG_SIZE];

static uint32_t dps368_samples_per_sec(uint8_t rate)
{
    // Note; could use bit manipulation to derive this generally.
    switch (rate)
    {
    case DPS368_REG_TMP_CFG_RATE_1:
        return 1;
    case DPS368_REG_TMP_CFG_RATE_2:
        return 2;
    case DPS368_REG_TMP_CFG_RATE_4:
        return 4;
    case DPS368_REG_TMP_CFG_RATE_8:
        return 8;
    case DPS368_REG_TMP_CFG_RATE_16:
        return 16;
    case DPS368_REG_TMP_CFG_RATE_32:
        return 32;
    case DPS368_REG_TMP_CFG_RATE_64:
        return 64;
    case DPS368_REG_TMP_CFG_RATE_128:
        return 128;
    default:
        return 1;
    }
}

static int32_t twoc(uint32_t value, uint8_t bits)
{
    if (value & (1u << (bits - 1)))
    {
        value = value - (1u << bits);
    }
    return (int32_t)value;
}

static void dps368_poll()
{
    // LOG_INF("dps368_poll");

    // FIXME- not working?
    // uint8_t fifo_sts = spi_read_uint8(&dps368, DPS368_REG_FIFO_STS);
    // if (fifo_sts & DPS368_REG_FIFO_STS_FIFO_FULL)
    // {
    //     LOG_WRN("fifo overrun");
    // }

    for (;;)
    {
        uint8_t value[3];
        value[0] = spi_read_uint8(&dps368, DPS368_REG_PRS_B2);
        value[1] = spi_read_uint8(&dps368, DPS368_REG_PRS_B1);
        value[2] = spi_read_uint8(&dps368, DPS368_REG_PRS_B0);
        // spi_read_buf(&dps368, DPS368_REG_PRS_B2, 3, prs);

        // LOG_INF("raw bytes: %02x %02x %02x", value[0], value[1], value[2]);

        if (value[0] == 0x80 && value[1] == 0x00 && value[2] == 0x00)
            break;

        uint32_t mode = value[2] & 1;

        int32_t raw = 0;
        raw |= (uint32_t)value[0] << 16;
        raw |= (uint32_t)value[1] << 8;
        raw |= (uint32_t)value[2] & 0xfe;
        raw = twoc(raw, 24);
        // LOG_INF("raw: %x", raw);

        if (mode)
        {
            prs_raw = raw;
            prs_sc = (float)prs_raw / scaling_facts[prs_osr];
            prs_comp = c00 + prs_sc * (c10 + prs_sc * (c20 + prs_sc * c30)) +
                       tmp_sc * (c01 + prs_sc * (c11 + prs_sc * c21));
            // LOG_INF("prs_raw: %x prs_sc: %f prs_comp: %f", prs_raw, (double)prs_sc, (double)prs_comp);
            log_prs[log_prs_count] = prs_comp;
            log_prs_count++;
        }
        else
        {
            tmp_raw = raw;
            tmp_sc = (float)tmp_raw / scaling_facts[tmp_osr];
            tmp_comp = c0Half + c1 * tmp_sc;
            // LOG_INF("tmp_raw: %x tmp_sc: %f tmp_comp: %f", tmp_raw, (double)tmp_sc, (double)tmp_comp);
            log_tmp[log_tmp_count] = tmp_comp;
            log_tmp_count++;
        }
    }

    uint64_t log_timestamp = timestamp();

    if (log_prs_count > 0)
    {
        // LOG_INF("flushing pressure log at %d", log_prs_count);
        
        uint32_t prs_samples_per_sec = dps368_samples_per_sec(prs_rate);
        if (start_packet("dps368.hpa", log_timestamp, prs_samples_per_sec, prs_step))
        {
            for (int i = 0; i < log_prs_count; i++)
                add_packet_sample(log_prs[i]);
            finish_packet();
        }

        log_prs_count = 0;
    }

    if (log_tmp_count > 0)
    {
        // LOG_INF("flushing temperature log at %d", log_tmp_count);
        uint32_t tmp_samples_per_sec = dps368_samples_per_sec(tmp_rate);
        if (start_packet("dps368.temp", log_timestamp, tmp_samples_per_sec, tmp_step))
        {
            for (int i = 0; i < log_tmp_count; i++)
                add_packet_sample(log_tmp[i]);
            finish_packet();
        }
        
        log_tmp_count = 0;
    }
}

K_THREAD_STACK_DEFINE(dps368_thread_stack, 1024);
struct k_thread dps368_thread;

static void dps368_thread_main(void *, void *, void *)
{
    while (1)
    {
        uint32_t prs_samples_per_sec = dps368_samples_per_sec(prs_rate);
        uint32_t tmp_samples_per_sec = dps368_samples_per_sec(tmp_rate);
        uint32_t sleep_usec = prs_tmp_watermark * 1000000 / (prs_samples_per_sec + tmp_samples_per_sec);
        // LOG_INF("sleep_usec: %d", sleep_usec);
        k_usleep(sleep_usec);

        dps368_poll();
    }
}

static void dps368_read_coefs(void)
{
    uint8_t coef[18];
    for (int i = 0; i < 18; i++)
    {
        coef[i] = spi_read_uint8(&dps368, DPS368_REG_COEF + i);
    }
    // spi_read_buf(&dps368, DPS368_REG_COEF, 18, coef);

    c0Half = twoc(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12) / 2;
    c1 = twoc((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);

    c00 = twoc(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F), 20);
    c10 = twoc((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 20);

    c01 = twoc(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);
    c11 = twoc(((uint32_t)coef[10] << 8) | (uint32_t)coef[11], 16);
    c20 = twoc(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);
    c21 = twoc(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);
    c30 = twoc(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

    LOG_INF("c0Half: %f c1: %f", (double)c0Half, (double)c1);
    LOG_INF("c00: %f c10: %f c20: %f c30: %f", (double)c00, (double)c10, (double)c20, (double)c30);
    LOG_INF("c01: %f c11: %f c21: %f", (double)c01, (double)c11, (double)c21);
}

void dps368_init(void)
{
    uint8_t id = spi_read_uint8(&dps368, DPS368_REG_PRODUCT_ID);
    if (id != 0x10)
    {
        LOG_ERR("device not detected!");
        return;
    }

    dps368_read_coefs();

    uint8_t tmpcfg = tmp_osr | tmp_rate | DPS368_REG_TMP_CFG_TMP_EXT;
    uint8_t prscfg = prs_osr | prs_rate;
    uint8_t cfg_reg = DPS368_REG_CFG_REG_FIFO_EN;
    uint8_t meas_cfg = DPS368_REG_MEAS_CFG_MEAS_CTRL_CONT_BOTH;

    spi_write_uint8(&dps368, DPS368_REG_TMP_CFG, tmpcfg);
    spi_write_uint8(&dps368, DPS368_REG_PRS_CFG, prscfg);
    spi_write_uint8(&dps368, DPS368_REG_CFG_REG, cfg_reg);
    spi_write_uint8(&dps368, DPS368_REG_MEAS_CFG, meas_cfg);

    k_thread_create(&dps368_thread, dps368_thread_stack,
                    K_THREAD_STACK_SIZEOF(dps368_thread_stack),
                    dps368_thread_main,
                    NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);

    LOG_INF("initialized");
}

void dps368_latest(float *temperature, float *pressure)
{
    *temperature = tmp_comp;
    *pressure = prs_comp;
}
