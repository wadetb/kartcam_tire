#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "dps368.h"
#include "common.h"

extern struct spi_dt_spec dps368;

LOG_MODULE_REGISTER(DPS368);

float c0Half;
float c1;
float c00;
float c10;
float c20;
float c30;
float c01;
float c11;
float c21;

float scaling_facts[] = {524288.0f, 1572864.0f, 3670016.0f, 7864320.0f, 253952.0f, 516096.0f, 1040384.0f, 2088960.0f};
int tmp_osr;
int prs_osr;

bool dps368_is_present(void)
{
    uint8_t id = spi_read_uint8(&dps368, DPS368_REG_PRODUCT_ID);
    return id == 0x10;
}

int32_t twoc(uint32_t value, uint8_t bits)
{
    if (value & (1u << (bits - 1))) {
        value = value - (1u << bits);
    }
    return (int32_t)value;
}

void dps368_read_coefs(void)
{
    uint8_t coef[18];
    for (int i = 0; i < 18; i++) {
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
    if (!dps368_is_present()) {
        LOG_ERR("DPS368 not detected!");
        return;
    }

    dps368_read_coefs();

    tmp_osr = DPS368_REG_TMP_CFG_PRC_4;
    prs_osr = DPS368_REG_PRS_CFG_PRC_4;

    uint8_t tmpcfg = tmp_osr | DPS368_REG_TMP_CFG_RATE_4 | DPS368_REG_TMP_CFG_TMP_EXT;
    spi_write_uint8(&dps368, DPS368_REG_TMP_CFG, tmpcfg);

    uint8_t prscfg = prs_osr | DPS368_REG_PRS_CFG_RATE_4;
    spi_write_uint8(&dps368, DPS368_REG_PRS_CFG, prscfg);
}

void dps368_read(float *temperature, float *pressure)
{
    if (!dps368_is_present()) {
        LOG_ERR("DPS368 not detected!");
        return;
    }

    // uint8_t meas_cfg = spi_read_uint8(&dps368, DPS368_REG_MEAS_CFG);
    // return (meas_cfg & DPS368_REG_MEAS_CFG_SENSOR_RDY) != 0;

    spi_write_uint8(&dps368, DPS368_REG_MEAS_CFG, DPS368_REG_MEAS_CFG_MEAS_CTRL_CMD_TMP);

    uint8_t tmp[3];
    tmp[0] = spi_read_uint8(&dps368, DPS368_REG_TMP_B2);
    tmp[1] = spi_read_uint8(&dps368, DPS368_REG_TMP_B1);
    tmp[2] = spi_read_uint8(&dps368, DPS368_REG_TMP_B0);
    // spi_read_buf(&dps368, DPS368_REG_TMP_B2, 3, tmp);
    float tmp_raw = twoc(((uint32_t)tmp[0] << 16) | ((uint32_t)tmp[1] << 8) | (uint32_t)tmp[2], 24);

    spi_write_uint8(&dps368, DPS368_REG_MEAS_CFG, DPS368_REG_MEAS_CFG_MEAS_CTRL_CMD_PRS);

    uint8_t prs[3];
    prs[0] = spi_read_uint8(&dps368, DPS368_REG_PRS_B2);
    prs[1] = spi_read_uint8(&dps368, DPS368_REG_PRS_B1);
    prs[2] = spi_read_uint8(&dps368, DPS368_REG_PRS_B0);
    // spi_read_buf(&dps368, DPS368_REG_PRS_B2, 3, prs);
    float prs_raw = twoc(((uint32_t)prs[0] << 16) | ((uint32_t)prs[1] << 8) | (uint32_t)prs[2], 24);

    float tmp_sc = tmp_raw / scaling_facts[tmp_osr];
	float tmp_comp = c0Half + c1 * tmp_sc;

	float prs_sc = prs_raw / scaling_facts[prs_osr];
	float prs_comp = c00 + prs_sc * (c10 + prs_sc * (c20 + prs_sc * c30)) + 
                           tmp_sc * (c01 + prs_sc * (c11 + prs_sc * c21));

    *temperature = tmp_comp;
    *pressure = prs_comp;
}
