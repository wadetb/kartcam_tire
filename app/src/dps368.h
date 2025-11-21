#ifndef DPS368_H
#define DPS368_H

#include <stdint.h>

#define DPS368_REG_PRS_B2 (0x00) // Pressure data byte 2
#define DPS368_REG_PRS_B1 (0x01) // Pressure data byte 1
#define DPS368_REG_PRS_B0 (0x02) // Pressure data byte 0
#define DPS368_REG_TMP_B2 (0x03) // Temperature data byte 2
#define DPS368_REG_TMP_B1 (0x04) // Temperature data byte 1
#define DPS368_REG_TMP_B0 (0x05) // Temperature data byte 0
#define DPS368_REG_PRS_CFG (0x06) // Pressure configuration register
#define DPS368_REG_TMP_CFG (0x07) // Temperature configuration register
#define DPS368_REG_MEAS_CFG (0x08) // Measurement configuration register
#define DPS368_REG_CFG_REG (0x09) // Configuration register
#define DPS368_REG_INT_STS (0x0A) // Interrupt and status register
#define DPS368_REG_FIFO_STS (0x0B) // FIFO status register
#define DPS368_REG_RESET (0x0C) // Reset register
#define DPS368_REG_PRODUCT_ID (0x0D) // Product ID register
#define DPS368_REG_COEF (0x10) // Coefficient start register
#define DPS368_REG_COEF_SRCE (0x28) // Coefficient source register

#define DPS368_REG_PRS_CFG_PRC_MASK (0x07)
#define DPS368_REG_PRS_CFG_PRC_1 (0x00)
#define DPS368_REG_PRS_CFG_PRC_2 (0x01)
#define DPS368_REG_PRS_CFG_PRC_4 (0x02)
#define DPS368_REG_PRS_CFG_PRC_8 (0x03)
#define DPS368_REG_PRS_CFG_PRC_16 (0x04)
#define DPS368_REG_PRS_CFG_PRC_32 (0x05)
#define DPS368_REG_PRS_CFG_PRC_64 (0x06)
#define DPS368_REG_PRS_CFG_PRC_128 (0x07)
#define DPS368_REG_PRS_CFG_RATE_MASK (0x70)
#define DPS368_REG_PRS_CFG_RATE_1 (0x00)
#define DPS368_REG_PRS_CFG_RATE_2 (0x10)
#define DPS368_REG_PRS_CFG_RATE_4 (0x20)
#define DPS368_REG_PRS_CFG_RATE_8 (0x30)
#define DPS368_REG_PRS_CFG_RATE_16 (0x40)
#define DPS368_REG_PRS_CFG_RATE_32 (0x50)
#define DPS368_REG_PRS_CFG_RATE_64 (0x60)
#define DPS368_REG_PRS_CFG_RATE_128 (0x70)

#define DPS368_REG_TMP_CFG_PRC_MASK (0x07)
#define DPS368_REG_TMP_CFG_PRC_1 (0x00)
#define DPS368_REG_TMP_CFG_PRC_2 (0x01)
#define DPS368_REG_TMP_CFG_PRC_4 (0x02)
#define DPS368_REG_TMP_CFG_PRC_8 (0x03)
#define DPS368_REG_TMP_CFG_PRC_16 (0x04)
#define DPS368_REG_TMP_CFG_PRC_32 (0x05)
#define DPS368_REG_TMP_CFG_PRC_64 (0x06)
#define DPS368_REG_TMP_CFG_PRC_128 (0x07)
#define DPS368_REG_TMP_CFG_RATE_MASK (0x70)
#define DPS368_REG_TMP_CFG_RATE_1 (0x00)
#define DPS368_REG_TMP_CFG_RATE_2 (0x10)
#define DPS368_REG_TMP_CFG_RATE_4 (0x20)
#define DPS368_REG_TMP_CFG_RATE_8 (0x30)
#define DPS368_REG_TMP_CFG_RATE_16 (0x40)
#define DPS368_REG_TMP_CFG_RATE_32 (0x50)
#define DPS368_REG_TMP_CFG_RATE_64 (0x60)
#define DPS368_REG_TMP_CFG_RATE_128 (0x70)
#define DPS368_REG_TMP_CFG_TMP_EXT (0x80)

#define DPS368_REG_MEAS_CFG_MEAS_CTRL_MASK (0x07)
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_IDLE (0x00)
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_CMD_PRS (0x01)
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_CMD_TMP (0x02)
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_CONT_PRS (0x05)
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_CONT_TMP (0x06)
#define DPS368_REG_MEAS_CFG_MEAS_CTRL_CONT_BOTH (0x07)
#define DPS368_REG_MEAS_CFG_PRS_RDY (0x10)
#define DPS368_REG_MEAS_CFG_TMP_RDY (0x20)
#define DPS368_REG_MEAS_CFG_SENSOR_RDY (0x40)
#define DPS368_REG_MEAS_CFG_COEFF_RDY (0x80)

#define DPS368_REG_CFG_REG_SPI_MODE_3WIRE (0x01)
#define DPS368_REG_CFG_REG_FIFO_EN (0x02)
#define DPS368_REG_CFG_REG_P_SHIFT (0x04)
#define DPS368_REG_CFG_REG_T_SHIFT (0x08)
#define DPS368_REG_CFG_REG_INT_PRS (0x10)
#define DPS368_REG_CFG_REG_INT_TMP (0x20)
#define DPS368_REG_CFG_REG_INT_FIFO (0x40)
#define DPS368_REG_CFG_REG_INT_HL (0x80)

#define DPS368_REG_INT_STS_PRS_RDY (0x01)
#define DPS368_REG_INT_STS_TMP_RDY (0x02)
#define DPS368_REG_INT_STS_FIFO_FULL (0x04)

#define DPS368_REG_FIFO_STS_FIFO_EMPTY (0x01)
#define DPS368_REG_FIFO_STS_FIFO_FULL (0x02)

#define DPS368_REG_RESET_SOFT_RESET (0x09)
#define DPS368_REG_RESET_FIFO_FLUSH (0x80)

#define DPS368_REG_PRODUCT_ID_PROD_ID_MASK (0x0F)
#define DPS368_REG_PRODUCT_ID_REV_ID_MASK (0xF0)

#define DPS368_REG_TMP_COEF_SRCE_TEMP_SENSOR_EXTERNAL (0x80)
#define DPS368_REG_TMP_COEF_SRCE_TEMP_SENSOR_INTERNAL (0x00)

bool dps368_is_present(void);
void dps368_init(void);
void dps368_latest(float *temperature, float *pressure);

#endif
