#ifndef COMMON_H
#define COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

enum channel
{
    CHANNEL_NULL,
    CHANNEL_ACCEL_X,
    CHANNEL_ACCEL_Y,
    CHANNEL_ACCEL_Z,
    CHANNEL_TEMPERATURE,
    CHANNEL_PRESSURE,
    CHANNEL_COUNT
};

enum quantize
{
    QUANTIZE_1_0,
    QUANTIZE_0_1,
    QUANTIZE_0_01,
    QUANTIZE_0_001,
    QUANTIZE_0_0001,
    QUANTIZE_COUNT
};

extern const char *quantize_names[QUANTIZE_COUNT];
#define QUANTIZE_HELP "1.0|0.1|0.01|0.001|0.0001"

int cmd_table_lookup(const struct shell *shell, const char **table, size_t table_size, const char *value);

uint8_t spi_read_uint8(const struct spi_dt_spec *spec, uint8_t reg);
void spi_write_uint8(const struct spi_dt_spec *spec, uint8_t reg, uint8_t val);

uint64_t channel_timestamp();

bool channel_start_packet(enum channel ch, enum quantize quant, uint64_t ts, uint16_t rate, uint16_t sample_count);
void channel_add_packet_sample(float s);
void channel_finish_packet();

void dps368_init(void);
void dps368_latest(float *temperature, float *pressure);
void dps368_stop(void);

void lis3dh_init(void);
void lis3dh_latest(float *x, float *y, float *z);
void lis3dh_wake_on_z(void);

#endif
