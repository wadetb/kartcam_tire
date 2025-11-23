#ifndef COMMON_H
#define COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

int cmd_table_lookup(const struct shell *shell, const char **table, size_t table_size, const char *value);

uint8_t spi_read_uint8(const struct spi_dt_spec *spec, uint8_t reg);
void spi_write_uint8(const struct spi_dt_spec *spec, uint8_t reg, uint8_t val);

int32_t quantf(float v, float step);

uint64_t timestamp();

bool start_packet(const char *type, uint64_t ts, uint32_t rate, float step);
void add_packet_sample(float s);
void finish_packet();

void dps368_init(void);
void dps368_latest(float *temperature, float *pressure);

void lis3dh_init(void);
void lis3dh_latest(int16_t *x, int16_t *y, int16_t *z);

#endif
