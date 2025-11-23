#ifndef COMMON_H
#define COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <math.h>

uint8_t spi_read_uint8(const struct spi_dt_spec *spec, uint8_t reg);
int16_t spi_read_int16(const struct spi_dt_spec *spec, uint8_t reg);
void spi_read_buf(const struct spi_dt_spec *spec, uint8_t reg, uint8_t length, uint8_t *output);
void spi_write_uint8(const struct spi_dt_spec *spec, uint8_t reg, uint8_t val);

int32_t quantf(float v, float step);

uint64_t timestamp();

bool start_packet(const char *type, uint64_t ts, uint32_t rate, float step);
void add_packet_sample(float s);
void finish_packet();

#endif
