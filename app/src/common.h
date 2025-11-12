#ifndef COMMON_H
#define COMMON_H

uint8_t spi_read_uint8(const struct spi_dt_spec *spec, uint8_t reg);
int16_t spi_read_int16(const struct spi_dt_spec *spec, uint8_t reg);
void spi_read_buf(const struct spi_dt_spec *spec, uint8_t reg, uint8_t length, uint8_t *output);
void spi_write_uint8(const struct spi_dt_spec *spec, uint8_t reg, uint8_t val);

#endif
