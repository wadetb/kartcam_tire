#include "common.h"

void spi_write_uint8(const struct spi_dt_spec *spec, uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2] = {reg, val};

    const struct spi_buf tx = {.buf = tx_buf, .len = 2};
    const struct spi_buf_set txs = {.buffers = &tx, .count = 1};

    spi_write_dt(spec, &txs);
}

uint8_t spi_read_uint8(const struct spi_dt_spec *spec, uint8_t reg)
{
    uint8_t tx_buf[2] = {reg | 0x80, 0};
    uint8_t rx_buf[2] = {};

    const struct spi_buf tx = {.buf = tx_buf, .len = 2};
    const struct spi_buf rx = {.buf = rx_buf, .len = 2};
    const struct spi_buf_set txs = {.buffers = &tx, .count = 1};
    const struct spi_buf_set rxs = {.buffers = &rx, .count = 1};

    spi_transceive_dt(spec, &txs, &rxs);

    return rx_buf[1];
}
