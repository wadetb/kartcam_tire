#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "lis3dhtr.h"
#include "common.h"

LOG_MODULE_REGISTER(lis3dhtr);

#define LIS3DHTR_NODE DT_NODELABEL(lis3dhtr)

#define LIS3DHTR_OP SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

const struct spi_dt_spec lis3dhtr = SPI_DT_SPEC_GET(LIS3DHTR_NODE, LIS3DHTR_OP);
const struct gpio_dt_spec lis3dhtr_int = GPIO_DT_SPEC_GET(LIS3DHTR_NODE, int_gpios);

struct gpio_callback lis3dhtr_gpio_cb;

#define SPI_BUF_SIZE (1 + 32 * 3 * sizeof(int16_t))

uint8_t tx_buf[SPI_BUF_SIZE] = {};
uint8_t rx_buf[SPI_BUF_SIZE] = {};

// should be 12 bit data, left justified, but what I'm seeing looks like 10 bit data, so I'm quantizing more aggressively for now
#define LOG_QUANITIZE 6
#define LOG_SIZE 100

int log_count;
int16_t log_x[LOG_SIZE];
int16_t log_y[LOG_SIZE];
int16_t log_z[LOG_SIZE];

int rate = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_100;
int watermark = 30;

// static void log_fifo(const char *label, int16_t *data, uint8_t len)
// {
//     char buf[256];
//     int offset = 0;
//     offset += snprintf(buf + offset, sizeof(buf) - offset, "%s:", label);
//     for (int i = 0; i < len; i++)
//     {
//         offset += snprintf(buf + offset, sizeof(buf) - offset, " %d", data[i]);
//     }
//     LOG_INF("%s", buf);
// }

// https://github.com/rygorous/gaffer_net/blob/master/main.cpp
// https://go-compression.github.io/algorithms/arithmetic/
// https://github.com/WangXuan95/TinyZZZ/blob/main/src/lz4C.c

uint8_t packet[3 * LOG_SIZE];
char str[2 * 3 * LOG_SIZE + 1];

int32_t last_s = 0;
uint32_t packet_len = 0;

static uint32_t lis3dhtr_samples_per_sec()
{
    switch (rate)
    {
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_1:
        return 1;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_10:
        return 10;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_25:
        return 25;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_50:
        return 50;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_100:
        return 100;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_200:
        return 200;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_400:
        return 400;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_1_6K:
        return 1600;
    case LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_5K:
        return 5000;
    default:
        return 0;
    }
}

static uint32_t lis3dhtr_fifo_fill_usec()
{
    uint32_t samples_per_sec = lis3dhtr_samples_per_sec();
    return watermark * 1000000 / samples_per_sec;
}

static void lis3dhtr_add_sample(int16_t s)
{
    s = s >> LOG_QUANITIZE;

    int32_t d = s - last_s;

    if (d >= 127 || d < -128)
    {
        uint32_t us = (uint32_t)(s + 32768); // make unsigned for transmission
        packet[packet_len++] = 0xff;
        packet[packet_len++] = (us >> 8) & 0xff;
        packet[packet_len++] = us & 0xff;
    }
    else
    {
        packet[packet_len++] = (uint8_t)(d + 128);
    }

    last_s = s;
}

static void lis3dhtr_flush_log()
{
    // LOG_INF("flushing log at %d", log_count);

    packet_len = 0;
    last_s = 0;

    for (int i = 0; i < log_count; i++)
        lis3dhtr_add_sample(log_x[i]);
    for (int i = 0; i < log_count; i++)
        lis3dhtr_add_sample(log_y[i]);
    for (int i = 0; i < log_count; i++)
        lis3dhtr_add_sample(log_z[i]);

    for (int i = 0; i < packet_len; i++)
    {
        snprintf(&str[i * 2], 3, "%02X", packet[i]);
    }
    LOG_INF("accel packet: %s", str);

    log_count = 0;
}

static void lis3dhtr_read_fifo(int samples)
{
    // LOG_INF("lis3dhtr_read_fifo: samples=%d", samples);

    // Read mode, auto-increment - addr automatically wraps around after each sample
    tx_buf[0] = LIS3DHTR_REG_ACCEL_OUT_X_L | 0x80 | 0x40;

    uint8_t len = 1 + samples * 3 * sizeof(int16_t);

    struct spi_buf tx0 = {.buf = tx_buf, .len = len};
    struct spi_buf rx0 = {.buf = rx_buf, .len = len};
    struct spi_buf_set txs = {.buffers = &tx0, .count = 1};
    struct spi_buf_set rxs = {.buffers = &rx0, .count = 1};

    spi_transceive_dt(&lis3dhtr, &txs, &rxs);

    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_FIFO_CTRL, LIS3DHTR_REG_ACCEL_FIFO_CTRL_FIFO_MODE_BYPASS);
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_FIFO_CTRL, LIS3DHTR_REG_ACCEL_FIFO_CTRL_FIFO_MODE_FIFO | watermark);

    if (log_count + samples > LOG_SIZE)
    {
        lis3dhtr_flush_log();
    }

    int l = log_count;
    for (int i = 0; i < samples; i++, l++)
    {
        int offset = 1 + i * 6;
        log_x[l] = rx_buf[offset + 0] | (rx_buf[offset + 1] << 8);
        log_y[l] = rx_buf[offset + 2] | (rx_buf[offset + 3] << 8);
        log_z[l] = rx_buf[offset + 4] | (rx_buf[offset + 5] << 8);
    }
    log_count += samples;
}

static void lis3dhtr_thread_main(void *, void *, void *)
{
    while (1)
    {
        uint32_t fifo_fill_usec = lis3dhtr_fifo_fill_usec();
        k_usleep(fifo_fill_usec);
        // LOG_INF("fifo_samples_per_sec: %d", lis3dhtr_samples_per_sec());
        // LOG_INF("fifo_fill_usec: %d", fifo_fill_usec);

        uint8_t fifo_src = spi_read_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_FIFO_SRC);
        uint8_t ovrn_fifo = fifo_src & 0x40;
        uint8_t fss = fifo_src & 0x1F;
        // uint8_t wtm = fifo_src & 0x80;
        // uint8_t empty = fifo_src & 0x20;
        // LOG_INF("FIFO_SRC: 0x%02x wtm=%x ovrn_fifo=%x empty=%x, fss=%x", fifo_src, wtm, ovrn_fifo, empty, fss);

        if (ovrn_fifo)
        {
            LOG_WRN("fifo overrun");
        }

        if (fss > 0)
        {
            lis3dhtr_read_fifo(fss);
        }
    }
}

K_THREAD_STACK_DEFINE(lis3dhtr_thread_stack, 1024);
struct k_thread lis3dhtr_thread;

static void lis3dhtr_int_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    uint8_t src = spi_read_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_INT1_SRC);
    (void)src;

    // LOG_INF("INT triggered by %x", src);
}

void lis3dhtr_init(void)
{
    uint8_t id = spi_read_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_WHO_AM_I);
    if (id != 0x33)
    {
        LOG_ERR("device not detected!");
        return;
    }

    spi_read_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_INT1_SRC); // Clear any pending interrupts

    gpio_pin_configure_dt(&lis3dhtr_int, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&lis3dhtr_int, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&lis3dhtr_gpio_cb, lis3dhtr_int_handler, BIT(lis3dhtr_int.pin));
    gpio_add_callback(lis3dhtr_int.port, &lis3dhtr_gpio_cb);

    uint8_t config1 = rate |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE;
    uint8_t config2 = 0;
    uint8_t config3 = 0;
    uint8_t config4 = LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_2G |
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_ENABLE;
    uint8_t config5 = LIS3DHTR_REG_ACCEL_CTRL_REG5_FIFO_ENABLE;
    uint8_t fifoctrl = LIS3DHTR_REG_ACCEL_FIFO_CTRL_FIFO_MODE_FIFO | watermark;

    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG1, config1);
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG2, config2);
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG3, config3);
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG4, config4);
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG5, config5);
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_FIFO_CTRL, fifoctrl);

    lis3dhtr_read_fifo(32);

    k_thread_create(&lis3dhtr_thread, lis3dhtr_thread_stack,
                                 K_THREAD_STACK_SIZEOF(lis3dhtr_thread_stack),
                                 lis3dhtr_thread_main,
                                 NULL, NULL, NULL,
                                 7, 0, K_NO_WAIT);

    LOG_INF("initialized");
}

void lis3dhtr_latest(int16_t *x, int16_t *y, int16_t *z)
{
    // LOG_INF("log_count: %d", log_count);
    // int n = log_count < 10 ? log_count : 10;
    // log_fifo("log_x", log_x + log_count - n, n);
    // log_fifo("log_y", log_y + log_count - n, n);
    // log_fifo("log_z", log_z + log_count - n, n);

    if (log_count == 0)
    {
        *x = 0;
        *y = 0;
        *z = 0;
    }
    else
    {
        *x = log_x[log_count - 1];
        *y = log_y[log_count - 1];
        *z = log_z[log_count - 1];
    }
}
