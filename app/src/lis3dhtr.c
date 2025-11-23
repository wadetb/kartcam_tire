#include "common.h"

#include "lis3dhtr.h"

LOG_MODULE_REGISTER(lis3dhtr);

#define LIS3DHTR_NODE DT_NODELABEL(lis3dhtr)

#define LIS3DHTR_OP SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE

const struct spi_dt_spec lis3dhtr = SPI_DT_SPEC_GET(LIS3DHTR_NODE, LIS3DHTR_OP);
const struct gpio_dt_spec lis3dhtr_int = GPIO_DT_SPEC_GET(LIS3DHTR_NODE, int_gpios);

struct gpio_callback lis3dhtr_gpio_cb;

#define SPI_BUF_SIZE (1 + 32 * 3 * sizeof(int16_t))

uint8_t tx_buf[SPI_BUF_SIZE] = {};
uint8_t rx_buf[SPI_BUF_SIZE] = {};

#define LOG_SIZE 60 // must be at least 32

float log_accel_step = 1.0f; // mg

int log_accel_count;
int16_t log_x[LOG_SIZE];
int16_t log_y[LOG_SIZE];
int16_t log_z[LOG_SIZE];

enum lis3dh_mode mode = LIS3DH_MODE_HIGH_RES;
enum lis3dh_scale scale = LIS3DH_SCALE_2G;
enum lis3dh_rate rate = LIS3DH_RATE_100_HZ;
uint8_t watermark = 30;

static float lis3dh_mg_per_lsb_table[] = { 0.0625f, 0.125f, 0.25f, 0.75f };
static uint16_t lis3dhtr_samples_per_sec_table[] = {0, 1, 10, 25, 50, 100, 200, 400, 1600, 5000};

static uint32_t lis3dhtr_samples_per_sec()
{
    return lis3dhtr_samples_per_sec_table[rate];
}

static uint32_t lis3dhtr_fifo_fill_usec()
{
    uint32_t samples_per_sec = lis3dhtr_samples_per_sec();
    // LOG_INF("samples_per_sec: %d", samples_per_sec);
    return watermark * 1000000 / samples_per_sec;
}

static void lis3dhtr_flush_log()
{
    // LOG_INF("flushing log at %d", log_accel_count);

    float mg_scale = lis3dh_mg_per_lsb_table[scale];
    // LOG_INF("mg_scale %f", (double)mg_scale);

    uint64_t log_timestamp = timestamp();
    uint32_t samples_per_sec = lis3dhtr_samples_per_sec();
    if (start_packet("lis3dhtr.accel.x", log_timestamp, samples_per_sec, log_accel_step))
    {
        for (int i = 0; i < log_accel_count; i++)
            add_packet_sample(log_x[i] * mg_scale);
        finish_packet();
    }
    if (start_packet("lis3dhtr.accel.y", log_timestamp, samples_per_sec, log_accel_step))
    {
        for (int i = 0; i < log_accel_count; i++)
            add_packet_sample(log_y[i] * mg_scale);
        finish_packet();
    }
    if (start_packet("lis3dhtr.accel.z", log_timestamp, samples_per_sec, log_accel_step))
    {
        for (int i = 0; i < log_accel_count; i++)
            add_packet_sample(log_z[i] * mg_scale);
        finish_packet();
    }

    log_accel_count = 0;
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

    // LOG_INF("reading %d samples from fifo, log_size=%d", samples, log_accel_count);

    // Note, we skip the first sample of each FIFO read as it is consistently invalid, for unknown reasons.

    int l = log_accel_count;
    for (int i = 1; i < samples; i++, l++)
    {
        int offset = 1 + i * 6;
        log_x[l] = (int16_t)(rx_buf[offset + 1] << 8 | rx_buf[offset + 0]);
        log_y[l] = (int16_t)(rx_buf[offset + 3] << 8 | rx_buf[offset + 2]);
        log_z[l] = (int16_t)(rx_buf[offset + 5] << 8 | rx_buf[offset + 4]);
    }
    log_accel_count += samples;

    lis3dhtr_flush_log();
}

static void lis3dhtr_thread_main(void *, void *, void *)
{
    while (1)
    {
        uint32_t fifo_fill_usec = lis3dhtr_fifo_fill_usec();
        k_usleep(fifo_fill_usec);
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

    uint8_t config1 = (rate << 4) |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE |
                      (mode == LIS3DH_MODE_LOW_POWER ? LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_LOW : 0);
    uint8_t config2 = 0;
    uint8_t config3 = 0;
    uint8_t config4 = (scale << 4) |
                      (mode == LIS3DH_MODE_HIGH_RES ? LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_ENABLE : 0);
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
    if (log_accel_count == 0)
    {
        *x = 0;
        *y = 0;
        *z = 0;
    }
    else
    {
        *x = log_x[log_accel_count - 1];
        *y = log_y[log_accel_count - 1];
        *z = log_z[log_accel_count - 1];
    }
}
