#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "lis3dhtr.h"
#include "common.h"

extern struct spi_dt_spec lis3dhtr;

LOG_MODULE_REGISTER(LIS3DHTR);

bool lis3dhtr_is_present(void)
{
    uint8_t id = spi_read_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_WHO_AM_I);
    return id == 0x33;
}

void lis3dhtr_init(void)
{
    if (!lis3dhtr_is_present()) {
        LOG_ERR("LIS3DHTR not detected!");
        return;
    }

    uint8_t tempcfg = LIS3DHTR_REG_TEMP_ADC_PD_ENABLED |
                      LIS3DHTR_REG_TEMP_TEMP_EN_DISABLED;
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_TEMP_CFG, tempcfg);

    k_msleep(LIS3DHTR_CONVERSIONDELAY);

    uint8_t config1 = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_400 |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_NORMAL |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE;
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG1, config1);
    k_msleep(LIS3DHTR_CONVERSIONDELAY);

    uint8_t config4 = LIS3DHTR_REG_ACCEL_CTRL_REG4_BDU_NOTUPDATED |
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_BLE_LSB |
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_2G |
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_ENABLE |
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_ST_NORMAL |
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_SIM_4WIRE;
    spi_write_uint8(&lis3dhtr, LIS3DHTR_REG_ACCEL_CTRL_REG4, config4);
    k_msleep(LIS3DHTR_CONVERSIONDELAY);
}

int16_t lis3dhtr_get_temperature(void)
{
    int16_t result = spi_read_int16(&lis3dhtr, LIS3DHTR_REG_ACCEL_OUT_ADC3_L);
    return 25 + result / 256;
}

void lis3dhtr_get_acceleration(int16_t *x, int16_t *y, int16_t *z)
{
    int16_t buf[3];
    spi_read_buf(&lis3dhtr, LIS3DHTR_REG_ACCEL_OUT_X_L, 6, (uint8_t *)buf);

    *x = buf[0];
    *y = buf[1];
    *z = buf[2];
}

// uint8_t lis3dhtr_get_int_status(void)
// {
//     return lis3dhtr_read(LIS3DHTR_REG_ACCEL_INT1_SRC);
// }

// void lis3dhtr_set_interrupt(void)
// {
//     uint8_t config1 = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_50 |
//                       LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_NORMAL | // Normal Mode
//                       LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE | // Acceleration Z-Axis Enabled
//                       LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE | // Acceleration Y-Axis Enabled
//                       LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE;
//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_CTRL_REG1, config1);  // (50 Hz),  X/Y/Z-axis enable
//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_CTRL_REG2, 0x00);  // 

//     uint8_t config3 = LIS3DHTR_CTRL_REG3_IA1_ENABLE; 
//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_CTRL_REG3, config3);  // IA1 interrupt

//     lis3dhtr_set_full_scale_range(LIS3DHTR_RANGE_8G);

//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_CTRL_REG5, 0x00);  // Latch interrupt request 
//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_CTRL_REG6, 0x42);  // IA1, active-low  Enable interrupt 1 function on INT2 pin
//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_INT1_THS, 0x50);    //set Threshold，2g =>16mg/LSB，4g => 32mg/LSB，8g => 62mg/LSB，16g => 186mg/LSB

//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_INT1_DURATION, 0); 

//     (void)lis3dhtr_read(LIS3DHTR_REG_ACCEL_INT1_SRC);     //clear interrupt flag

//     lis3dhtr_write(LIS3DHTR_REG_ACCEL_INT1_CFG, 0x2a);        //trigger when ZHIE/YHIE/XHIE
// }
