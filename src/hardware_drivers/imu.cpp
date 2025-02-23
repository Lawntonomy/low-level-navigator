#include "imu.h"
#include "hardware/i2c.h"

IMU::IMU() : yaw_rate(0.0)
{
}

void IMU::init()
{
    i2c_init(i2c_default, 100 * 1000);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

float IMU::getYawRate()
{
    // Read gyro Z-axis data and convert to rad/s
    return yaw_rate;
}
