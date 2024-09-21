#ifndef I2C_MOTOR_CONTROLLER_H
#define I2C_MOTOR_CONTROLLER_H

#include <vector>
#include <string>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <ros/ros.h>

/**
 * @class I2CMotorController
 * @brief A class to control motors via I2C protocol.
 */
class I2CMotorController
{
public:
    /**
     * @brief Constructor for I2CMotorController.
     * @param i2c_address The I2C address of the motor controller.
     * @param i2c_device The I2C device file.
     */
    I2CMotorController(uint8_t i2c_address = 0x11, const std::string &i2c_device = "/dev/i2c-1");

    /**
     * @brief Destructor for I2CMotorController.
     */
    ~I2CMotorController();

    /**
     * @brief Sends speed values to the motors.
     * @param speed_l Speed for the left motors.
     * @param speed_r Speed for the right motors.
     * @param log Whether to log the operation.
     */
    void sendSpeed(int16_t speed_l, int16_t speed_r, bool log = false);

private:
    int i2c_file_; ///< File descriptor for the I2C device.
    uint8_t address_; ///< I2C address of the motor controller.

    /**
     * @brief Converts an integer to a byte array.
     * @param value The integer value to convert.
     * @return A vector of bytes representing the integer.
     */
    std::vector<uint8_t> intToBytesArray(int16_t value);

    /**
     * @brief Sends a vector of byte values to the I2C device.
     * @param values The vector of byte values to send.
     */
    void sendValues(const std::vector<uint8_t> &values);
};

#endif // I2C_MOTOR_CONTROLLER_H
