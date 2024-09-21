#include "i2c_motor_controller.h"

/**
 * @brief Constructor for I2CMotorController.
 * @param i2c_address The I2C address of the motor controller.
 * @param i2c_device The I2C device file.
 */
I2CMotorController::I2CMotorController(uint8_t i2c_address, const std::string& i2c_device)
    : address_(i2c_address) {
    // Open the I2C device
    if ((i2c_file_ = open(i2c_device.c_str(), O_RDWR)) < 0) {
        ROS_ERROR("Failed to open the I2C bus");
        exit(1);
    }
    // Specify the address of the I2C slave to communicate with
    if (ioctl(i2c_file_, I2C_SLAVE, address_) < 0) {
        ROS_ERROR("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }
}

/**
 * @brief Destructor for I2CMotorController.
 */
I2CMotorController::~I2CMotorController() {
    close(i2c_file_);
}

/**
 * @brief Converts an integer to a byte array.
 * @param value The integer value to convert.
 * @return A vector of bytes representing the integer.
 */
std::vector<uint8_t> I2CMotorController::intToBytesArray(int16_t value) {
    std::vector<uint8_t> result(2);
    result[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    result[1] = static_cast<uint8_t>(value & 0xFF);
    return result;
}

/**
 * @brief Sends a vector of byte values to the I2C device.
 * @param values The vector of byte values to send.
 */
void I2CMotorController::sendValues(const std::vector<uint8_t>& values) {
    // Write data to the I2C device
    if (write(i2c_file_, values.data(), values.size()) != static_cast<ssize_t>(values.size())) {
        ROS_ERROR("Failed to write to the I2C bus");
    }
}

/**
 * @brief Sends speed values to the motors.
 * @param speed_l Speed for the left motor.
 * @param speed_r Speed for the right motor.
 * @param log Whether to log the operation.
 */
void I2CMotorController::sendSpeed(int16_t speed_l, int16_t speed_r, bool log) {
    std::vector<uint8_t> var_l = intToBytesArray(speed_l);
    std::vector<uint8_t> var_r = intToBytesArray(speed_r);

    // Combine left and right speed bytes
    std::vector<uint8_t> values = {var_l[0], var_l[1], var_r[0], var_r[1]};

    sendValues(values);

    if (log) {
        ROS_INFO("Speed Left: %d, Speed Right: %d", speed_l, speed_r);
    }
}
