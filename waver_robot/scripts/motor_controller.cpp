#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <mutex>
#include "i2c_motor_controller.h"

/**
 * @class MotorController
 * @brief A class to control motors via ROS messages and I2C protocol.
 */
class MotorController {
public:
    /**
     * @brief Constructor for MotorController.
     * @param motor_interface Reference to the I2C motor controller interface.
     */
    MotorController(I2CMotorController& motor_interface)
        : motor_interface_(motor_interface),
        el_max_(200),
        l_val_(0),
        r_val_(0),
        nh_("~"),
        rate_(10) {
        sub_ = nh_.subscribe("/motors/motor_twist", 1000, &MotorController::dataProcessingCallback, this);
        thread_ = std::thread(&MotorController::i2cThread, this);
    }

    /**
     * @brief Destructor for MotorController.
     */
    ~MotorController() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    /**
     * @brief Callback function for processing incoming Twist messages.
     * @param twist The incoming Twist message.
     */
    void dataProcessingCallback(const geometry_msgs::Twist::ConstPtr& twist);

    /**
     * @brief Function to be run in a separate thread for I2C communication.
     */
    void i2cThread();

    I2CMotorController& motor_interface_; ///< Reference to the I2C motor controller.
    int vel_max_; ///< Maximum velocity.
    int l_val_; ///< Left motor value.
    int r_val_; ///< Right motor value.
    ros::NodeHandle nh_; ///< ROS node handle.
    ros::Subscriber sub_; ///< ROS subscriber.
    std::thread thread_; ///< Thread for I2C communication.
    std::mutex mutex_; ///< Mutex for thread synchronization.
    ros::Rate rate_; ///< Rate object for controlling loop frequency.
};
