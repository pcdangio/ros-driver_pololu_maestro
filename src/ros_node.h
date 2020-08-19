/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>
#include <actuator_msgs/servo_target.h>

/// \brief Implements the driver's ROS node functionality.
class ros_node
{
public:
    // CONSTRUCTORS
    /// \brief Initializes the ROS node.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ros_node(int32_t argc, char **argv);
    ~ros_node();

    // METHODS
    /// \brief Runs the node.
    void spin();

private:

    // VARIABLES
    /// \brief The driver instance.
    driver* m_driver;
    /// \brief The node's handle.
    ros::NodeHandle* m_node;
    /// \brief The rate at which to run the node.
    ros::Rate* m_rate;
    /// \brief Stores the operational channels.
    std::vector<uint8_t> m_channels;
    /// \brief Stores the PWM period of the Maestro.
    uint16_t m_pwm_period;

    // PUBLISHERS
    /// \brief A vector of ServoState publishers.
    std::vector<ros::Publisher> m_publishers_state;

    // SUBSCRIBERS
    /// \brief A vector of ServoTarget subscribers.
    std::vector<ros::Subscriber> m_subscribers_target;

    // CALLBACKS
    /// \brief The callback for ServoTarget messages.
    /// \param message The received ServoTarget message.
    /// \param channel The channel associated with the subscriber/message.
    void target_callback(const actuator_msgs::servo_targetConstPtr& message, uint8_t channel);

    // METHODS
    /// \brief Processes and logs errors read from the Maestro
    void handle_errors();
};

#endif // ROS_NODE_H

