/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>
#include <actuator_msgs/ServoTarget.h>

///
/// \brief Implements the driver's ROS node functionality.
///
class ros_node
{
public:
    // CONSTRUCTORS
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(int argc, char **argv);
    ~ros_node();

    // METHODS
    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:

    // VARIABLES
    ///
    /// \brief m_driver The driver instance.
    ///
    driver* m_driver;
    ///
    /// \brief m_node The node's handle.
    ///
    ros::NodeHandle* m_node;
    ///
    /// \brief m_rate The rate at which to run the node.
    ///
    ros::Rate* m_rate;
    ///
    /// \brief m_channels Stores the operational channels.
    ///
    std::vector<unsigned char> m_channels;
    ///
    /// \brief m_pwm_period Stores the PWM period of the Maestro.
    ///
    unsigned short m_pwm_period;

    // PUBLISHERS
    ///
    /// \brief m_publishers_position A vector of servo_position publishers.
    ///
    std::vector<ros::Publisher> m_publishers_position;

    // SUBSCRIBERS
    ///
    /// \brief m_subscribers_target A vector of servo_target subscribers.
    ///
    std::vector<ros::Subscriber> m_subscribers_target;

    // CALLBACKS
    ///
    /// \brief target_callback The callback for servo_target messages.
    /// \param message The received servo_target message.
    /// \param channel The channel associated with the subscriber/message.
    ///
    void target_callback(const actuator_msgs::ServoTargetConstPtr& message, unsigned char channel);
};

#endif // ROS_NODE_H

