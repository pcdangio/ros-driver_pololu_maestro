/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>
#include <driver_pololu_maestro/servo_target.h>

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
    // ENUMERATIONS
    enum class maestro_type
    {
        MICRO_MAESTRO_6 = 6,
        MINI_MAESTRO_12 = 12,
        MINI_MAESTRO_18 = 18,
        MINI_MAESTRO_24 = 24,
    };

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

    // PUBLISHERS
    std::vector<ros::Publisher> m_publishers_position;

    // SUBSCRIBERS
    std::vector<ros::Subscriber> m_subscribers_target;

    // CALLBACKS
    void target_callback(const driver_pololu_maestro::servo_targetConstPtr& message, unsigned char channel);
};

#endif // ROS_NODE_H

