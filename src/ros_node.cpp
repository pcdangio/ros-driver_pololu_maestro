#include "ros_node.h"

#include <sstream>

ros_node::ros_node(int argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "maestro");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle();

    // Read parameters.
    ros::NodeHandle private_node("~");
    std::string param_serial_port;
    private_node.param<std::string>("serial_port", param_serial_port, "/dev/ttyTHS2");
    int param_baud_rate;
    private_node.param<int>("baud_rate", param_baud_rate, 57600);
    bool param_crc_enabled;
    private_node.param<bool>("crc_enabled", param_crc_enabled, false);
    double param_update_rate;
    private_node.param<double>("update_rate", param_update_rate, 30.0);
    int param_device_number;
    private_node.param<int>("device_number", param_device_number, 12);
    int param_device_type;
    private_node.param<int>("device_type", param_device_type, 6);
    std::vector<int> param_channels;
    private_node.param<std::vector<int>>("channels", param_channels, {0, 1, 2, 3, 4, 5});

    // Set up subscribers.
    for(unsigned int i = 0; i < param_channels.size(); i++)
    {
        std::stringstream topic_name;
        topic_name << ros::this_node::getName() << "/set_target_" << param_channels.at(i);
        ros_node::m_subscribers_target.push_back(ros_node::m_node->subscribe<driver_pololu_maestro::servo_target>(topic_name.str(), 1, std::bind(&ros_node::target_callback, this, std::placeholders::_1, param_channels.at(i))));
    }

    // Set up publishers.

    // Initialize ros node members.
    ros_node::m_rate = new ros::Rate(param_update_rate);

    // Create a new driver.
    ros_node::m_driver = new driver(param_serial_port, static_cast<unsigned int>(param_baud_rate), static_cast<unsigned char>(param_device_number), param_crc_enabled);
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_rate;
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

void ros_node::spin()
{
    while(ros::ok())
    {
        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_rate->sleep();
    }
}

void ros_node::target_callback(const driver_pololu_maestro::servo_targetConstPtr &message, unsigned char channel)
{
    ROS_INFO_STREAM("Channel: " << channel << "\tPosition: " << message->position);
}
