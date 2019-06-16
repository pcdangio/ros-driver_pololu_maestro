#include "ros_node.h"

#include <driver_pololu_maestro/servo_position.h>

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
    std::vector<int> param_channels;
    private_node.param<std::vector<int>>("channels", param_channels, {0, 1, 2, 3, 4, 5});
    bool param_publish_positions;
    private_node.param<bool>("publish_positions", param_publish_positions, false);

    // Iterate through given channels.
    for(unsigned int i = 0; i < param_channels.size(); i++)
    {
        // Add channel to internal storage.
        ros_node::m_channels.push_back(static_cast<unsigned char>(param_channels.at(i)));

        // Set up subscriber for this channel.
        std::stringstream subscriber_topic;
        subscriber_topic << ros::this_node::getName() << "/set_target/channel_" << param_channels.at(i);
        ros_node::m_subscribers_target.push_back(ros_node::m_node->subscribe<driver_pololu_maestro::servo_target>(subscriber_topic.str(), 1, std::bind(&ros_node::target_callback, this, std::placeholders::_1, param_channels.at(i))));

        // Add position publisher.
        if(param_publish_positions == true)
        {
            std::stringstream publisher_topic;
            publisher_topic << ros::this_node::getName() << "/position/channel_" << param_channels.at(i);
            ros_node::m_publishers_position.push_back(ros_node::m_node->advertise<driver_pololu_maestro::servo_position>(publisher_topic.str(), 1));
        }
    }

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
        // Publish positions if configured to do so.
        if(ros_node::m_publishers_position.size() > 0)
        {
            for(unsigned int i = 0; i < ros_node::m_channels.size(); i++)
            {
                unsigned char& channel = ros_node::m_channels.at(i);

                try
                {
                    // Read the position from the Maestro.
                    unsigned short position_raw = ros_node::m_driver->get_position(channel);
                    // Convert the raw position to a percentage.
                    float position = ((static_cast<float>(position_raw) / 4.0f) - 1500.0f)/500.0f;
                    // Coerce to the allowable range.
                    position = std::max(std::min(position, 1.0f), -1.0f);

                    // Create the position message.
                    driver_pololu_maestro::servo_position message;
                    message.position = position;

                    // Publish the message.
                    ros_node::m_publishers_position.at(i).publish(message);
                }
                catch(...)
                {
                    ROS_WARN_STREAM("Could not read position for channel " << channel << ".");
                }
            }
        }

        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_rate->sleep();
    }
}

void ros_node::target_callback(const driver_pololu_maestro::servo_targetConstPtr &message, unsigned char channel)
{
    // Set the speed and acceleration first.
    if(std::isnan(message->acceleration) == false)
    {
        ros_node::m_driver->set_acceleration(channel, static_cast<unsigned short>(message->acceleration * 255));
    }
    if(std::isnan(message->speed) == false)
    {
        ros_node::m_driver->set_speed(channel, static_cast<unsigned short>(message->speed * 16383));
    }

    // Set the target.
    // Clip the target to -1.0 to 1.0.
    float target_f = std::max(std::min(message->position, 1.0f), -1.0f);
    // Convert percentage into microsecond quarters.
    unsigned short target = static_cast<unsigned short>((target_f * 500.0f + 1500.0f) * 4.0f);
    ros_node::m_driver->set_target(channel, target);
}
