#include "ros_node.h"

#include <actuator_msgs/servo_state.h>

#include <sstream>

// CONSTRUCTORS
ros_node::ros_node(int32_t argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "maestro");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle("servo_controller");

    // Read parameters.
    ros::NodeHandle private_node("~");
    std::string param_serial_port = private_node.param<std::string>("serial_port", "/dev/ttyTHS2");
    int32_t param_baud_rate = private_node.param<int32_t>("baud_rate", 57600);
    bool param_crc_enabled = private_node.param<bool>("crc_enabled", false);
    double param_update_rate = private_node.param<double>("update_rate", 30.0);
    int32_t param_device_number = private_node.param<int32_t>("device_number", 12);
    int32_t param_pwm_period = private_node.param<int32_t>("pwm_period", 20);
    std::vector<int32_t> param_channels = private_node.param<std::vector<int32_t>>("channels", {0, 1, 2, 3, 4, 5});
    bool param_publish_states = private_node.param<bool>("publish_states", false);

    // Store pwm period.
    ros_node::m_pwm_period = static_cast<uint8_t>(param_pwm_period);

    // Iterate through given channels.
    for(uint32_t i = 0; i < param_channels.size(); i++)
    {
        // Add channel to internal storage.
        ros_node::m_channels.push_back(static_cast<uint8_t>(param_channels.at(i)));

        // Set up subscriber for this channel.
        std::stringstream subscriber_topic;
        subscriber_topic << "set_target/channel_" << param_channels.at(i);
        ros_node::m_subscribers_target.push_back(private_node.subscribe<actuator_msgs::servo_target>(subscriber_topic.str(), 1, std::bind(&ros_node::target_callback, this, std::placeholders::_1, static_cast<uint8_t>(param_channels.at(i)))));

        // Add position publisher.
        if(param_publish_states == true)
        {
            std::stringstream publisher_topic;
            publisher_topic << "state/channel_" << param_channels.at(i);
            ros_node::m_publishers_state.push_back(private_node.advertise<actuator_msgs::servo_state>(publisher_topic.str(), 1));
        }
    }

    // Initialize ros node members.
    ros_node::m_rate = new ros::Rate(param_update_rate);

    // Create a new driver.
    ros_node::m_driver = new driver(param_serial_port, static_cast<uint32_t>(param_baud_rate), static_cast<uint8_t>(param_device_number), param_crc_enabled);
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_rate;
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// SPIN
void ros_node::spin()
{
    while(ros::ok())
    {
        // Publish positions if configured to do so.
        if(ros_node::m_publishers_state.size() > 0)
        {
            for(uint32_t i = 0; i < ros_node::m_channels.size(); i++)
            {
                uint8_t& channel = ros_node::m_channels.at(i);

                try
                {
                    // Read the position from the Maestro.
                    uint16_t position_qus = ros_node::m_driver->get_position(channel);
                    // Convert the quarter microsecond position to microsecond position.
                    float position = static_cast<float>(position_qus) / 4.0f;

                    // Create the position message.
                    actuator_msgs::servo_state message;
                    message.position = position;

                    // Publish the message.
                    ros_node::m_publishers_state.at(i).publish(message);
                }
                catch(...)
                {
                    ROS_WARN_STREAM("Could not read position for channel " << channel << ".");
                }
            }
        }

        // Read out and clear any errors.
        ros_node::handle_errors();

        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_rate->sleep();
    }
}

// CALLBACKS
void ros_node::target_callback(const actuator_msgs::servo_targetConstPtr &message, uint8_t channel)
{
    // Set the speed and acceleration first.
    if(std::isnan(message->acceleration) == false)
    {
        // Convert from %/sec^2 to appropriate units.
        // Appropriate units are based on the PWM period.
        float conversion = 0.000004f * std::pow(static_cast<float>(ros_node::m_pwm_period), 2.0f);
        if(ros_node::m_pwm_period < 20)
        {
            conversion *= 8.0f;
        }
        else
        {
            conversion *= 2.0f;
        }
        // Conversion is now at %/sec^2 to the value the maestro expects.
        // Round to nearest value, and coerce to 0 to 255.
        uint16_t accel = static_cast<uint16_t>(std::max(std::min(static_cast<int32_t>(std::round(message->acceleration * conversion)), 255), 0));

        ros_node::m_driver->set_acceleration(channel, accel);
    }
    if(std::isnan(message->velocity) == false)
    {
        // Convert from %/sec to appropriate units.
        // Appropriate units are based on the PWM perid.
        float conversion = 0.004f * static_cast<float>(ros_node::m_pwm_period);
        if(ros_node::m_pwm_period >= 20)
        {
            conversion *= 0.5f;
        }
        // Conversion is now at %/sec^2 to the value the maestro expects.
        // Round to the nearest value, and coerce to 0 to 16383.
        uint16_t speed = static_cast<uint16_t>(std::max(std::min(static_cast<int32_t>(std::round(message->velocity * conversion)), 16383), 0));

        ros_node::m_driver->set_speed(channel, speed);
    }

    // Set the target.
    // Clip the target to -500.0 to 2500.0.
    float target_f = std::max(std::min(message->position, 2500.0f), -500.0f);
    // Convert microseconds into microsecond quarters.
    uint16_t target = static_cast<uint16_t>(std::round(target_f * 4.0f));
    ros_node::m_driver->set_target(channel, target);
}

// ERROR HANDLING
void ros_node::handle_errors()
{
    // First read the errors.
    uint16_t errors = 0;

    try
    {
        errors = ros_node::m_driver->get_errors();
    }
    catch(std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }

    // Check each bit in the field.

    if(errors & static_cast<uint16_t>(driver::error_type::SERIAL_SIGNAL_ERROR))
    {
        ROS_ERROR_STREAM("SERIAL_SIGNAL_ERROR: Possible baudrate mismatch.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SERIAL_OVERRUN_ERROR))
    {
        ROS_ERROR_STREAM("SERIAL_OVERRUN_ERROR: UART internal buffer full.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SERIAL_BUFFER_FULL))
    {
        ROS_ERROR_STREAM("SERIAL_BUFFER_FULL: RX buffer full and bytes are being lost.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SERIAL_CRC_ERROR))
    {
        ROS_ERROR_STREAM("SERIAL_CRC_RROR: CRC mismatch detected on last received packet.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SERIAL_PROTOCOL_ERROR))
    {
        ROS_ERROR_STREAM("SERIAL_PROTOCOL_ERROR: Invalid packet format received.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SERIAL_TIMEOUT))
    {
        ROS_ERROR_STREAM("SERIAL_TIMEOUT: Timeout occured since last packet was received.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SCRIPT_CALL_STACK_ERROR))
    {
        ROS_ERROR_STREAM("SCRIPT_CALL_STACK_ERROR: Script caused over/underflow in stack.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SCRIPT_CALL_STACK_ERROR))
    {
        ROS_ERROR_STREAM("SCRIPT_CALL_STACK_ERROR: Script caused over/underflow in call stack.");
    }

    if(errors & static_cast<uint16_t>(driver::error_type::SCRIPT_PROGRAM_COUNTER_ERROR))
    {
        ROS_ERROR_STREAM("SCRIPT_PROGRAM_COUNTER_ERROR: Script caused program counter has gone out of bounds.");
    }
}
