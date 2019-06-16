#include "driver.h"

#include <endian.h>
#include <stdexcept>

driver::driver(std::string port, unsigned int baud_rate, unsigned char device_number, bool crc_enabled)
{
    // Open the serial port.
    driver::m_serial_port = new serial::Serial(port, baud_rate, serial::Timeout::simpleTimeout(30));

    // Store the device number.
    driver::m_device_number = device_number;

    // Store the CRC mode.
    driver::m_crc_enabled = crc_enabled;
}
driver::~driver()
{
    // Close the serial port.
    driver::m_serial_port->close();

    // Clean up resources.
    delete driver::m_serial_port;
}


// SET METHODS
void driver::set_target(unsigned char channel, unsigned short target)
{
    // Create data array.
    unsigned char data[3];
    // Set channel.
    data[0] = channel;
    // Set target.
    driver::serialize(target, &data[1]);

    // Send the message.
    driver::tx(0x04, data, 3);
}
void driver::set_target(std::vector<std::pair<unsigned char, unsigned short>> targets)
{
    // Create data array.
    unsigned int length = static_cast<unsigned int>(targets.size() * 3);
    unsigned char* data = new unsigned char[length];

    // Iterate through the channels.
    for(unsigned int i = 0; i < targets.size(); i++)
    {
        // Get reference to pair.
        std::pair<unsigned char, unsigned short>& current = targets.at(i);
        // Set channel.
        data[i*3] = current.first;
        // Set target.
        driver::serialize(current.second, &data[i*3 + 1]);
    }

    // Send the message.
    driver::tx(0x1F, data, length);

    // Delete the data.
    delete [] data;
}
void driver::set_speed(unsigned char channel, unsigned short speed)
{
    // Create data array.
    unsigned char data[3];
    // Set channel.
    data[0] = channel;
    // Set speed.
    driver::serialize(speed, &data[1]);

    // Send the message.
    driver::tx(0x07, data, 3);
}
void driver::set_acceleration(unsigned char channel, unsigned short acceleration)
{
    // Create data array.
    unsigned char data[3];
    // Set channel.
    data[0] = channel;
    // Set acceleration
    driver::serialize(acceleration, &data[1]);

    // Send the message.
    driver::tx(0x09, data, 3);
}
void driver::set_pwm(unsigned short on_time, unsigned short period)
{
    // Create data array.
    unsigned char data[4];
    // Set on_time.
    driver::serialize(on_time, &data[0]);
    // Set period.
    driver::serialize(period, &data[2]);

    // Send the message.
    driver::tx(0x0A, data, 4);
}
void driver::go_home()
{
    driver::tx(0x22, nullptr, 0);
}


// GET METHODS
unsigned short driver::get_position(unsigned char channel)
{
    // Create data array.
    unsigned char data[1] = {channel};
    // Send the message.
    driver::tx(0x10, data, 1);

    // Read the response.
    unsigned char response[2];
    if(driver::rx(response, 2))
    {
        // Deserialize response.
        return driver::deserialize(response);
    }
    else
    {
        throw std::runtime_error("get_position: read timeout.");
    }
}
bool driver::get_moving_state()
{
    // Send the message.
    driver::tx(0x13, nullptr, 0);

    // Read the response.
    unsigned char response[1];
    if(driver::rx(response, 1))
    {
        // Deserialize response.
        return static_cast<bool>(response[0]);
    }
    else
    {
        throw std::runtime_error("get_moving_state: read timeout.");
    }
}
unsigned short driver::get_errors()
{
    // Send the message.
    driver::tx(0x21, nullptr, 0);

    // Read the response.
    unsigned char response[2];
    if(driver::rx(response, 2))
    {
        // Deserialize response.
        return driver::deserialize(response);
    }
    else
    {
        throw std::runtime_error("get_errors: read timeout.");
    }
}

// TRANSMISSION METHODS
void driver::tx(unsigned char command, unsigned char *data, unsigned int data_length)
{
    // Create a new packet to send.
    // Pololu Protocol: Header (1), Device Number (1), Command ID (1), data bytes, Checksum if enabled (1)
    unsigned int packet_length = 3 + data_length + static_cast<unsigned int>(driver::m_crc_enabled);
    unsigned char* packet = new unsigned char[packet_length];

    // Set packet fields.
    packet[0] = 0xAA;
    packet[1] = driver::m_device_number;
    packet[2] = command;
    if(data) // Data can be empty.
    {
        std::memcpy(&packet[3], data, data_length);
    }

    // Add checksum if needed.
    if(driver::m_crc_enabled)
    {
        packet[packet_length-1] = driver::checksum(packet, packet_length - 1);
    }

    // Transmit the bytes via serial.
    driver::m_serial_port->write(packet, packet_length);

    // Delete the packet.
    delete [] packet;
}
bool driver::rx(unsigned char *data, unsigned int length)
{
    // Read bytes from the serial port.
    unsigned long n_read = driver::m_serial_port->read(data, length);

    // Return if amount of bytes were read before timeout.
    return n_read == length;
}
unsigned char driver::checksum(unsigned char *packet, unsigned int length)
{
    // Calculate the CRC-7 checksum (https://www.pololu.com/docs/0J44/6.7.6)
    const unsigned char polynomial = 0x91;

    unsigned char crc = 0;

    // Iterate over the bytes of the message.
    for(unsigned int i = 0; i < length; i++)
    {
        crc ^= packet[i];
        // Iterate over the bits of the current byte.
        for(unsigned char j = 0; j < 8; j++)
        {
            if(crc & 1)
            {
                crc ^= polynomial;
            }
            crc >>= 1;
        }
    }

    return crc;
}
void driver::serialize(unsigned short value, unsigned char *buffer)
{
    // Convert target to little endian.
    unsigned short le_value = htole16(value);
    // Populate data field.
    buffer[0] = le_value & 0x007F; // Grabs lower 7 bytes.
    buffer[1] = (le_value & 0x3F80u) >> 7 ; // Grabs upper 7 bytes.
}
unsigned short driver::deserialize(unsigned char *buffer)
{
    // Create output variable.
    unsigned short output;
    // Deserialize the little endian 7-bit buffer.
    output = buffer[0];
    output |= (static_cast<unsigned short>(buffer[1]) << 7);
    // Convert from little endian to host.
    return le16toh(output);
}
