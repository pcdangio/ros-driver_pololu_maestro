#include "driver.h"

#include <endian.h>
#include <stdexcept>

driver::driver(std::string port, uint32_t baud_rate, uint8_t device_number, bool crc_enabled)
{
    // Open the serial port.
    driver::m_serial_port = new serial::Serial(port, baud_rate, serial::Timeout::simpleTimeout(30));

    // Transmit a single 0xAA to initialize the automatic baud rate detection.
    uint8_t initiator = 0xAA;
    driver::m_serial_port->write(&initiator, 1);

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
void driver::set_target(uint8_t channel, uint16_t target)
{
    // Create data array.
    uint8_t data[3];
    // Set channel.
    data[0] = channel;
    // Set target.
    driver::serialize(target, &data[1]);

    // Send the message.
    driver::tx(0x04, data, 3);
}
void driver::set_target(std::vector<std::pair<uint8_t, uint16_t>> targets)
{
    // Create data array.
    uint32_t length = static_cast<uint32_t>(targets.size() * 3);
    uint8_t* data = new uint8_t[length];

    // Iterate through the channels.
    for(uint32_t i = 0; i < targets.size(); i++)
    {
        // Get reference to pair.
        std::pair<uint8_t, uint16_t>& current = targets.at(i);
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
void driver::set_speed(uint8_t channel, uint16_t speed)
{
    // Create data array.
    uint8_t data[3];
    // Set channel.
    data[0] = channel;
    // Set speed.
    driver::serialize(speed, &data[1]);

    // Send the message.
    driver::tx(0x07, data, 3);
}
void driver::set_acceleration(uint8_t channel, uint16_t acceleration)
{
    // Create data array.
    uint8_t data[3];
    // Set channel.
    data[0] = channel;
    // Set acceleration
    driver::serialize(acceleration, &data[1]);

    // Send the message.
    driver::tx(0x09, data, 3);
}
void driver::set_pwm(uint16_t on_time, uint16_t period)
{
    // Create data array.
    uint8_t data[4];
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
uint16_t driver::get_position(uint8_t channel)
{
    // Create data array.
    uint8_t data[1] = {channel};
    // Send the message.
    driver::tx(0x10, data, 1);

    // Read the response.
    uint8_t response[2];
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
    uint8_t response[1];
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
uint16_t driver::get_errors()
{
    // Send the message.
    driver::tx(0x21, nullptr, 0);

    // Read the response.
    uint8_t response[2];
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
void driver::tx(uint8_t command, uint8_t *data, uint32_t data_length)
{
    // Create a new packet to send.
    // Pololu Protocol: Header (1), Device Number (1), Command ID (1), data bytes, Checksum if enabled (1)
    uint32_t packet_length = 3 + data_length + static_cast<uint32_t>(driver::m_crc_enabled);
    uint8_t* packet = new uint8_t[packet_length];

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
bool driver::rx(uint8_t *data, uint32_t length)
{
    // Read bytes from the serial port.
    unsigned long n_read = driver::m_serial_port->read(data, length);

    // Return if amount of bytes were read before timeout.
    return n_read == length;
}
uint8_t driver::checksum(uint8_t *packet, uint32_t length)
{
    // Calculate the CRC-7 checksum (https://www.pololu.com/docs/0J44/6.7.6)
    const uint8_t polynomial = 0x91;

    uint8_t crc = 0;

    // Iterate over the bytes of the message.
    for(uint32_t i = 0; i < length; i++)
    {
        crc ^= packet[i];
        // Iterate over the bits of the current byte.
        for(uint8_t j = 0; j < 8; j++)
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
void driver::serialize(uint16_t value, uint8_t *buffer)
{
    // Convert target to little endian.
    uint16_t le_value = htole16(value);
    // Populate data field.
    buffer[0] = le_value & 0x007F; // Grabs lower 7 bytes.
    buffer[1] = (le_value & 0x3F80u) >> 7 ; // Grabs upper 7 bytes.
}
uint16_t driver::deserialize(uint8_t *buffer)
{
    // Create output variable.
    uint16_t output;
    // Deserialize the little endian 8-bit buffer.
    output = buffer[0];
    output |= (static_cast<uint16_t>(buffer[1]) << 8);
    // Convert from little endian to host.
    return le16toh(output);
}
