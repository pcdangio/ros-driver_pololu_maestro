/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>

#include <vector>

///
/// \brief A driver for the Pololu Maestro series of servo controllers.
///
class driver
{
public:
    // ENUMERATIONS
    ///
    /// \brief An enumeration of Maestro errors.
    ///
    enum class error_type
    {
        SERIAL_SIGNAL_ERROR = 0x0000,               ///< Occurs when there is a baud rate mismatch.
        SERIAL_OVERRUN_ERROR = 0x0002,              ///< The Maestro's internal UART buffer has overrun.
        SERIAL_BUFFER_FULL = 0x0004,                ///< The Maestro's RX buffer has overrun.
        SERIAL_CRC_ERROR = 0x0008,                  ///< CRC mismatch.
        SERIAL_PROTOCOL_ERROR = 0x0016,             ///< Incorrectly formatted data received.
        SERIAL_TIMEOUT = 0x0032,                    ///< Serial timeout period has elapsed.
        SCRIPT_STACK_ERROR = 0x0064,                ///< User script stack underflow or overflow.
        SCRIPT_CALL_STACK_ERROR = 0x0128,           ///< User script call stack underflow or overflow.
        SCRIPT_PROGRAM_COUNTER_ERROR = 0x0256       ///< User script caused program counter to go out of bounds.
    };


    // CONSTRUCTORS
    ///
    /// \brief driver Creates a new driver instance.
    /// \param port The serial port to communicate with the Maestro through.
    /// \param baud_rate The baud rate to use for the serial communication.
    /// \param device_number The device number of the Maestro to communicate with.
    /// \param crc_enabled A flag indicating if the Maestro is configured for CRC mode.
    ///
    driver(std::string port, unsigned int baud_rate, unsigned char device_number, bool crc_enabled = false);
    ~driver();


    // SET METHODS
    ///
    /// \brief set_target Sets the target position of a specified channel.
    /// \param channel The channel to set the target for.
    /// \param target The target to set, in units of pulse width quarter microseconds.
    ///
    void set_target(unsigned char channel, unsigned short target);
    ///
    /// \brief set_target Sets targets for multiple channels at once.
    /// \param targets A vector of pairs including each channel and the associated target position,
    /// in units of pulse width quarter microseconds.
    /// \note This command is not compatible with the Micro Maestro.
    ///
    void set_target(std::vector<std::pair<unsigned char, unsigned short>> targets);
    ///
    /// \brief set_speed Limits the speed at which a servo can travel to reach a target.
    /// \param channel The channel to set the speed for.
    /// \param speed The speed to set, in units of pulse width quarter microseconds per 10 milliseconds.
    ///
    void set_speed(unsigned char channel, unsigned short speed);
    ///
    /// \brief set_acceleration Limits the acceleration at which a servo can travel to reach a target.
    /// \param channel The channel to set the acceleration for.
    /// \param acceleration The acceleration to set, in units of pulse width quarter microseconds per 10 milliseconds per 80 milliseconds.
    ///
    void set_acceleration(unsigned char channel, unsigned short acceleration);
    ///
    /// \brief set_pwm Configures the pulse width modulation shape for all channels.
    /// \param on_time The period of the logic high in the duty cycle, in units of 1/48 microseconds.
    /// \param period The overall period of the duty cycle, in units of 1/48 microseconds.
    /// \note This command is not compatible with the Micro Maestro.
    ///
    void set_pwm(unsigned short on_time, unsigned short period);
    ///
    /// \brief go_home Sends all channels to their home positions.
    ///
    void go_home();


    // GET METHODS
    ///
    /// \brief get_position Reads the current pulse width of a channel.
    /// \param channel The channel to read the pulse width for.
    /// \return The current pulse width of the channel, in units of pulse width quarter microseconds.
    ///
    unsigned short get_position(unsigned char channel);
    ///
    /// \brief get_moving_state Reads if any channels are currently in motion.
    /// \return TRUE if one or more channels is still in motion, otherwise FALSE.
    /// \details This only functions if the channels are being limited by a speed or acceleration.
    /// \note This command is not compatible with the Micro Maestro.
    ///
    bool get_moving_state();
    ///
    /// \brief get_errors Reads and clears the error register from the Maestro.
    /// \return The list of all errors as a bit-encoded field .
    ///
    unsigned short get_errors();

private:
    // VARIABLES
    ///
    /// \brief m_serial_port The serial port for communicating with the Maestro.
    ///
    serial::Serial* m_serial_port;
    ///
    /// \brief m_device_number The device number of the Maestro to communicate with.
    ///
    unsigned char m_device_number;
    ///
    /// \brief m_crc_enabled Indicates if the Maestro is operating in CRC checking mode.
    ///
    bool m_crc_enabled;

    // METHODS
    ///
    /// \brief tx Transmits a command over serial using the Pololu protocol.
    /// \param command The command ID.
    /// \param data The serialized data to send.
    /// \param data_length The length of the serialized data to send.
    ///
    void tx(unsigned char command, unsigned char* data, unsigned int data_length);
    ///
    /// \brief rx Receives data back from the Maestro.
    /// \param data The buffer to store the data in.
    /// \param length The length of data to read.
    /// \return TRUE if the requested length was read, otherwise FALSE.
    ///
    bool rx(unsigned char* data, unsigned int length);
    ///
    /// \brief checksum Calculates the CRC-7 checksum of a packet.
    /// \param packet The message packet to calculate the CRC for.
    /// \param length The length of the message packet, in bytes.
    /// \return The CRC-7 checksum of the packet.
    ///
    unsigned char checksum(unsigned char* packet, unsigned int length);
    ///
    /// \brief serialize Serializes an unsigned short into a uchar buffer.
    /// \param value The value to serialize.
    /// \param buffer The buffer to insert the serialized value into.
    ///
    void serialize(unsigned short value, unsigned char* buffer);
    ///
    /// \brief deserialize Deserializes an unsigned short from a uchar buffer.
    /// \param buffer The buffer containing the unsigned short.
    /// \return The deserialized unsigned short.
    ///
    unsigned short deserialize(unsigned char* buffer);
};

#endif // DRIVER_H
