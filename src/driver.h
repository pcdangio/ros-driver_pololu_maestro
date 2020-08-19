/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <serial/serial.h>

#include <vector>

/// \brief A driver for the Pololu Maestro series of servo controllers.
class driver
{
public:
    // ENUMERATIONS
    /// \brief An enumeration of Maestro errors.
    enum class error_type
    {
        SERIAL_SIGNAL_ERROR = 0,               ///< Occurs when there is a baud rate mismatch.
        SERIAL_OVERRUN_ERROR = 2,              ///< The Maestro's internal UART buffer has overrun.
        SERIAL_BUFFER_FULL = 4,                ///< The Maestro's RX buffer has overrun.
        SERIAL_CRC_ERROR = 8,                  ///< CRC mismatch.
        SERIAL_PROTOCOL_ERROR = 16,             ///< Incorrectly formatted data received.
        SERIAL_TIMEOUT = 32,                    ///< Serial timeout period has elapsed.
        SCRIPT_STACK_ERROR = 64,                ///< User script stack underflow or overflow.
        SCRIPT_CALL_STACK_ERROR = 128,           ///< User script call stack underflow or overflow.
        SCRIPT_PROGRAM_COUNTER_ERROR = 256       ///< User script caused program counter to go out of bounds.
    };


    // CONSTRUCTORS
    /// \brief Creates a new driver instance.
    /// \param port The serial port to communicate with the Maestro through.
    /// \param baud_rate The baud rate to use for the serial communication.
    /// \param device_number The device number of the Maestro to communicate with.
    /// \param crc_enabled A flag indicating if the Maestro is configured for CRC mode.
    driver(std::string port, uint32_t baud_rate, uint8_t device_number, bool crc_enabled = false);
    ~driver();


    // SET METHODS
    /// \brief Sets the target position of a specified channel.
    /// \param channel The channel to set the target for.
    /// \param target The target to set, in units of pulse width quarter microseconds.
    void set_target(uint8_t channel, uint16_t target);
    /// \brief Sets targets for multiple channels at once.
    /// \param targets A vector of pairs including each channel and the associated target position,
    /// in units of pulse width quarter microseconds.
    /// \note This command is not compatible with the Micro Maestro.
    void set_target(std::vector<std::pair<uint8_t, uint16_t>> targets);
    /// \brief Limits the speed at which a servo can travel to reach a target.
    /// \param channel The channel to set the speed for.
    /// \param speed The speed to set, in units of pulse width quarter microseconds per 10 milliseconds.
    void set_speed(uint8_t channel, uint16_t speed);
    /// \brief Limits the acceleration at which a servo can travel to reach a target.
    /// \param channel The channel to set the acceleration for.
    /// \param acceleration The acceleration to set, in units of pulse width quarter microseconds per 10 milliseconds per 80 milliseconds.
    void set_acceleration(uint8_t channel, uint16_t acceleration);
    /// \brief Configures the pulse width modulation shape for all channels.
    /// \param on_time The period of the logic high in the duty cycle, in units of 1/48 microseconds.
    /// \param period The overall period of the duty cycle, in units of 1/48 microseconds.
    /// \note This command is not compatible with the Micro Maestro.
    void set_pwm(uint16_t on_time, uint16_t period);
    /// \brief Sends all channels to their home positions.
    void go_home();


    // GET METHODS
    /// \brief Reads the current pulse width of a channel.
    /// \param channel The channel to read the pulse width for.
    /// \return The current pulse width of the channel, in units of pulse width quarter microseconds.
    uint16_t get_position(uint8_t channel);
    /// \brief Reads if any channels are currently in motion.
    /// \return TRUE if one or more channels is still in motion, otherwise FALSE.
    /// \details This only functions if the channels are being limited by a speed or acceleration.
    /// \note This command is not compatible with the Micro Maestro.
    bool get_moving_state();
    /// \brief Reads and clears the error register from the Maestro.
    /// \return The list of all errors as a bit-encoded field .
    uint16_t get_errors();

private:
    // VARIABLES
    /// \brief The serial port for communicating with the Maestro.
    serial::Serial* m_serial_port;
    /// \brief The device number of the Maestro to communicate with.
    uint8_t m_device_number;
    /// \brief Indicates if the Maestro is operating in CRC checking mode.
    bool m_crc_enabled;

    // METHODS
    /// \brief Transmits a command over serial using the Pololu protocol.
    /// \param command The command ID.
    /// \param data The serialized data to send.
    /// \param data_length The length of the serialized data to send.
    void tx(uint8_t command, uint8_t* data, uint32_t data_length);
    /// \brief Receives data back from the Maestro.
    /// \param data The buffer to store the data in.
    /// \param length The length of data to read.
    /// \return TRUE if the requested length was read, otherwise FALSE.
    bool rx(uint8_t* data, uint32_t length);
    /// \brief Calculates the CRC-7 checksum of a packet.
    /// \param packet The message packet to calculate the CRC for.
    /// \param length The length of the message packet, in bytes.
    /// \return The CRC-7 checksum of the packet.
    uint8_t checksum(uint8_t* packet, uint32_t length);
    /// \brief Serializes an uint16_t into a uchar buffer.
    /// \param value The value to serialize.
    /// \param buffer The buffer to insert the serialized value into.
    void serialize(uint16_t value, uint8_t* buffer);
    /// \brief Deserializes an uint16_t from a uchar buffer.
    /// \param buffer The buffer containing the uint16_t.
    /// \return The deserialized uint16_t.
    uint16_t deserialize(uint8_t* buffer);
};

#endif // DRIVER_H
