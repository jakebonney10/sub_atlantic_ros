#ifndef VALVE_PACK_HPP
#define VALVE_PACK_HPP

#include <string>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sub_atlantic_interfaces/msg/valve_cmd.hpp"
#include "sub_atlantic_interfaces/msg/valve_pack_cmd.hpp"
#include "sub_atlantic_interfaces/msg/valve_pack_state.hpp"

/**
 * @brief Class for controlling a valve pack using ROS2 and serial communication.
 */
class ValvePackDriver : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the ValvePackDriver class.
   */
  ValvePackDriver();

private:
  boost::asio::io_service io;
  boost::asio::serial_port serial;
  rclcpp::Subscription<sub_atlantic_interfaces::msg::ValveCmd>::SharedPtr valve_pack_subscription_;

  /**
   * @brief Initializes parameters from ROS2 parameter server.
   */
  void initializeParameters();

  /**
   * @brief Callback function for ValveCmd messages.
   * 
   * @param msg Shared pointer to the received ValveCmd message.
   */
  void valveCmdCallback(const sub_atlantic_interfaces::msg::ValveCmd::SharedPtr msg);

  /**
   * @brief Connects to the serial port with the specified settings.
   * 
   * @param baud_rate Baud rate for serial communication.
   * @param device_address Address of the serial device.
   * @return True if connection is successful, false otherwise.
   */
  bool connectSerial(int baud_rate, std::string device_address);

  /**
   * @brief Calculates the CRC-16 checksum for a given data vector.
   * 
   * @param data Vector of unsigned char containing the data for CRC calculation.
   * @return The computed CRC-16 checksum.
   */
  std::uint16_t calculateCRC16(const std::vector<unsigned char>& data);

  /**
   * @brief Splits a 16-bit word into high and low bytes.
   * 
   * @param crc 16-bit word to be split.
   * @param high_byte Reference to store the high byte.
   * @param low_byte Reference to store the low byte.
   */
  void splitBytes(std::uint16_t crc, unsigned char& high_byte, unsigned char& low_byte);

  /**
   * @brief Changes the state of a specific bit in a byte.
   * 
   * @param byte Reference to the byte whose bit is to be modified.
   * @param bit Position of the bit to be changed.
   * @param value Boolean value to set the specified bit to.
   */
  void changeBit(uint8_t& byte, uint8_t bit, bool value);

  /**
   * @brief Replaces a specific character in a data vector with a substitution character.
   * 
   * @param data Reference to a vector of unsigned char.
   * @param characterToReplace Character in the data vector to be replaced.
   * @param substitutionCharacter Character to replace the specified character.
   */
  void replaceSubstitutionCharacter(std::vector<unsigned char>& data, 
                                    unsigned char characterToReplace, 
                                    unsigned char substitutionCharacter);

  /**
   * @brief Constructs a standard control message for the valve pack board.
   * 
   * This function builds a message according to the specified communication protocol.
   * The message includes various control flags and data points, such as PWM settings and current set points.
   * 
   * @param pwm1_8 Control byte for PWM1-8 (defaults to 0x00, all off).
   * @param pwm9_16 Control byte for PWM9-16 (defaults to 0x00, all off).
   * @param pwm_open_loop Control byte for PWM1-16 Open Loop On/Off (defaults to 0x00).
   * @return std::vector<uint8_t> The constructed message as a vector of bytes.
   */
  std::vector<uint8_t> standardControlMsg(uint8_t pwm1_8 = 0x00, uint8_t pwm9_16 = 0x00, uint8_t pwm_open_loop = 0x00);
  
};

#endif // VALVE_PACK_HPP

