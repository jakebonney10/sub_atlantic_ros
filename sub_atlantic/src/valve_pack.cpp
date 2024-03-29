#include <sub_atlantic/valve_pack.hpp>

NS_HEAD

ValvePackDriver::ValvePackDriver() : Node("valve_pack_driver"), serial(io) {
  // Initialize parameters
  initializeParameters();

  // TODO: Set up serial connection

  // Subscribe to ValvePack messages
  valve_pack_subscription_ = this->create_subscription<sub_atlantic_interfaces::msg::ValveCmd>(
    "valve_topic", 10,
    std::bind(&ValvePackDriver::valveCmdCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ValvePackDriver initialized");
}

void ValvePackDriver::initializeParameters() {
  this->declare_parameter<int>("baud_rate", 38400);
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("update_rate_hz", 40);
  this->declare_parameter<int>("connect_timeout_s", 10);

  //int baud_rate = this->get_parameter("baud_rate").as_int();
  //std::string serial_port = this->get_parameter("serial_port").as_string();
  //int update_rate = this->get_parameter("update_rate_hz").as_int();
  //int connect_timeout = this->get_parameter("connect_timeout_s").as_int();
}

void ValvePackDriver::valveCmdCallback(const sub_atlantic_interfaces::msg::ValveCmd::SharedPtr msg) {
  int32_t valve_id = msg->valve_id;
  bool is_on = msg->is_on;
  bool is_open_loop_control = msg->is_open_loop_control;
  double current_set_point = msg->current_set_point;
  
  RCLCPP_INFO(
      this->get_logger(), 
      "Received Valve Command - Valve ID: %d, Is On: %s, Is Open Loop Control: %s, Current Set Point: %f", 
      valve_id, 
      is_on ? "true" : "false", 
      is_open_loop_control ? "true" : "false", 
      current_set_point
  );

  datagrams::Control control_msg;
  control_msg.setPwmPair(0,1);

  auto serialized_message = control_msg.serialize();
  
  // TODO: Implement the logic for handling the ValveCmd message
}

bool ValvePackDriver::connectSerial(int baud_rate, std::string serial_port) {
  try {
      serial.open(serial_port);
      serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
      serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial.set_option(boost::asio::serial_port_base::character_size(8));
      return true;
  } catch (const boost::system::system_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Serial Error: %s", e.what());
      return false;
  }
}

std::uint16_t ValvePackDriver::calculateCRC16(const std::vector<unsigned char>& data) {
  boost::crc_16_type crcComputer;
  crcComputer.process_bytes(data.data(), data.size() - 3);
  return crcComputer.checksum();
}

void ValvePackDriver::splitBytes(std::uint16_t crc, unsigned char& high_byte, unsigned char& low_byte) {
  high_byte = static_cast<unsigned char>((crc >> 8) & 0xFF);
  low_byte = static_cast<unsigned char>(crc & 0xFF);
}

void ValvePackDriver::changeBit(uint8_t& byte, uint8_t bit, bool value) {
  if (value) {
      byte |= (1 << bit);
  } else {
      byte &= ~(1 << bit);
  }
}

uint8_t ValvePackDriver::findSubstitutionCharacter(const std::vector<uint8_t>& frame) {
    uint8_t possibleSubChar = 0x00;
    do {
        if (possibleSubChar != 0xAA) { 
            if (std::find(frame.begin(), frame.end(), possibleSubChar) == frame.end()) { 
                return possibleSubChar; // Found a suitable substitution character
            }
        }
        possibleSubChar++;
    } while (possibleSubChar != 0x00); // When possibleSubChar wraps around to 0x00, the loop ends

    throw std::runtime_error("No suitable substitution character found");
}

void ValvePackDriver::replaceSubstitutionCharacter(std::vector<unsigned char>& data, 
                                                   unsigned char characterToReplace, 
                                                   unsigned char substitutionCharacter) {
  std::replace(data.begin(), data.end(), characterToReplace, substitutionCharacter);
}

std::vector<uint8_t> ValvePackDriver::standardControlMsg(uint8_t pwm1_8, uint8_t pwm9_16, uint8_t pwm_open_loop) {
  std::vector<uint8_t> msg; 
  msg.push_back(0x00); // Index 0: Substitution Character - TODO: placeholder for now
  msg.push_back(0x01); // Index 1: Message ID - always 0x01 for standard control message
  msg.push_back(0x00); // Index 2: Control Serials and Message Address for second board - not implemented, set to 0
  msg.push_back(0x00); // Index 3: Control Analogues - not implemented, set to 0x00
  msg.push_back(pwm1_8); // Index 4: PWM1-8 On/Off, 0x00 = all off
  msg.push_back(pwm9_16); // Index 5: PWM9-16 On/Off, 0x00 = all off
  msg.push_back(0x00); // Index 6: Switched outputs DOUT1-8 On/Off
  msg.push_back(0x00); // Index 7: Switched outputs DOUT1-8 On/Off
  msg.push_back(0x00); // Index 8: Secondary supplies SSUP1-4 On/Off
  msg.push_back(pwm_open_loop); // Index 9: PWM1-16 Open Loop On/Off, 2 valves per bit

  // Index 10-25: Current Set Points for PWM pairs - Index 10: 1-2 (upper byte), Index 11: 1-2 (lower byte), etc.
  for (int i = 0; i < 16; i++) {
      msg.push_back(0x00); // TODO: placeholder, need to get 16-bit current set-point and split into upper and lower bytes
  }

  msg.push_back(0x00); // Index 26: RS232 Channel 2 Data To Transmit, not implemented, set to 0x00
  msg.push_back(0x00); // Index 27: CAN 1 Data To Transmit, not implemented, set to 0x00
  msg.push_back(0x00); // Index 28: CAN 2 Data To Transmit, not implemented, set to 0x00 
  msg.push_back(0x00); // Index 29: CRC-16 upper byte, TODO: placeholder, calculate actual CRC
  msg.push_back(0x00); // Index 30: CRC-16 lower byte
  msg.push_back(0xAA); // End of Frame identifier
  return msg;

}

NS_FOOT