#include <sub_atlantic/valve_pack.hpp>

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
  this->declare_parameter<std::string>("device_address", "/dev/ttyUSB0");
  this->declare_parameter<int>("connect_timeout", 10);

  int baud_rate = this->get_parameter("baud_rate").as_int();
  std::string device_address = this->get_parameter("device_address").as_string();
  int connect_timeout = this->get_parameter("connect_timeout").as_int();
}

void ValvePackDriver::valveCmdCallback(const sub_atlantic_interfaces::msg::ValveCmd::SharedPtr msg) {
  // Implement the functionality here
  // ...
}

bool ValvePackDriver::connectSerial(int baud_rate, std::string device_address) {
  try {
      serial.open(device_address);
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

void ValvePackDriver::replaceSubstitutionCharacter(std::vector<unsigned char>& data, 
                                                   unsigned char characterToReplace, 
                                                   unsigned char substitutionCharacter) {
  std::replace(data.begin(), data.end(), characterToReplace, substitutionCharacter);
}
