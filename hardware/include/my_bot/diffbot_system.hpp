#ifndef MY_BOT__DIFFBOT_SYSTEM_HPP_
#define MY_BOT__DIFFBOT_SYSTEM_HPP_

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "my_bot/wheel.hpp"
#include "my_bot/arduino_comms.hpp"

namespace my_bot
{

struct Config {
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
};

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  DiffBotSystemHardware();
  
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  ArduinoComms comms_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  rclcpp::Logger logger_{rclcpp::get_logger("DiffBotSystemHardware")};
};

}  // namespace my_bot

#endif  // MY_BOT__DIFFBOT_SYSTEM_HPP_