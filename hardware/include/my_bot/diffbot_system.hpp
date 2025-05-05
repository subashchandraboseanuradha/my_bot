#ifndef MY_BOT_DIFFBOT_SYSTEM_HPP
#define MY_BOT_DIFFBOT_SYSTEM_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "my_bot/visibility_control.h"
#include "my_bot/arduino_comms.hpp"
#include "my_bot/wheel.hpp"

namespace my_bot
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  MY_BOT_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MY_BOT_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MY_BOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MY_BOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MY_BOT_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MY_BOT_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MY_BOT_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MY_BOT_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double motor_power_multiplier_;
  
  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Wheel
  std::unique_ptr<Wheel> wheel_l_;
  std::unique_ptr<Wheel> wheel_r_;

  // Arduino Communication
  std::unique_ptr<ArduinoComms> comms_;
  
  struct Config
  {
    std::string left_wheel_name = "";
    std::string right_wheel_name = "";
    float loop_rate = 0.0;
    std::string device = "";
    int baud_rate = 0;
    int timeout_ms = 0;
    int enc_counts_per_rev = 0;
    double motor_power_multiplier = 0.0;
  };

  Config cfg_;
};

}  // namespace my_bot

#endif  // MY_BOT_DIFFBOT_SYSTEM_HPP 