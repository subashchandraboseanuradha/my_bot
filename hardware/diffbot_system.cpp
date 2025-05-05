#include "my_bot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_bot
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
  // Get motor power multiplier from parameters (default to 1.0 if not specified)
  if (info_.hardware_parameters.count("motor_power_multiplier") > 0) {
    cfg_.motor_power_multiplier = std::stod(info_.hardware_parameters["motor_power_multiplier"]);
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Motor power multiplier set to: %f", cfg_.motor_power_multiplier);
  } else {
    cfg_.motor_power_multiplier = 1.0;
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Motor power multiplier not specified, using default: %f", cfg_.motor_power_multiplier);
  }

  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configuring ...please wait...");
  
  // Initialize instance first
  comms_ = std::make_unique<ArduinoComms>();
  // Then connect
  comms_->connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  
  // Create wheel instances
  wheel_l_ = std::make_unique<Wheel>(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_ = std::make_unique<Wheel>(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Starting ...please wait...");

  // Set initial values for positions and velocities
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Hardware started, ready to go!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Stopping ...please wait...");
  
  // Stop motors
  comms_->set_motor_values(0, 0);

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read encoder values
  int enc_l = 0;
  int enc_r = 0;
  
  if (comms_->connected())
  {
    comms_->read_encoder_values(enc_l, enc_r);
    
    // Update wheel encoder values and calculate positions
    wheel_l_->enc = enc_l;
    wheel_r_->enc = enc_r;
    
    // Calculate wheel positions and update
    hw_positions_[0] = wheel_l_->calc_enc_angle();
    hw_positions_[1] = wheel_r_->calc_enc_angle();
    
    // Set velocities (these would ideally be calculated from position changes over time)
    // For simplicity, we'll just use the commands as velocities for now
    hw_velocities_[0] = wheel_l_->vel;
    hw_velocities_[1] = wheel_r_->vel;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Serial connection not established.");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Save the commands to the wheels
  wheel_l_->cmd = hw_commands_[0];
  wheel_r_->cmd = hw_commands_[1];
  
  // Apply the multiplier to the motor commands to increase power
  int motor_l = static_cast<int>(hw_commands_[0] * cfg_.motor_power_multiplier);
  int motor_r = static_cast<int>(hw_commands_[1] * cfg_.motor_power_multiplier);
  
  RCLCPP_DEBUG(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Sending commands to motors: left: %d, right: %d", motor_l, motor_r);
  
  if (comms_->connected())
  {
    comms_->set_motor_values(motor_l, motor_r);
    
    // Update wheel velocities based on commands
    wheel_l_->vel = wheel_l_->cmd;
    wheel_r_->vel = wheel_r_->cmd;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Serial connection not established.");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace my_bot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_bot::DiffBotSystemHardware, hardware_interface::SystemInterface) 