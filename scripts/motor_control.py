#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import threading

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Serial port configuration
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=1.0,
                write_timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(2)  # Wait for serial connection to stabilize
            self.get_logger().info('Successfully connected to /dev/ttyUSB0')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            raise
        
        # Constants for commands
        self.CMD_MOTOR_SPEEDS = 'm'
        self.CMD_READ_ENCODERS = 'e'
        self.CMD_RESET_ENCODERS = 'r'
        
        # Create subscription to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Create timer for reading encoders
        self.create_timer(0.1, self.read_encoders)
        
        # Initialize by resetting encoders
        self.reset_encoders()
        self.get_logger().info('Motor Controller initialized')
        
    def send_command(self, cmd, wait_for_response=True):
        """Helper function to send commands and read responses"""
        try:
            # Clear buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # Send command with proper termination
            full_command = f"{cmd}\r\n"
            self.get_logger().debug(f"Sending command: {full_command.strip()}")
            self.serial_port.write(full_command.encode())
            self.serial_port.flush()
            
            if wait_for_response:
                time.sleep(0.05)  # Wait for response
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    self.get_logger().debug(f"Received response: {response}")
                    return response
            return None
        except Exception as e:
            self.get_logger().error(f"Error sending command '{cmd}': {str(e)}")
            return None

    def joy_callback(self, msg):
        try:
            # Check if 'A' button (usually button[0]) is pressed for encoder reset
            if len(msg.buttons) > 0 and msg.buttons[0]:
                self.reset_encoders()
                return

            # Get joystick values
            if len(msg.axes) >= 2:
                forward = msg.axes[1]
                turn = msg.axes[0]
                
                # Convert joystick values to motor speeds
                left_speed = int((forward + turn) * 127)
                right_speed = int((forward - turn) * 127)
                
                # Clamp values
                left_speed = max(-127, min(127, left_speed))
                right_speed = max(-127, min(127, right_speed))
                
                # Format command: m <left> <right>
                command = f"{self.CMD_MOTOR_SPEEDS} {left_speed} {right_speed}"
                self.send_command(command, wait_for_response=False)
                self.get_logger().debug(f'Motor speeds - Left: {left_speed}, Right: {right_speed}')
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {str(e)}')
    
    def read_encoders(self):
        response = self.send_command(self.CMD_READ_ENCODERS)
        if response:
            try:
                # Expected format: <left_enc> <right_enc>
                values = response.split()
                if len(values) >= 2:
                    left_enc, right_enc = map(int, values[:2])
                    self.get_logger().info(f'Encoders - Left: {left_enc}, Right: {right_enc}')
                else:
                    self.get_logger().warning(f'Invalid encoder response format: {response}')
            except ValueError as e:
                self.get_logger().warning(f'Invalid encoder values in response: {response}')
    
    def reset_encoders(self):
        response = self.send_command(self.CMD_RESET_ENCODERS)
        self.get_logger().info('Encoders reset command sent')
        if response:
            self.get_logger().info(f'Reset response: {response}')
    
    def cleanup(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            # Stop motors before closing
            self.send_command(f"{self.CMD_MOTOR_SPEEDS} 0 0", wait_for_response=False)
            time.sleep(0.1)
            self.serial_port.close()
            self.get_logger().info('Serial port closed')

def main(args=None):
    rclpy.init(args=args)
    controller = None
    try:
        controller = MotorController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if controller:
            controller.cleanup()
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()