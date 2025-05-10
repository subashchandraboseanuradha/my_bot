#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>  // For std::clamp

// Helper function to clamp values
template<typename T>
T clamp_value(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    try {
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    } catch (const LibSerial::OpenFailed&) {
      std::cerr << "Failed to open serial port: " << serial_device << std::endl;
      throw;
    }
  }

  void disconnect()
  {
    try {
      serial_conn_.Close();
    } catch (const std::exception& e) {
      std::cerr << "Error during disconnect: " << e.what() << std::endl;
    }
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  void flush_buffers()
  {
    try {
      serial_conn_.FlushIOBuffers();
    } catch (const std::exception& e) {
      std::cerr << "Error flushing buffers: " << e.what() << std::endl;
    }
  }

  bool send_msg(const std::string &msg_to_send, bool print_output = false, int retry_count = 3)
  {
    while (retry_count-- > 0) {
      try {
        flush_buffers();
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try {
          serial_conn_.ReadLine(response, '\n', timeout_ms_);
        } catch (const LibSerial::ReadTimeout&) {
          if (print_output) {
            std::cerr << "Read timeout occurred." << std::endl;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }

        if (print_output) {
          std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }
        return true;
      } catch (const std::exception& e) {
        std::cerr << "Serial error (attempts left: " << retry_count << "): " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    return false;
  }

  void send_empty_msg()
  {
    send_msg("\r");
  }

  bool read_encoder_values(int &val_1, int &val_2)
  {
    try {
      flush_buffers();
      serial_conn_.Write("e\r");

      std::string response = "";
      try {
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
      } catch (const LibSerial::ReadTimeout&) {
        std::cerr << "Read timeout occurred while reading encoders." << std::endl;
        return false;
      }

      if (response.empty()) {
        return false;
      }

      std::string delimiter = " ";
      size_t del_pos = response.find(delimiter);
      if (del_pos == std::string::npos) {
        return false;
      }
      
      std::string token_1 = response.substr(0, del_pos);
      std::string token_2 = response.substr(del_pos + delimiter.length());

      val_1 = std::atoi(token_1.c_str());
      val_2 = std::atoi(token_2.c_str());
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Error reading encoder values: " << e.what() << std::endl;
      return false;
    }
  }

  bool set_motor_values(int val_1, int val_2)
  {
    // Clamp values to valid range
    val_1 = clamp_value(val_1, -255, 255);
    val_2 = clamp_value(val_2, -255, 255);
    
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    return send_msg(ss.str());
  }

  bool set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    return send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP