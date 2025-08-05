#include <ros/package.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <string>
#include <serial_port.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <filesystem>
#include <fstream>

#include "uvdar_ros_driver/UwbRange.h"
#include "uvdar_ros_driver/UwbRangeStamped.h"

extern "C" {
#include "llcp.h"
}

#include <thread>

#define SERIAL_BUFFER_SIZE 1024

namespace uvdar_ros_driver
{

class UvdarRosDriver : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  struct msg_counter
  {
    uint32_t  init;
    uint32_t  resp;
    uint16_t num;
  };

  struct range
  {
  uint8_t initiator_address_lo;
  uint8_t initiator_address_hi;
  uint8_t responder_address_lo;
  uint8_t responder_address_hi;
  uint8_t own_address_lo;
  uint8_t own_address_hi;
  uint32_t distance;  // Distance in mm
  uint32_t number;
  };

  std::vector<msg_counter> received_msgs;

  LLCP_Receiver_t llcp_receiver;

  serial_port::SerialPort serial_port_;
  std::string getDeviceSerialPort(const std::string& serial_short);
  bool openSerialPort(std::string st_serial_number, int baudrate);
  void serialThread(void);

  std::thread serial_thread_;

  void callbackSendMessage(const uvdar_ros_driver::UwbRangeStampedConstPtr &msg);
  void connectToSerial();
  void callbackMaintainerTimer(const ros::TimerEvent &event);
  void callbackConnectTimer(const ros::TimerEvent &event);

  ros::NodeHandle nh_;

  bool running_     = true;
  bool initialized_ = false;
  bool connected_   = false;

  std::string usb_serial_number_;
  const int   baudrate_ = 2000000; //  // Baudrate for UVDAR module, should be set to 2000000 or test higher values
  bool        debug_serial_;
  
  std::string frame_id_;

  ros::Publisher uvdar_publisher_;

  ros::Timer maintainer_timer_;

  std::mutex mutex_connected_;

  ros::Time maintainer_last_time_;
};

// std::string UvdarRosDriver::getDeviceSerialPort(const std::string& serial_short) {
//   namespace fs = std::filesystem;
//   for (const auto& entry : fs::directory_iterator("/sys/class/tty")) {
//     const std::string tty_name = entry.path().filename();
//     if (tty_name.find("ttyACM") != 0)
//       continue;

//     std::string device_path = entry.path().string() + "/device/../";
//     std::string serial_file = device_path + "serial";
//     std::ifstream serial_ifs(serial_file);
//     if (!serial_ifs.is_open())
//       continue;

//     std::string serial;
//     std::getline(serial_ifs, serial);
//     serial_ifs.close();

//     if (serial == serial_short) {
//       ROS_INFO("[UvdarRosDriver]: Found serial port for device with serial number %s: %s", serial_short.c_str(), tty_name.c_str());
//       //return "/dev/" + tty_name;
//     }
//   }
//   return "";
// }

std::string UvdarRosDriver::getDeviceSerialPort(const std::string& serial_short) {
  namespace fs = std::filesystem;
  std::vector<std::string> result;
  for (const auto& entry : fs::directory_iterator("/sys/class/tty")) {
    const std::string tty_name = entry.path().filename();
    if (tty_name.find("ttyACM") != 0)
      continue;

    std::string device_path = entry.path().string() + "/device/../";
    std::string serial_file = device_path + "serial";
    std::ifstream serial_ifs(serial_file);
    if (!serial_ifs.is_open())
      continue;

    std::string serial;
    std::getline(serial_ifs, serial);
    serial_ifs.close();

    if (serial == serial_short) {
      result.push_back("/dev/" + tty_name);
    }
  }
  // Seřadit podle čísla (ttyACM0, ttyACM1, ...)
  std::sort(result.begin(), result.end());
  return result[0];
}

void UvdarRosDriver::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[UvdarRosDriver]: node initialized");

  llcp_initialize(&llcp_receiver);

  ROS_INFO("[UvdarRosDriver]: llcp receiver initialized");

  // TODO: configure UVDAR module as soon as it is implemented on UVDAR firmware side
  // ROS_INFO("[UvdarRosDriver]: UVDAR module configured");

  nh_.getParam("uvdar_usb_serial", usb_serial_number_);
  nh_.param<std::string>("uvdar_frame_id", frame_id_, "uvdar");
  nh_.param<bool>("debug_serial", debug_serial_, false);

  uvdar_publisher_ = nh_.advertise<uvdar_ros_driver::UwbRangeStamped>("distance", 1);

  maintainer_timer_     = nh_.createTimer(ros::Rate(0.1), &UvdarRosDriver::callbackMaintainerTimer, this);
  maintainer_last_time_ = ros::Time::now();
  {
    std::scoped_lock lock(mutex_connected_);
    connected_ = false;
  }

  if (debug_serial_) {
    ROS_INFO("[UvdarRosDriver]: Serial Debug is enabled");
  }

  connectToSerial();
  initialized_ = true;
}

void UvdarRosDriver::connectToSerial() {

  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }

  bool tmp_conected = false;

  while (!tmp_conected) {
    tmp_conected = openSerialPort(usb_serial_number_, baudrate_);
  }

  {
    std::scoped_lock lock(mutex_connected_);
    connected_ = true;
  }

  serial_thread_ = std::thread(&UvdarRosDriver::serialThread, this);
}

bool UvdarRosDriver::openSerialPort(std::string serial_number, int baudrate) {

  std::string portname = "/dev/ttySTM_" + serial_number + "_00";
  ROS_INFO_THROTTLE(1.0, "[%s]: Trying to find serial port for device", ros::this_node::getName().c_str());
  if (serial_number.empty()) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Serial number is not set, available device serial numbers:", ros::this_node::getName().c_str());

    return false;
  }
  ROS_INFO_STREAM_THROTTLE(1.0, "SerialNumber: " << serial_number);
  portname = getDeviceSerialPort(serial_number);
  if (portname.empty()) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Device not found, it's serial port does not exist", ros::this_node::getName().c_str());
    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Openning serial port.", ros::this_node::getName().c_str());
  ROS_INFO_STREAM_THROTTLE(1.0, "Portname: " << portname << " baudrate: " << baudrate);

  if (!serial_port_.connect(portname, baudrate)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not connect to the serial port", ros::this_node::getName().c_str());
    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Connected to sensor.", ros::this_node::getName().c_str());

  return true;
}

void UvdarRosDriver::serialThread(void) {

  uint8_t  rx_buffer[SERIAL_BUFFER_SIZE];
  uint16_t bytes_read;

  ROS_INFO("[UvdarRosDriver]: serial thread starting");

  while (running_) {

    bool connected;
    {
      std::scoped_lock lock(mutex_connected_);
      connected = connected_;
    }
    if (!connected) {
      ROS_WARN("[UvdarRosDriver]: terminating serial thread because the serial port was disconnected");
      return;
    }

    bytes_read = serial_port_.readSerial(rx_buffer, SERIAL_BUFFER_SIZE);
    if (bytes_read > 0 && bytes_read < SERIAL_BUFFER_SIZE) {
      /* If the serial device is disconnected and readSerial() is called, it will return max_int number of read bytes, thats why we check it against the
       * SERIAL_BUFFER_SIZE */

      for (uint16_t i = 0; i < bytes_read; i++) {

        LLCP_Message_t *message_in;

        bool checksum_matched = false;
        if (debug_serial_) {
          std::cout << rx_buffer[i];
        }
        if (llcp_processChar(rx_buffer[i], &llcp_receiver, &message_in, &checksum_matched)) {
          /* ROS_INFO_STREAM("[UvdarRosDriver]: received message with id " << message_in->id << "; checksum is: " << checksum_matched); */
          struct range *range_msg = (struct range *)message_in->payload;
          uvdar_ros_driver::UwbRangeStamped msg_out;
          msg_out.header.stamp = ros::Time::now();
          msg_out.header.frame_id = frame_id_;
          msg_out.range.initiator_address = (range_msg->initiator_address_hi << 8) | (range_msg->initiator_address_lo);
          msg_out.range.responder_address = (range_msg->responder_address_hi << 8) | (range_msg->responder_address_lo);
          msg_out.range.own_address = (range_msg->own_address_hi << 8) | (range_msg->own_address_lo);
          msg_out.range.distance = range_msg->distance;
          uvdar_publisher_.publish(msg_out);

          std::vector<msg_counter>::iterator it =
              std::find_if(received_msgs.begin(), received_msgs.end(), boost::bind(&msg_counter::init, _1) == msg_out.range.initiator_address &&
                           boost::bind(&msg_counter::resp, _1) == msg_out.range.responder_address);
          if (it != received_msgs.end()) {
            it->num++;
          } else {
            msg_counter tmp;
            tmp.init  = msg_out.range.initiator_address;
            tmp.resp  = msg_out.range.responder_address;
            tmp.num = 1;
            received_msgs.push_back(tmp);
          }
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void UvdarRosDriver::callbackMaintainerTimer(const ros::TimerEvent &event) {

  bool connected;

  {
    std::scoped_lock lock(mutex_connected_);
    connected = connected_;
  }

  if (connected) {

    if (!serial_port_.checkConnected()) {
      {
        std::scoped_lock lock(mutex_connected_);
        connected_ = false;
      }
      ROS_ERROR("[UvdarRosDriver] Serial device is disconnected! ");
      connectToSerial();
    }
  }
  std::vector<std::string> testvector;
  size_t                   tmp_len = received_msgs.size();
  for (int i = 0; i < tmp_len; i++) {
    testvector.push_back("                                          ");
  }

  ROS_INFO_STREAM("------------------------------------------------");
  ROS_INFO("- uvdar ros driver stats for last %7.04f secs -", (ros::Time::now() - maintainer_last_time_).toSec());
  ROS_INFO_STREAM("received messages:");
  for (size_t i = 0; i < received_msgs.size(); i++) {
    std::string tmp_string = "initiator " + std::to_string(received_msgs[i].init) +", responder " + std::to_string(received_msgs[i].resp) + ": " + std::to_string(received_msgs[i].num) + " msgs";
    testvector[i].replace(5, tmp_string.length(), tmp_string);
  }
  received_msgs.clear();
  for (int i = 0; i < tmp_len; i++) {
    ROS_INFO_STREAM(testvector[i]);
  }
  ROS_INFO_STREAM("------------------------------------------------");

  maintainer_last_time_ = ros::Time::now();
}

}  // namespace end

PLUGINLIB_EXPORT_CLASS(uvdar_ros_driver::UvdarRosDriver, nodelet::Nodelet);
