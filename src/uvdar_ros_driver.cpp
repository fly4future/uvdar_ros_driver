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
  std::map<std::string, std::vector<std::string>> listAllDeviceSerialPorts();
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

std::map<std::string, std::vector<std::string>> UvdarRosDriver::listAllDeviceSerialPorts() {
  namespace fs = std::filesystem;
  std::map<std::string, std::vector<std::string>> serial_map;

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

    std::string tty_full_path = "/dev/" + tty_name;
    serial_map[serial].push_back(tty_full_path);
  }

  // Sort each list of tty devices for every serial
  for (auto& [serial, ttys] : serial_map) {
    std::sort(ttys.begin(), ttys.end());
  }

  return serial_map;
}

std::string UvdarRosDriver::getDeviceSerialPort(const std::string& serial_short) {
  auto serial_map = listAllDeviceSerialPorts();
  auto it = serial_map.find(serial_short);
  if (it == serial_map.end() || it->second.empty()) {
    return "";
  }
  return it->second.front();
}

void UvdarRosDriver::onInit() {

  // Get paramters
  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[%s]: Node initialized", ros::this_node::getName().c_str());

  llcp_initialize(&llcp_receiver);

  ROS_INFO("[%s]: LLCP receiver initialized", ros::this_node::getName().c_str());

  // TODO: configure UVDAR module as soon as it is implemented on UVDAR firmware side
  // ROS_INFO("[UvdarRosDriver]: UVDAR module configured");

  nh_.getParam("uvdar_usb_serial", usb_serial_number_);
  nh_.param<std::string>("uvdar_frame_id", frame_id_, "uvdar");
  nh_.param<bool>("debug_serial", debug_serial_, false);

  uvdar_publisher_ = nh_.advertise<uvdar_ros_driver::UwbRangeStamped>("distance", 1);

  maintainer_timer_     = nh_.createTimer(ros::Rate(0.2), &UvdarRosDriver::callbackMaintainerTimer, this);
  maintainer_last_time_ = ros::Time::now();
  {
    std::scoped_lock lock(mutex_connected_);
    connected_ = false;
  }

  if (debug_serial_) {
    ROS_INFO("[%s]: Serial Debug is enabled", ros::this_node::getName().c_str());
  }

  connectToSerial();
  initialized_ = true;
}

void UvdarRosDriver::connectToSerial() {

  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }

  bool tmp_connected = false;

  while (!tmp_connected) {
    tmp_connected = openSerialPort(usb_serial_number_, baudrate_);
    if (!tmp_connected && usb_serial_number_.empty())
    {
      ros::shutdown(); // Clean exit
      return;
    }
  }

  {
    std::scoped_lock lock(mutex_connected_);
    connected_ = true;
  }

  serial_thread_ = std::thread(&UvdarRosDriver::serialThread, this);
}

bool UvdarRosDriver::openSerialPort(std::string serial_number, int baudrate) {

  std::map<std::string, std::vector<std::string>> serial_map = listAllDeviceSerialPorts();
  std::string portname = getDeviceSerialPort(serial_number);
  if (serial_number.empty()) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Serial number is not set, available device serial numbers:", ros::this_node::getName().c_str());
    for (const auto& [serial, ttys] : serial_map) {
      std::ostringstream oss;
      oss << "Device " << serial << ": ";
      
      for (size_t i = 0; i < ttys.size(); ++i) {
        const std::string& tty_path = ttys[i];
        std::string tty_name = tty_path.substr(tty_path.find_last_of('/') + 1);
        oss << tty_name;
        if (i < ttys.size() - 1)
        oss << ", ";
      }
      
      ROS_ERROR_STREAM(oss.str()); // Log the constructed message
    }
    return false;
  }
  ROS_INFO_THROTTLE(1.0, "[%s]: Trying to find serial port for device:", ros::this_node::getName().c_str());
  ROS_INFO_STREAM_THROTTLE(1.0, std::string(ros::this_node::getName().length() +4,' ') << "SerialNumber: " << serial_number);
  portname = getDeviceSerialPort(serial_number);
  if (portname.empty()) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Device not found, it's serial port does not exist", ros::this_node::getName().c_str());
    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[%s]: Opening serial port:", ros::this_node::getName().c_str());
  ROS_INFO_STREAM_THROTTLE(1.0, std::string(ros::this_node::getName().length() +4,' ') << "Portname: " << portname);
  ROS_INFO_STREAM_THROTTLE(1.0, std::string(ros::this_node::getName().length() +4,' ') << "Baudrate: " << baudrate);

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

  ROS_INFO("[%s]: Serial thread starting", ros::this_node::getName().c_str());

  while (running_) {

    bool connected;
    {
      std::scoped_lock lock(mutex_connected_);
      connected = connected_;
    }
    if (!connected) {
      ROS_WARN("[%s]: Terminating serial thread because the serial port was disconnected", ros::this_node::getName().c_str());
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
      ROS_ERROR("[%s] Serial device is disconnected!", ros::this_node::getName().c_str());
      connectToSerial();
    }
  }
  std::vector<std::string> testvector;
  size_t                   tmp_len = received_msgs.size();
  for (int i = 0; i < tmp_len; i++) {
    testvector.push_back("                                          ");
  }

  ROS_INFO_STREAM("------------------------------------------------");
  ROS_INFO("- uvdar_ros_driver stats for last %7.04f secs -", (ros::Time::now() - maintainer_last_time_).toSec());
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
