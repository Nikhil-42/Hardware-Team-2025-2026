#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class Chassis : public rclcpp::Node
{
  public:
    Chassis() : Node("chassis")
    {
      this->declare_parameter("device", "/dev/ttyAMA0");
      std::string device = this->get_parameter("device").as_string();

      RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s", device.c_str());
      this->serial_port_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

      if (this->serial_port_ < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
      }

      struct termios tty;
      if (tcgetattr(this->serial_port_, &tty) != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
      }

      // Set baud rate (input and output)
      // Use standard constants like B9600, B19200, B115200, B230400, etc.
      cfsetospeed(&tty, B115200);
      cfsetispeed(&tty, B115200);
      tcsetattr(this->serial_port_, TCSANOW, &tty);

      this->cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          // Serialize and send the command over UART
          char buffer[2 + 4 * 3 + 2];
          buffer[0] = 0xFF;
          buffer[1] = 0xFF;
          *(float*)(buffer+2) = static_cast<float>(msg->linear.x);
          *(float*)(buffer+6) = static_cast<float>(msg->linear.y);
          *(float*)(buffer+10) = static_cast<float>(msg->angular.z);
          buffer[14] = 0xEE;
          buffer[15] = 0xEE;

          ssize_t n = write(this->serial_port_, buffer, sizeof(buffer));
          if (n < 0)
          {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", strerror(errno));
          } else {
            RCLCPP_INFO(this->get_logger(), "Wrote %zd bytes to serial port", n);
          }
        });

      this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
      this->read_thread_ = std::thread([this]() {
        while (rclcpp::ok())
        {
          // Align to first 0xFFFF
          uint8_t count = 0;
          while (rclcpp::ok() && count < 2) {
            // Read byte
            uint8_t c;
            read(this->serial_port_, &c, 1);

            if (c == 0xFF) {
              count++;
            }
            else {
              count = 0;
            }
          }

          // Read payload + tail
          char buf[3*4 + 3*4 + 2];
          const uint32_t LEN = sizeof(buf) / sizeof(buf[0]);

          ssize_t idx = 0;
          while (idx < LEN) {
            ssize_t n = read(this->serial_port_, buf, sizeof(buf));
            if (n < 0)
            {
              RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
              break;
            } else {
              idx += n;
            }
          }

          if (idx < LEN) {
            continue;
          }

          if (buf[LEN - 2] == 0xEE && buf[LEN - 1] == 0xEE)
          {

            // RCLCPP_INFO(this->get_logger(), "Read packet");

            auto msg = nav_msgs::msg::Odometry();
            msg.pose.pose.position.x = *(float*)(buf+0);
            msg.pose.pose.position.y = *(float*)(buf+4);
            // quaternion from angle in radians
            float a = *(float*)(buf+8);
            msg.pose.pose.orientation.x = cosf32(a / 2);
            msg.pose.pose.orientation.w = sinf32(a / 2);

            msg.twist.twist.linear.x = *(float*)(buf+12);
            msg.twist.twist.linear.y = *(float*)(buf+16);
            msg.twist.twist.angular.z = *(float*)(buf+20);

            this->odom_pub->publish(msg);
          }
          else {
            RCLCPP_ERROR(this->get_logger(), "Malformed packet (missing tail)");
          }
          std::this_thread::sleep_for(100ms); // Adjust sleep duration as needed
        }
      });
      this->read_thread_.detach();
    }
  private:
    int serial_port_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    std::thread read_thread_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Chassis>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
