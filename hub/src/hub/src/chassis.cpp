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

int read_blocking(int fd, void* buf, size_t nbytes) {
  size_t idx = 0;
  while (idx < nbytes) {
    ssize_t n = read(fd, (buf + idx), nbytes - idx);
    if (n <= 0) {
      return n;
    }
    idx += n;
  }

  return idx;
}

struct odom_t {
  float velx, vely, veltheta;
  float posex, posey, posetheta;
};

class Chassis : public rclcpp::Node
{
  public:
    Chassis() : Node("chassis")
    {
      this->declare_parameter("device", "/dev/ttyAMA0");
      std::string device = this->get_parameter("device").as_string();

      RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s", device.c_str());
      this->serial_port_ = open(device.c_str(), O_RDWR | O_NOCTTY);

      if (this->serial_port_ < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
      }

      struct termios tty;
      struct termios old_tty;

      if (tcgetattr(this->serial_port_, &old_tty) != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
      }
      tty = old_tty;

      // Set baud rate (input and output)
      cfsetospeed(&tty, B115200);
      cfsetispeed(&tty, B115200);
      cfmakeraw(&tty);

      if (tcsetattr(this->serial_port_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
      };

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
            // RCLCPP_INFO(this->get_logger(), "Wrote %zd bytes to serial port", n);
          }
        });

      this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
      this->read_thread_ = std::thread([this]() {
        while (rclcpp::ok())
        {
          RCLCPP_INFO(this->get_logger(), "Scanning for start word (0xFFFF)...");
          // Align to first 0xFFFF
          uint8_t count = 0;
          while (rclcpp::ok() && count < 2) {
            // Read byte
            uint8_t c;
            int n = read(this->serial_port_, &c, 1);
            if (n < 0) {
              RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
            } else if (n > 0) {
              RCLCPP_INFO(this->get_logger(), "Read %d", c);
              if (c == 0xFF) {
                count++;
              }
              else {
                count = 0;
              }
            }
          }

          RCLCPP_INFO(this->get_logger(), "Start word found!");
          
          while (rclcpp::ok()) {
            // Read payload + tail
            char buf[3*4 + 3*4 + 2];
            const uint32_t LEN = sizeof(buf) / sizeof(buf[0]);

            ssize_t n = read_blocking(this->serial_port_, buf, sizeof(buf));
            if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
                break;
            }

            if (buf[LEN - 2] != 0xEE && buf[LEN - 1] != 0xEE)
            {
              RCLCPP_ERROR(this->get_logger(), "Malformed packet (missing tail)");
              break;
            }

            // RCLCPP_INFO(this->get_logger(), "Before: ");
            // for (size_t i = 0; i+3 < LEN; i += 4) {
            //   RCLCPP_INFO(this->get_logger(), "%02x%02x%02x%02x", buf[i], buf[i+1], buf[i+2], buf[i+3]);
            // }
            
            for (size_t i = 0; i < 6; ++i) {
              char flip[4];
              for (size_t j = 0; j < 4; ++j) {
                flip[j] = buf[i * 4 + 3 - j];
              }
              for (size_t j = 0; j < 4; ++j) {
                buf[i * 4 + j] = flip[j];
              }
            }

            // RCLCPP_INFO(this->get_logger(), "After: ");
            // for (size_t i = 0; i+3 < LEN; i += 4) {
            //   RCLCPP_INFO(this->get_logger(), "%02x%02x%02x%02x", buf[i], buf[i+1], buf[i+2], buf[i+3]);
            // }

            odom_t odom;
            memcpy(&odom, &buf, sizeof(odom_t));

            auto msg = nav_msgs::msg::Odometry();
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "base_link";

            msg.pose.pose.position.x = odom.posex;
            msg.pose.pose.position.y = odom.posey;

            msg.pose.pose.orientation.z = sinf32(odom.posetheta / 2.0f);
            msg.pose.pose.orientation.w = cosf32(odom.posetheta / 2.0f);

            msg.twist.twist.linear.x = odom.velx;
            msg.twist.twist.linear.y = odom.vely;
            msg.twist.twist.angular.z = odom.veltheta;

            this->odom_pub->publish(msg);

            uint16_t start_word;
            n = read_blocking(this->serial_port_, &start_word, 2);
            if (n < 0) {
              RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
              break;
            }

            if (start_word != 0xFFFF) {
              RCLCPP_ERROR(this->get_logger(), "Invalid start word: %x", start_word);
              break;
            }
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
