#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

std::array<double, 36> get_pose_covariance()
{
	//create row major covariance matrix for pose
	// may need to tune these values
	std::array<double, 36>pose_cov{};
	pose_cov.fill(0.0);

	pose_cov[0] = 0.05; //cov(x)
	pose_cov[7] = 0.05; //cov(y)
	pose_cov[35] = 0.1; //cov(yaw)

	return pose_cov;
}

std::array<double, 36> get_twist_covariance()
{
	std::array<double, 36>twist_cov{};
	twist_cov.fill(0.0);

	twist_cov[0] = 0.1; //cov(vx)
	twist_cov[7] = 0.1; //cov(vy)
	twist_cov[35] = 0.2; //cov(wz)

	return twist_cov;

}

uint8_t xor_checksum(uint8_t* data, const uint32_t len)
{
	uint8_t checksum = 0;
	for(uint32_t i = 0; i < len; i++)
	{
		checksum ^= data[i];
	}
	return checksum;
}

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

      pose_cov_ = get_pose_covariance();
      twist_cov_ = get_twist_covariance();

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
          char buffer[2 + 4 * 3 + 1 + 2];
          buffer[0] = 0xFF;
          buffer[1] = 0xFF;
          *(float*)(buffer+2) = static_cast<float>(msg->linear.x);
          *(float*)(buffer+6) = static_cast<float>(msg->linear.y);
          *(float*)(buffer+10) = static_cast<float>(msg->angular.z);
          uint8_t data[4*3];
          const uint32_t DATA_LEN = sizeof(data)/sizeof(data[0]);
          memcpy(&data, &buffer[2], DATA_LEN);
          buffer[14] = (char)xor_checksum(data, DATA_LEN);
          buffer[15] = 0xEE;
          buffer[16] = 0xEE;

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
            char buf[3*4 + 3*4 + 1 + 2];
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

			uint8_t recieved_checksum = buf[LEN - 3];
			uint8_t data[2 * 3 * 4];
			const uint32_t DATA_LEN = sizeof(data)/sizeof(data[0]);
			memcpy(&data, &buf, DATA_LEN);
			uint8_t calculated_checksum = xor_checksum(data, DATA_LEN);
			if(recieved_checksum != calculated_checksum)
			{
				RCLCPP_ERROR(this->get_logger(), "Corrupted packet (checksum mismatch)");
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
            msg.pose.covariance = pose_cov_;

            msg.twist.twist.linear.x = odom.velx;
            msg.twist.twist.linear.y = odom.vely;
            msg.twist.twist.angular.z = odom.veltheta;
            msg.twist.covariance = twist_cov_;

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
	std::array<double, 36> pose_cov_;
	std::array<double, 36> twist_cov_;
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
