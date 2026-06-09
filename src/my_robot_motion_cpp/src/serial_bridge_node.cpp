#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <sstream>

class SerialBridgeNode : public rclcpp::Node
{
public:

    SerialBridgeNode()
    : Node("serial_bridge_node")
    {
        this->declare_parameter<std::string>(
            "port",
            "/dev/ttyUSB0"
        );

        std::string port =
            this->get_parameter("port").as_string();

        serial_fd_ =
            open(port.c_str(),
                 O_RDWR | O_NOCTTY);

        if (serial_fd_ < 0)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Cannot open serial port %s",
                port.c_str()
            );

            rclcpp::shutdown();
            return;
        }

        struct termios tty;

        tcgetattr(serial_fd_, &tty);

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag |= CREAD | CLOCAL;

        tcsetattr(
            serial_fd_,
            TCSANOW,
            &tty
        );

        sub_ =
            create_subscription<
                sensor_msgs::msg::JointState>(
                "/arm_joint_targets",
                10,
                std::bind(
                    &SerialBridgeNode::callback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            get_logger(),
            "Serial bridge ready"
        );
    }

private:

    void callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6)
            return;

        std::stringstream ss;

        ss
            << msg->position[0] << " "
            << msg->position[1] << " "
            << msg->position[2] << " "
            << msg->position[3] << " "
            << msg->position[4] << " "
            << msg->position[5] << "\n";

        std::string out = ss.str();

        write(
            serial_fd_,
            out.c_str(),
            out.size()
        );

        RCLCPP_INFO(
            get_logger(),
            "TX: %s",
            out.c_str()
        );
    }

    int serial_fd_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node =
        std::make_shared<SerialBridgeNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}