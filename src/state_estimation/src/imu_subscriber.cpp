#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_broadcaster/imu",
            10,
            std::bind(
                &ImuSubscriber::imu_callback,
                this,
                std::placeholders::_1
            )
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Imu subscriber started"
        );
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Angular Velocity: x=%.3f y=%.3f z=%.3f |"
            "Linear Acceleration: x=%.3f y=%.3f z=%.3f",
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z,
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}