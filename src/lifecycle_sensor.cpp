#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/*
 * TODO: Create a Class named 'LifecycleSensor' that inherits from rclcpp_lifecycle::LifecycleNode.
 * Requirements:
 * 1. The constructor should name the node "lifecycle_sensor".
 * 2. Implement lifecycle callbacks:
 * - on_configure: Initialize publisher, log "Sensor configured"
 * - on_activate: Start timer (500ms), log "Sensor activated"
 * - on_deactivate: Stop timer, log "Sensor deactivated"
 * - on_cleanup: Reset publisher, log "Sensor cleaned up"
 * - on_shutdown: Log "Sensor shutting down"
 * 3. Timer should publish random values (0-100) to "/sensor_data"
 */

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleSensor()
        : LifecycleNode("lifecycle_sensor"),
          gen_(rd_()),
          dist_(0.0, 100.0)
    {
        RCLCPP_INFO(this->get_logger(), "Lifecycle sensor node created");
    }

    // TODO: Implement on_configure callback
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sensor_data", 10);
        RCLCPP_INFO(this->get_logger(), "Sensor configured");
        return CallbackReturn::SUCCESS;
    }

    // TODO: Implement on_activate callback
    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&LifecycleSensor::timer_callback, this));
        auto lc_pub = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>(publisher_);
        if (lc_pub) {
            lc_pub->on_activate();
        }
        RCLCPP_INFO(this->get_logger(), "Sensor activated");
        return CallbackReturn::SUCCESS;
    }

    // TODO: Implement on_deactivate callback
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        auto lc_pub = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>(publisher_);
        if (lc_pub) {
            lc_pub->on_deactivate();
        }
        RCLCPP_INFO(this->get_logger(), "Sensor deactivated");
        return CallbackReturn::SUCCESS;
    }

    // TODO: Implement on_cleanup callback
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        publisher_.reset();
        RCLCPP_INFO(this->get_logger(), "Sensor cleaned up");
        return CallbackReturn::SUCCESS;
    }

    // TODO: Implement on_shutdown callback
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        publisher_.reset();
        RCLCPP_INFO(this->get_logger(), "Sensor shutting down");
        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = dist_(gen_);
        RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", msg.data);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleSensor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}