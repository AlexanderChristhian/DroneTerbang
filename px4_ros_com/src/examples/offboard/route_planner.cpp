#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <cmath>
#include <vector>

class RoutePlanner : public rclcpp::Node
{
public:
    RoutePlanner() : Node("route_planner"), current_setpoint_index_(0)
    {
        // Publisher to trajectory setpoint
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        // Subscriber to the vehicle's current position with compatible QoS settings
        rclcpp::QoS qos{rclcpp::SensorDataQoS()};
        position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&RoutePlanner::positionCallback, this, std::placeholders::_1));

        // Define the route (list of waypoints in NED with yaw)
        route_ = {
            {0.0, 0.0, -1.0, 1.57}, // Waypoint 1: 10m North, 0m East, 2m above ground, 0 rad yaw
            {0.0, 6.0, -1.0, 1.57}, // Waypoint 2: 10m North, 10m East, 2m above ground, 90 degrees yaw
            {0.0, 6.0, -1.0, 0}, // Waypoint 2: 10m North, 10m East, 2m above ground, 90 degrees yaw
            {6.0, 6.0, -1.0, 0},  // Waypoint 3: 0m North, 10m East, 2m above ground, 180 degrees yaw
            {6.0, 6.0, -1.25, 0},    // Waypoint 4: Return to start, -90 degrees yaw
            {11.0, 6.0, -1.25, 0},    // Waypoint 5: Return to start, 0 rad yaw
            {11.0, 6.0, -1.25, 0},    // Waypoint 6: Return to start, 0 rad yaw
            {11.0, 6.0, 0, 0}    // Waypoint 6: Return to start, 0 rad yaw
        };

        tolerance_ = 0.1; // Position tolerance in meters

        // Timer to publish the initial setpoint
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RoutePlanner::publishCurrentSetpoint, this));
    }

private:
    void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = msg;
    }

    void publishCurrentSetpoint()
    {
        if (!current_position_) {
            return;
        }

        // Debug information for current drone location
        RCLCPP_INFO(this->get_logger(), "Current drone location: [%.2f, %.2f, %.2f]", current_position_->x, current_position_->y, current_position_->z);

        // Check if the drone is within the tolerance of the current setpoint
        bool within_tolerance = std::abs(route_[current_setpoint_index_][0] - current_position_->x) <= tolerance_ &&
                                std::abs(route_[current_setpoint_index_][1] - current_position_->y) <= tolerance_ &&
                                std::abs(route_[current_setpoint_index_][2] - current_position_->z) <= tolerance_;

        if (within_tolerance)
        {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_setpoint_index_ + 1);

            // Move to the next waypoint if available
            if (current_setpoint_index_ < route_.size() - 1)
            {
                current_setpoint_index_++;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Route completed");
                rclcpp::shutdown();
                return;
            }
        }

        // Publish the current trajectory setpoint
        auto msg = px4_msgs::msg::TrajectorySetpoint();
        msg.position[0] = route_[current_setpoint_index_][0];
        msg.position[1] = route_[current_setpoint_index_][1];
        msg.position[2] = route_[current_setpoint_index_][2];
        msg.yaw = route_[current_setpoint_index_][3];
        msg.velocity[0] = 0.0;
        msg.velocity[1] = 0.0;
        msg.velocity[2] = 0.0;

        trajectory_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing waypoint %zu: [%.2f, %.2f, %.2f, %.2f]",
                    current_setpoint_index_ + 1,
                    msg.position[0],
                    msg.position[1],
                    msg.position[2],
                    msg.yaw);
    }

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr delay_timer_;

    std::vector<std::array<double, 4>> route_; // List of waypoints with yaw
    size_t current_setpoint_index_; // Index of the current setpoint
    double tolerance_; // Tolerance for reaching waypoints
    px4_msgs::msg::VehicleLocalPosition::SharedPtr current_position_; // Current position of the drone
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoutePlanner>());
    rclcpp::shutdown();
    return 0;
}
