#include <cstdio>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include "Eigen/Dense"
#include "Eigen/Core"


#define NUM_DRONES_TOPIC "/num_drones"
#define DRONES_ACTIVE_TOPIC "/drones_active"
#define DRONES_RADII_TOPIC "/drones_radii"
#define DRONE_STATES_TOPIC "/drones_states"
#define DRONE_GOALS_TOPIC "/drones_goals"

class SwarmPlannerNode: public rclcpp::Node {
public:
  SwarmPlannerNode();

private:
  int num_drones=-1;
  std::vector<bool> drones_active;
  std::vector<double> drone_radii;
  std::vector<Eigen::Vector4d> drone_states;
  std::vector<Eigen::Vector2d> drone_goals;

  std::vector<std::vector<Eigen::Vector2d>> drone_paths;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr num_drones_subscription;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr drones_active_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drones_radii;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drones_states_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drones_goals_subscription;

  std::vector<std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>>> path_publishers_vector;


  void num_drones_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void drones_active_subscription_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
};

SwarmPlannerNode::SwarmPlannerNode(): rclcpp::Node("swarm_planner_node") {
  RCLCPP_INFO_ONCE(this->get_logger(), "starting SwarmPlannerNode");

  this->num_drones_subscription = this->create_subscription<std_msgs::msg::Int32>(NUM_DRONES_TOPIC, 10,
                                                                                  std::bind(&SwarmPlannerNode::num_drones_subscription_callback,
                                                                                            this,
                                                                                            std::placeholders::_1));

  this->drones_active_subscription = this->create_subscription<std_msgs::msg::Int32MultiArray>(DRONES_ACTIVE_TOPIC, 10,
                                                                                               std::bind(&SwarmPlannerNode::drones_active_subscription_callback,
                                                                                                         this,
                                                                                                         std::placeholders::_1));
}

void SwarmPlannerNode::num_drones_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (this->num_drones == -1) {
    this->num_drones = msg->data;
    RCLCPP_INFO_ONCE(this->get_logger(), "setting num_drones to %d", this->num_drones);
    this->num_drones_subscription.reset();
  } else {
    RCLCPP_INFO_ONCE(this->get_logger(), "num_drones subscription has not been shutdown, retaining value of %d for num_drones", this->num_drones);
  }
}

void SwarmPlannerNode::drones_active_subscription_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  std::vector<int> drones_active_vector = msg->data;
  if (this->num_drones == -1) {
    RCLCPP_WARN_STREAM(this->get_logger(), "num_drones is not initialized");
  } else if (this->num_drones != drones_active_vector.size()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "size of the drones_active_array is " << drones_active_vector.size() << " when it is supposed to be " << this->num_drones);
  } else {
    this->drones_active.clear();
    for (int i=0; i < this->num_drones; i++) {
      this->drones_active.push_back(static_cast<bool>(drones_active_vector[i]));
    }
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  auto swarm_planner_node = std::make_shared<SwarmPlannerNode>();
  rclcpp::spin(swarm_planner_node);

  return 0;
}
