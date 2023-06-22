#include <cstdio>
#include <iostream>
#include <string>

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
#define DRONES_STATES_TOPIC "/drones_states"
#define DRONES_GOALS_TOPIC "/drones_goals"

class SwarmPlannerNode: public rclcpp::Node {
public:
  SwarmPlannerNode();

private:
  int num_drones=-1;
  std::vector<bool> drones_active;
  std::vector<double> drones_radii;
  std::vector<Eigen::Vector4d> drones_states;
  std::vector<Eigen::Vector2d> drones_goals;

  std::vector<std::vector<Eigen::Vector2d>> drone_paths;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr num_drones_subscription;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr drones_active_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drones_radii_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drones_states_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drones_goals_subscription;

  std::vector<std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>>> path_publishers_vector;


  void num_drones_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void drones_active_subscription_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void drones_radii_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void drones_states_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void drones_goals_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  bool num_drones_initialized();
  bool check_planner_params();
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

  this->drones_radii_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(DRONES_RADII_TOPIC, 10,
                                                                                                std::bind(&SwarmPlannerNode::drones_radii_subscription_callback,
                                                                                                          this,
                                                                                                          std::placeholders::_1));

  this->drones_states_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(DRONES_STATES_TOPIC, 10,
                                                                                                 std::bind(&SwarmPlannerNode::drones_states_subscription_callback,
                                                                                                           this,
                                                                                                           std::placeholders::_1));

  this->drones_goals_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(DRONES_GOALS_TOPIC, 10,
                                                                                                std::bind(&SwarmPlannerNode::drones_goals_subscription_callback,
                                                                                                          this,
                                                                                                          std::placeholders::_1));
}

bool SwarmPlannerNode::num_drones_initialized() {
  if (this->num_drones == -1) {
    RCLCPP_WARN_STREAM(this->get_logger(), "num_drones is not initialized");
    return false;
  }

  return true;
}

bool SwarmPlannerNode::check_planner_params() {
  if (this->num_drones_initialized()) {
    if ((this->drones_active.size() == this->num_drones) &&
        (this->drones_radii.size() == this->num_drones) &&
        (this->drones_states.size() == this->num_drones) &&
        (this->drones_goals.size() == this->num_drones) &&
        (this->path_publishers_vector.size() == this->num_drones)) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}


void SwarmPlannerNode::num_drones_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (!this->num_drones_initialized()) {
    this->num_drones = msg->data;
    RCLCPP_INFO_ONCE(this->get_logger(), "setting num_drones to %d", this->num_drones);
    for (int i=0; i < this->num_drones; i++) {
      std::string drone_path_topic_name = "/drone_path";
      drone_path_topic_name.append(std::to_string(i));
      this->path_publishers_vector.push_back(
              this->create_publisher<nav_msgs::msg::Path>(drone_path_topic_name, 10));
    }
    this->num_drones_subscription.reset();
  } else {
    RCLCPP_INFO_ONCE(this->get_logger(), "num_drones subscription has not been shutdown, retaining value of %d for num_drones", this->num_drones);
  }
}

void SwarmPlannerNode::drones_active_subscription_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  std::vector<int> drones_active_vector = msg->data;
  if (this->num_drones_initialized()) {
    if (this->num_drones != drones_active_vector.size()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "size of the drones_active_array is " << drones_active_vector.size() << " when it is supposed to be " << this->num_drones);
    } else {
      this->drones_active.clear();
      for (int i = 0; i < this->num_drones; i++) {
        this->drones_active.push_back(static_cast<bool>(drones_active_vector[i]));
      }
    }
  }
}

void SwarmPlannerNode::drones_radii_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  std::vector<double> drones_radii_vector = msg->data;
  if (this->num_drones_initialized()) {
    if (this->num_drones != drones_radii_vector.size()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "size of the drones_radii_array is " << drones_radii_vector.size() << " when it is supposed to be " << this->num_drones);
    } else {
      this->drones_radii = drones_radii_vector;
    }
  }
}

void SwarmPlannerNode::drones_states_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  std::vector<double> drones_states_vector = msg->data;
  if (this->num_drones_initialized()) {
    if (this->num_drones * 4 != drones_states_vector.size()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "size of the drones_state_array is " << drones_states_vector.size() << " when it is supposed to be " << 4 * this->num_drones);
    } else {
      this->drones_states.clear();
      for (int i=0; i < 4 * this->num_drones; i=i+4) {
        this->drones_states.push_back(Eigen::Vector4d(drones_states_vector[i], drones_states_vector[i+1], drones_states_vector[i+2], drones_states_vector[i+3]));
      }
    }
  }
}

void SwarmPlannerNode::drones_goals_subscription_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  std::vector<double> drones_goals_vector = msg->data;
  if (this->num_drones_initialized()) {
    if (this->num_drones * 2 != drones_goals_vector.size()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "size of the drones_goals_array is " << drones_goals_vector.size() << " when it is supposed to be " << 2 * this->num_drones);
    } else {
      this->drones_goals.clear();
      for (int i=0; i < 2 * this->num_drones; i=i+2) {
        this->drones_goals.push_back(Eigen::Vector2d(drones_goals_vector[i], drones_goals_vector[i+1]));
      }
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
