// SafetyNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>

class SafetyNode : public BT::SyncActionNode
{
public:
  SafetyNode(const std::string &name, const BT::NodeConfiguration &config)
      : SyncActionNode(name, config)
  {
    node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
    range_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
        "<YOUR_RANGE_SENSOR_TOPIC>", 10,
        std::bind(&SafetyNode::rangeCallback, this, std::placeholders::_1));
    safety_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "<YOUR_SAFETY_OVERRIDE_TOPIC>", 10);
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // TODO: check range_; if too close, override
    std_msgs::msg::Bool msg;
    msg.data = (range_.range < MIN_SAFE_DISTANCE);
    safety_pub_->publish(msg);

    return msg.data
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
  }

private:
  void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    range_ = *msg;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_pub_;
  sensor_msgs::msg::Range range_;
  static constexpr double MIN_SAFE_DISTANCE = 0.5;
};
