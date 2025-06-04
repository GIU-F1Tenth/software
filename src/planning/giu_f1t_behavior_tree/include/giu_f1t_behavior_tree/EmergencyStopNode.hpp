// EmergencyStopNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

class EmergencyStopNode : public BT::SyncActionNode
{
public:
    EmergencyStopNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_->declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
        node_->declare_parameter<std::string>("cmd_vel_topic", "/ackermann_cmd");

        this->get_parameter("emergency_stop_topic", emergency_stop_topic_);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);

        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        emergency_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            emergency_stop_topic_, 10,
            std::bind(&EmergencyStopNode::emergencyCallback, this, std::placeholders::_1));
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        if (emergency_)
        {
            geometry_msgs::msg::Twist stop;
            // zero out velocities
            stop.linear.x = 0.0;
            stop.angular.z = 0.0;
            cmd_pub_->publish(stop);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    void emergencyCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        emergency_ = msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::string emergency_stop_topic_;
    std::string cmd_vel_topic_;
    bool emergency_ = false;
};
