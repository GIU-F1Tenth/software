// WatchDogNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class WatchDogNode : public BT::SyncActionNode
{
public:
    WatchDogNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        watchdog_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "<YOUR_WATCHDOG_TOPIC>", 10,
            std::bind(&WatchDogNode::watchdogCallback, this, std::placeholders::_1));
        watchdog_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "<YOUR_WATCHDOG_ALARM_TOPIC>", 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // TODO: decide if watchdog has timed out
        if (timed_out_)
        {
            std_msgs::msg::Bool alarm_msg;
            alarm_msg.data = true;
            watchdog_pub_->publish(alarm_msg);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    void watchdogCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // TODO: update internal timeout state
        timed_out_ = !msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr watchdog_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr watchdog_pub_;
    bool timed_out_ = false;
};
