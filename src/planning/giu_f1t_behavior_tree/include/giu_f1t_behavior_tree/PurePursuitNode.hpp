// PurePursuitNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>

class PurePursuitNode : public BT::SyncActionNode
{
public:
    PurePursuitNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "<YOUR_OPTIMIZED_PATH_TOPIC>", 10,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "<YOUR_CMD_VEL_TOPIC>", 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // TODO: run pure pursuit on latest_path_
        geometry_msgs::msg::Twist cmd;
        // fill cmd.linear/angular
        cmd_pub_->publish(cmd);
        return BT::NodeStatus::SUCCESS;
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        latest_path_ = *msg;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    nav_msgs::msg::Path latest_path_;
};
