// OptimizedPathNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

class OptimizedPathNode : public BT::SyncActionNode
{
public:
    OptimizedPathNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "<YOUR_PLANNED_PATH_TOPIC>", 10,
            std::bind(&OptimizedPathNode::pathCallback, this, std::placeholders::_1));
        opt_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "<YOUR_OPTIMIZED_PATH_TOPIC>", 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // TODO: optimize latest_path_
        nav_msgs::msg::Path opt_path = latest_path_;
        // e.g., smooth or shorten...
        opt_pub_->publish(opt_path);
        return BT::NodeStatus::SUCCESS;
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        latest_path_ = *msg;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_pub_;
    nav_msgs::msg::Path latest_path_;
};
