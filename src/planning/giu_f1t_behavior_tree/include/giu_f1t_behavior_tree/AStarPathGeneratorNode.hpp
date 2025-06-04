// AStarPathGeneratorNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

// TODO
class AStarPathGeneratorNode : public BT::SyncActionNode
{
public:
    AStarPathGeneratorNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "<YOUR_PLANNED_PATH_TOPIC>", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<std::string>>("objects")};
    }

    BT::NodeStatus tick() override
    {
        auto objects = getInput<std::vector<std::string>>("objects");
        // TODO: compute nav_msgs::msg::Path using A*
        nav_msgs::msg::Path path_msg;
        // fill path_msg.poses...
        path_pub_->publish(path_msg);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};
