// GapFollowerNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

// TODO
class GapFollowerNode : public BT::SyncActionNode
{
public:
    GapFollowerNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_->declare_parameter<std::string>("gap_follower_topic", "/gap_follower");
        node_->declare_parameter<std::string>("cmd_vel_topic", "/ackermann_cmd");

        node_->get_parameter("gap_follower_topic", gap_follower_topic_);
        node_->get_parameter("cmd_vel_topic", cmd_vel_topic_);

        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        // scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "<YOUR_LASER_SCAN_TOPIC>", 10,
        //     std::bind(&GapFollowerNode::scanCallback, this, std::placeholders::_1));
        gap_follower_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            gap_follower_topic_, 10, gap_follower_callback);
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // TODO: implement gap-follower logic using latest_scan_
        geometry_msgs::msg::Twist cmd;
        cmd_pub_->publish(cmd);
        return BT::NodeStatus::SUCCESS;
    }

private:
    //     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    //     {
    //         latest_scan_ = *msg;
    //     }

    //     rclcpp::Node::SharedPtr node_;
    //     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    void gapFollowerCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::string gap_follower_topic_;
    std : string cmd_vel_topic_;
    // sensor_msgs::msg::LaserScan latest_scan_;
};
