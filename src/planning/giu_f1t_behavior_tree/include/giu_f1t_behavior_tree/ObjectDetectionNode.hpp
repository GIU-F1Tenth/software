// ObjectDetectionNode.hpp
#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <your_msgs/msg/detections.hpp> // replace with your actual message

// TODO
class ObjectDetectionNode : public BT::SyncActionNode
{
public:
    ObjectDetectionNode(const std::string &name, const BT::NodeConfiguration &config)
        : SyncActionNode(name, config)
    {
        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        detect_sub_ = node_->create_subscription<your_msgs::msg::Detections>(
            "<YOUR_DETECTIONS_TOPIC>", 10,
            std::bind(&ObjectDetectionNode::detectionCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::vector<std::string>>("objects")};
    }

    BT::NodeStatus tick() override
    {
        // TODO: convert latest_detections_ into vector<string>
        setOutput("objects", latest_objects_);
        return BT::NodeStatus::SUCCESS;
    }

private:
    void detectionCallback(const your_msgs::msg::Detections::SharedPtr msg)
    {
        latest_objects_.clear();
        for (const auto &det : msg->objects)
        {
            latest_objects_.push_back(det.label);
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<your_msgs::msg::Detections>::SharedPtr detect_sub_;
    std::vector<std::string> latest_objects_;
};
