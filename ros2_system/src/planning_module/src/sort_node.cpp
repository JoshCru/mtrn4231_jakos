/**
 * @file sort_node.cpp
 * @brief Sorting algorithm and decision making
 */

#include <rclcpp/rclcpp.hpp>
#include "sort_interfaces/msg/weight_estimate.hpp"
#include "sort_interfaces/msg/sort_decision.hpp"
#include "sort_interfaces/msg/target_area.hpp"
#include <vector>
#include <map>

class SortNode : public rclcpp::Node
{
public:
    SortNode() : Node("sort_node")
    {
        // Declare parameters
        this->declare_parameter("sorting_strategy", "weight_based");  // weight_based, random, sequential
        this->declare_parameter("weight_tolerance", 10.0);  // percentage

        // Get parameters
        sorting_strategy_ = this->get_parameter("sorting_strategy").as_string();
        weight_tolerance_ = this->get_parameter("weight_tolerance").as_double();

        // Subscribers
        weight_estimates_sub_ = this->create_subscription<sort_interfaces::msg::WeightEstimate>(
            "/recognition/estimated_weights", 10,
            std::bind(&SortNode::weight_estimate_callback, this, std::placeholders::_1));

        target_areas_sub_ = this->create_subscription<sort_interfaces::msg::TargetArea>(
            "/system/target_areas", 10,
            std::bind(&SortNode::target_area_callback, this, std::placeholders::_1));

        // Publisher
        sort_decisions_pub_ = this->create_publisher<sort_interfaces::msg::SortDecision>(
            "/planning/sort_decisions", 10);

        RCLCPP_INFO(this->get_logger(), "Sort Node initialized");
        RCLCPP_INFO(this->get_logger(), "Sorting strategy: %s", sorting_strategy_.c_str());
        RCLCPP_INFO(this->get_logger(), "Weight tolerance: %.1f%%", weight_tolerance_);
    }

private:
    void target_area_callback(const sort_interfaces::msg::TargetArea::SharedPtr msg)
    {
        target_areas_[msg->id] = *msg;
        RCLCPP_INFO(this->get_logger(),
                   "Registered target area %u: %s [%.1f - %.1f g]",
                   msg->id, msg->label.c_str(), msg->weight_min, msg->weight_max);
    }

    void weight_estimate_callback(const sort_interfaces::msg::WeightEstimate::SharedPtr msg)
    {
        if (target_areas_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No target areas defined yet");
            return;
        }

        // Make sorting decision
        auto decision = make_sorting_decision(*msg);

        if (decision) {
            sort_decisions_pub_->publish(*decision);

            RCLCPP_INFO(this->get_logger(),
                       "Sort Decision: Object %u (%.1f g) -> Target Area %u (%s)",
                       decision->object_id,
                       decision->estimated_weight,
                       decision->target_area_id,
                       decision->reason.c_str());
        }
    }

    std::shared_ptr<sort_interfaces::msg::SortDecision>
    make_sorting_decision(const sort_interfaces::msg::WeightEstimate& estimate)
    {
        auto decision = std::make_shared<sort_interfaces::msg::SortDecision>();
        decision->header = estimate.header;
        decision->object_id = estimate.object_id;
        decision->estimated_weight = estimate.estimated_weight;
        decision->pick_pose = estimate.pose;

        // Select target area based on strategy
        if (sorting_strategy_ == "weight_based") {
            select_by_weight(estimate, *decision);
        } else if (sorting_strategy_ == "sequential") {
            select_sequentially(*decision);
        } else if (sorting_strategy_ == "random") {
            select_randomly(*decision);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown sorting strategy: %s",
                       sorting_strategy_.c_str());
            select_by_weight(estimate, *decision);
        }

        return decision;
    }

    void select_by_weight(const sort_interfaces::msg::WeightEstimate& estimate,
                         sort_interfaces::msg::SortDecision& decision)
    {
        // Find target area that matches the weight range
        bool found = false;

        for (const auto& [id, area] : target_areas_) {
            if (estimate.estimated_weight >= area.weight_min &&
                estimate.estimated_weight <= area.weight_max) {
                decision.target_area_id = id;
                decision.place_pose = area.pose;
                decision.reason = "Weight " + std::to_string(estimate.estimated_weight) +
                                "g matches range [" + std::to_string(area.weight_min) +
                                " - " + std::to_string(area.weight_max) + "g]";
                found = true;
                break;
            }
        }

        if (!found) {
            // No exact match, find closest area
            RCLCPP_WARN(this->get_logger(),
                       "Weight %.1f g doesn't match any area exactly, using closest",
                       estimate.estimated_weight);
            select_closest_area(estimate, decision);
        }
    }

    void select_closest_area(const sort_interfaces::msg::WeightEstimate& estimate,
                            sort_interfaces::msg::SortDecision& decision)
    {
        if (target_areas_.empty()) {
            return;
        }

        double min_distance = std::numeric_limits<double>::max();
        uint8_t closest_id = 0;

        for (const auto& [id, area] : target_areas_) {
            double mid_weight = (area.weight_min + area.weight_max) / 2.0;
            double distance = std::abs(estimate.estimated_weight - mid_weight);

            if (distance < min_distance) {
                min_distance = distance;
                closest_id = id;
            }
        }

        const auto& area = target_areas_[closest_id];
        decision.target_area_id = closest_id;
        decision.place_pose = area.pose;
        decision.reason = "Closest match to weight " +
                         std::to_string(estimate.estimated_weight) + "g";
    }

    void select_sequentially(sort_interfaces::msg::SortDecision& decision)
    {
        // Round-robin selection
        static uint8_t next_area = 0;

        if (target_areas_.find(next_area) != target_areas_.end()) {
            const auto& area = target_areas_[next_area];
            decision.target_area_id = next_area;
            decision.place_pose = area.pose;
            decision.reason = "Sequential assignment";

            next_area = (next_area + 1) % target_areas_.size();
        }
    }

    void select_randomly(sort_interfaces::msg::SortDecision& decision)
    {
        if (target_areas_.empty()) {
            return;
        }

        // Random selection
        auto it = target_areas_.begin();
        std::advance(it, std::rand() % target_areas_.size());

        decision.target_area_id = it->first;
        decision.place_pose = it->second.pose;
        decision.reason = "Random assignment";
    }

    // Member variables
    rclcpp::Subscription<sort_interfaces::msg::WeightEstimate>::SharedPtr weight_estimates_sub_;
    rclcpp::Subscription<sort_interfaces::msg::TargetArea>::SharedPtr target_areas_sub_;
    rclcpp::Publisher<sort_interfaces::msg::SortDecision>::SharedPtr sort_decisions_pub_;

    std::map<uint8_t, sort_interfaces::msg::TargetArea> target_areas_;
    std::string sorting_strategy_;
    double weight_tolerance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SortNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
