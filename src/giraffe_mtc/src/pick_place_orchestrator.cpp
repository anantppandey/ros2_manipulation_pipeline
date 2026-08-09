#include <memory>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "giraffe_mtc/action/pick_place.hpp"

using namespace std::placeholders;

// ==========================================
// PICK-PLACE ORCHESTRATOR (step 2 version)
//
// This is what answers "how does it know which cube to pick": a
// target_object parameter, checked against the class_id of every entry
// in each /object_detections message. Whichever entry matches becomes
// pick_pose; the entry labeled "collection_box" becomes place_pose - so
// the old place_x/y/z parameters are gone, replaced by the box's real
// detected position.
//
// Change target_object at launch:
//   ros2 run giraffe_mtc pick_place_orchestrator --ros-args -p target_object:=blue_cube
// or live, while it's running:
//   ros2 param set /pick_place_orchestrator target_object yellow_cube
//
// Still scoped narrowly on purpose: no queueing of multiple targets, no
// re-picking after a successful place, no retry-on-abort. It sends one
// goal, waits for the result, and is ready to send the next one the
// moment the target (and the box) are both visible again.
// ==========================================
class PickPlaceOrchestrator : public rclcpp::Node
{
public:
    using PickPlace = giraffe_mtc::action::PickPlace;
    using GoalHandlePickPlace = rclcpp_action::ClientGoalHandle<PickPlace>;

    PickPlaceOrchestrator() : Node("pick_place_orchestrator")
    {
        this->declare_parameter<std::string>("target_object", "red_cube");

        // Reject anything that isn't a pickable object up front, whether it
        // comes from the launch file or a runtime `ros2 param set`.
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PickPlaceOrchestrator::validateParameters, this, std::placeholders::_1));

        action_client_ = rclcpp_action::create_client<PickPlace>(this, "pick_place");

        detections_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/object_detections", 10,
            std::bind(&PickPlaceOrchestrator::detectionsCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Pick-place orchestrator started, target_object='%s'",
                    this->get_parameter("target_object").as_string().c_str());
    }

private:
    rcl_interfaces::msg::SetParametersResult validateParameters(
        const std::vector<rclcpp::Parameter>& params)
    {
        static const std::vector<std::string> valid_targets = {"red_cube", "blue_cube", "yellow_cube"};

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& p : params) {
            if (p.get_name() == "target_object") {
                const auto& v = p.as_string();
                if (std::find(valid_targets.begin(), valid_targets.end(), v) == valid_targets.end()) {
                    result.successful = false;
                    result.reason = "target_object must be one of: red_cube, blue_cube, yellow_cube";
                }
            }
        }
        return result;
    }

    void detectionsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        if (goal_in_flight_.load()) {
            return;  // a pick-place is already running - ignore new detections until it's done
        }
        if (target_placed_.load()) {
            return;  // already placed successfully - nothing left to do
        }

        std::string target = this->get_parameter("target_object").as_string();

        const vision_msgs::msg::Detection3D* pick_det = nullptr;
        const vision_msgs::msg::Detection3D* box_det = nullptr;

        for (const auto& det : msg->detections) {
            if (det.results.empty()) continue;
            const std::string& class_id = det.results[0].hypothesis.class_id;
            if (class_id == target) pick_det = &det;
            if (class_id == "collection_box") box_det = &det;
        }

        if (!pick_det) {
            return;  // target not visible this frame - just wait for the next one
        }
        if (!box_det) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Target '%s' detected but collection_box is not visible - waiting",
                                  target.c_str());
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "pick_place action server not available");
            return;
        }

        auto goal_msg = PickPlace::Goal();
        goal_msg.object_id = target;
        goal_msg.pick_pose = pick_det->results[0].pose.pose;
        goal_msg.place_pose = box_det->results[0].pose.pose;

        auto send_goal_options = rclcpp_action::Client<PickPlace>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&PickPlaceOrchestrator::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&PickPlaceOrchestrator::resultCallback, this, _1);

        goal_in_flight_.store(true);
        RCLCPP_INFO(this->get_logger(),
                    "Sending pick-place goal - target '%s' at (%.3f, %.3f, %.3f) -> box at (%.3f, %.3f, %.3f)",
                    target.c_str(),
                    goal_msg.pick_pose.position.x, goal_msg.pick_pose.position.y, goal_msg.pick_pose.position.z,
                    goal_msg.place_pose.position.x, goal_msg.place_pose.position.y, goal_msg.place_pose.position.z);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedbackCallback(GoalHandlePickPlace::SharedPtr,
                           const std::shared_ptr<const PickPlace::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Stage: %s", feedback->current_stage.c_str());
    }

    void resultCallback(const GoalHandlePickPlace::WrappedResult& result)
    {
        goal_in_flight_.store(false);

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Pick-place succeeded: %s", result.result->message.c_str());
                target_placed_.store(true);  // stop sending new goals
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Pick-place aborted: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Pick-place canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Pick-place ended with an unknown result code");
                break;
        }
    }

    rclcpp_action::Client<PickPlace>::SharedPtr action_client_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::atomic<bool> goal_in_flight_{false};
    std::atomic<bool> target_placed_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceOrchestrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}