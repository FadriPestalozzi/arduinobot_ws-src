#include <arduinobot_msgs/action/fibonacci.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <thread>

using namespace std::placeholders;

namespace arduinobot_cpp_examples {
class SimpleActionServer : public rclcpp::Node {
public:
  explicit SimpleActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("simple_action_server", options) {
    action_server_ =
        rclcpp_action::create_server<arduinobot_msgs::action::Fibonacci>(
            this, "fibonacci", // declaring callback fns
            std::bind(&SimpleActionServer::goalCallback, this, _1, _2),
            // 2nd callback (bind) function only getting one input _1
            std::bind(&SimpleActionServer::cancelCallback, this, _1),
            std::bind(&SimpleActionServer::acceptedCallback, this, _1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Simple Action Server has been started.");
  }

private:
  rclcpp_action::Server<arduinobot_msgs::action::Fibonacci>::SharedPtr
      action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const arduinobot_msgs::action::Fibonacci::Goal> goal) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Received goal request with order: " << goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void acceptedCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>>
          goal_handle) {
    // calc in other thread to avoid blocking client
    std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void
  execute(const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>>
              goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto feedback =
        std::make_shared<arduinobot_msgs::action::Fibonacci::Feedback>();

    // reference variable
    auto &sequence = feedback->partial_sequence;

    sequence.push_back(0);
    sequence.push_back(1);

    auto result =
        std::make_shared<arduinobot_msgs::action::Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); i++) {
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Canceled");
        return;
      }
      // calc if no cancellation request received
      sequence.push_back(sequence[i] + sequence[i - 1]);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish Feedback");
      loop_rate.sleep(); // match frequency
    };

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Succeeded");
    };

  } // end execute

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::Fibonacci>>
          goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
};
} // namespace arduinobot_cpp_examples

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionServer)
