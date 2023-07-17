#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ap_custom_services/action/gm.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace at_client
{
class GMActionClient : public rclcpp::Node
{
public:
  using GM = ap_custom_services::action::GM;
  using GoalHandleGM = rclcpp_action::ClientGoalHandle<GM>;

  explicit GMActionClient(const rclcpp::NodeOptions & options)
  : Node("gm_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<GM>(
      this,
      "gm");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GMActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = GM::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GM>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&GMActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&GMActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&GMActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GM>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleGM::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleGM::SharedPtr,
    const std::shared_ptr<const GM::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    
    //for (auto number : feedback->result) {
      //ss << number << " ";
    //}
    auto number = feedback->current;
    ss << number << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleGM::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    //for (auto number : result.result->sequence) {
      //ss << number << " ";
    //}
    auto number = result.result->result;
    ss << number << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class GMActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(at_client::GMActionClient)

