#ifndef MY_CONTROLLER__MY_CONTROLLER_HPP_
#define MY_CONTROLLER__MY_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace my_controller
{

class MyController final : public controller_interface::ControllerInterface
{
public:
  MyController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  struct ActiveTrajectory
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    std::vector<double> start_positions;
    rclcpp::Time start_time;
    bool started{false};
    std::shared_ptr<GoalHandle> goal_handle;
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void handle_motion_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool configure_safety_parameters(std::string * reason);
  bool validate_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<double> & start_positions, std::string * reason) const;
  bool positions_within_limits(
    const std::vector<double> & positions, const std::string & context,
    std::string * reason) const;
  bool segment_delta_within_limit(
    const std::vector<double> & start, const std::vector<double> & finish,
    const std::string & context, std::string * reason) const;
  bool configure_envelope_parameters(std::string * reason);
  bool validate_point_derivatives(
    const trajectory_msgs::msg::JointTrajectoryPoint & point, const std::string & context,
    std::string * reason) const;
  bool validate_speed_envelope(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<double> & start_positions, std::string * reason) const;
  std::vector<double> sample_trajectory(
    const ActiveTrajectory & active_trajectory, const rclcpp::Duration & elapsed) const;
  std::vector<double> read_current_positions() const;
  std::shared_ptr<FollowJointTrajectory::Feedback> make_feedback(
    const rclcpp::Time & time, const rclcpp::Duration & elapsed,
    const std::vector<double> & desired) const;
  static FollowJointTrajectory::Result::SharedPtr make_result(
    int32_t error_code, const std::string & message);

  std::vector<std::string> position_interface_names() const;
  void write_position_commands(const std::vector<double> & positions);

  std::vector<std::string> joints_;
  std::vector<double> hold_positions_;
  std::vector<double> min_positions_;
  std::vector<double> max_positions_;
  double start_tolerance_{0.2};
  double max_segment_delta_{1.0};
  double envelope_tolerance_{1.0e-6};
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;
  bool safety_limits_configured_{false};
  bool motion_enabled_{false};
  std::unique_ptr<ActiveTrajectory> active_trajectory_;
  typename rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_enabled_service_;
  mutable std::mutex trajectory_mutex_;
};

}  // namespace my_controller

#endif  // MY_CONTROLLER__MY_CONTROLLER_HPP_
