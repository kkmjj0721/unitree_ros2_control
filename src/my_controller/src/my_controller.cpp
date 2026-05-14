#include "my_controller/my_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_controller
{
namespace
{
constexpr char kActionName[] = "~/follow_joint_trajectory";
constexpr char kMotionEnabledService[] = "~/follow_joint_trajectory/set_motion_enabled";
constexpr double kDefaultStartTolerance = 0.2;
constexpr double kDefaultMaxSegmentDelta = 1.0;
constexpr double kDefaultMaxVelocity = 1.2;
constexpr double kDefaultMaxAcceleration = 1.5;
constexpr double kDefaultEnvelopeTolerance = 1.0e-6;
constexpr double kDefaultMinJoint1 = -3.14;
constexpr double kDefaultMaxJoint1 = 3.14;
constexpr double kDefaultMinJoint2 = -2.0;
constexpr double kDefaultMaxJoint2 = 2.0;
constexpr double kDefaultMinJoint3 = -2.0;
constexpr double kDefaultMaxJoint3 = 2.0;
constexpr unsigned int kNanosecondsPerSecond = 1000000000U;

rclcpp::Duration point_time(const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return rclcpp::Duration(point.time_from_start);
}

bool is_zero_time(const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return (point.time_from_start.sec == 0) && (point.time_from_start.nanosec == 0U);
}

bool is_default_three_joint_arm(const std::vector<std::string> & joints)
{
  return (joints.size() == 3U) && (joints[0] == "joint1") && (joints[1] == "joint2") &&
         (joints[2] == "joint3");
}
}  // namespace

controller_interface::InterfaceConfiguration MyController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    position_interface_names()};
}

controller_interface::InterfaceConfiguration MyController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    position_interface_names()};
}

controller_interface::return_type MyController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)period;

  std::vector<double> command_positions;
  std::shared_ptr<GoalHandle> terminal_goal;
  FollowJointTrajectory::Result::SharedPtr terminal_result;
  bool terminal_canceled = false;
  bool terminal_aborted = false;
  std::shared_ptr<GoalHandle> feedback_goal;
  std::shared_ptr<FollowJointTrajectory::Feedback> feedback;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_)
    {
      if (!motion_enabled_)
      {
        terminal_goal = active_trajectory_->goal_handle;
        terminal_result = make_result(
          FollowJointTrajectory::Result::INVALID_GOAL, "Motion disabled");
        terminal_aborted = true;
        command_positions = hold_positions_;
        active_trajectory_.reset();
      }
      else if (active_trajectory_->goal_handle->is_canceling())
      {
        terminal_goal = active_trajectory_->goal_handle;
        terminal_result = make_result(
          FollowJointTrajectory::Result::SUCCESSFUL, "Trajectory canceled");
        terminal_canceled = true;
        command_positions = hold_positions_;
        active_trajectory_.reset();
      }
      else
      {
        if (!active_trajectory_->started)
        {
          active_trajectory_->start_time = time;
          active_trajectory_->started = true;
        }

        rclcpp::Duration elapsed = time - active_trajectory_->start_time;
        const rclcpp::Duration zero(0, 0);
        if (elapsed < zero)
        {
          elapsed = zero;
        }

        command_positions = sample_trajectory(*active_trajectory_, elapsed);
        if (command_positions.size() == joints_.size())
        {
          hold_positions_ = command_positions;
          feedback_goal = active_trajectory_->goal_handle;
          feedback = make_feedback(time, elapsed, command_positions);
        }

        const auto & points = active_trajectory_->trajectory.points;
        if (!points.empty() && elapsed >= point_time(points.back()))
        {
          terminal_goal = active_trajectory_->goal_handle;
          terminal_result = make_result(
            FollowJointTrajectory::Result::SUCCESSFUL, "Trajectory execution complete");
          active_trajectory_.reset();
        }
      }
    }
    else
    {
      command_positions = hold_positions_;
    }
  }

  write_position_commands(command_positions);

  if (feedback_goal && feedback_goal->is_executing() && feedback)
  {
    feedback_goal->publish_feedback(feedback);
  }

  if (terminal_goal && terminal_goal->is_active() && terminal_result)
  {
    if (terminal_canceled)
    {
      terminal_goal->canceled(terminal_result);
    }
    else if (terminal_aborted)
    {
      terminal_goal->abort(terminal_result);
    }
    else
    {
      terminal_goal->succeed(terminal_result);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MyController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<double>>("min_positions", std::vector<double>{});
    auto_declare<std::vector<double>>("max_positions", std::vector<double>{});
    auto_declare<double>("start_tolerance", kDefaultStartTolerance);
    auto_declare<double>("max_segment_delta", kDefaultMaxSegmentDelta);
    auto_declare<std::vector<double>>("max_velocities", std::vector<double>{});
    auto_declare<std::vector<double>>("max_accelerations", std::vector<double>{});
    auto_declare<double>("envelope_tolerance", kDefaultEnvelopeTolerance);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must contain at least one joint");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::set<std::string> unique_joints;
  for (const auto & joint : joints_)
  {
    if (joint.empty() || !unique_joints.insert(joint).second)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' contains an empty or duplicate name");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  std::string safety_reason;
  if (!configure_safety_parameters(&safety_reason))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid safety parameters: %s", safety_reason.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string envelope_reason;
  if (!configure_envelope_parameters(&envelope_reason))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid speed envelope parameters: %s",
      envelope_reason.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_positions_.assign(joints_.size(), 0.0);
  motion_enabled_ = false;

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node(), kActionName,
    std::bind(&MyController::handle_goal, this, _1, _2),
    std::bind(&MyController::handle_cancel, this, _1),
    std::bind(&MyController::handle_accepted, this, _1));
  motion_enabled_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    kMotionEnabledService,
    std::bind(&MyController::handle_motion_enabled, this, _1, _2));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured FollowJointTrajectory action on %s; motion gate service on %s",
    kActionName, kMotionEnabledService);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  if (command_interfaces_.size() != joints_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu",
      joints_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!safety_limits_configured_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Safety limits are not configured");
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_positions_ = read_current_positions();
  std::string reason;
  if (!positions_within_limits(hold_positions_, "current position", &reason))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Refusing activation: %s", reason.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  write_position_commands(hold_positions_);
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    motion_enabled_ = false;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Motion gate is disabled on activation");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  std::shared_ptr<GoalHandle> goal_to_abort;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_)
    {
      goal_to_abort = active_trajectory_->goal_handle;
      active_trajectory_.reset();
    }
    motion_enabled_ = false;
  }

  if (goal_to_abort && goal_to_abort->is_active())
  {
    goal_to_abort->abort(make_result(
      FollowJointTrajectory::Result::INVALID_GOAL, "Controller deactivated"));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse MyController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;

  if (get_state().label() != "active")
  {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting trajectory goal: controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::vector<double> current_positions;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!motion_enabled_)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Rejecting trajectory goal: motion gate is disabled");
      return rclcpp_action::GoalResponse::REJECT;
    }
    current_positions = read_current_positions();
  }

  std::string reason;
  if (!validate_trajectory(goal->trajectory, current_positions, &reason))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting trajectory goal: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MyController::handle_cancel(std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (active_trajectory_ && active_trajectory_->goal_handle == goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void MyController::handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
  std::shared_ptr<GoalHandle> goal_to_abort;
  bool accepted = false;
  std::string rejection_reason;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const std::vector<double> start_positions = read_current_positions();
    if (!motion_enabled_)
    {
      rejection_reason = "Motion disabled";
      if (!active_trajectory_)
      {
        hold_positions_ = start_positions;
      }
    }
    else if (!validate_trajectory(
        goal_handle->get_goal()->trajectory, start_positions, &rejection_reason))
    {
      if (!active_trajectory_)
      {
        hold_positions_ = start_positions;
      }
    }
    else
    {
      if (active_trajectory_)
      {
        goal_to_abort = active_trajectory_->goal_handle;
      }

      auto next_trajectory = std::make_unique<ActiveTrajectory>();
      next_trajectory->trajectory = goal_handle->get_goal()->trajectory;
      next_trajectory->start_positions = start_positions;
      next_trajectory->goal_handle = std::move(goal_handle);
      active_trajectory_ = std::move(next_trajectory);
      accepted = true;
    }
  }

  if (!accepted)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Aborting accepted trajectory before execution: %s",
      rejection_reason.c_str());
    if (goal_handle->is_active())
    {
      goal_handle->abort(make_result(
        FollowJointTrajectory::Result::INVALID_GOAL, rejection_reason));
    }
    return;
  }

  if (goal_to_abort && goal_to_abort->is_active())
  {
    goal_to_abort->abort(make_result(
      FollowJointTrajectory::Result::INVALID_GOAL, "Preempted by a newer trajectory"));
  }
}

void MyController::handle_motion_enabled(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if ((request == nullptr) || (response == nullptr))
  {
    return;
  }

  std::shared_ptr<GoalHandle> goal_to_abort;
  const bool enable_motion = request->data;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    motion_enabled_ = enable_motion;

    if (!enable_motion)
    {
      if (hold_positions_.size() != joints_.size())
      {
        hold_positions_ = read_current_positions();
      }

      if (active_trajectory_)
      {
        goal_to_abort = active_trajectory_->goal_handle;
        active_trajectory_.reset();
      }
    }
  }

  if (enable_motion)
  {
    response->success = true;
    response->message = "motion enabled";
    RCLCPP_INFO(get_node()->get_logger(), "Motion gate enabled");
    return;
  }

  if (goal_to_abort && goal_to_abort->is_active())
  {
    goal_to_abort->abort(make_result(
      FollowJointTrajectory::Result::INVALID_GOAL, "Motion disabled"));
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Motion gate disabled while executing; active trajectory aborted and holding last command");
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Motion gate disabled; holding last command");
  }

  response->success = true;
  response->message = "motion disabled";
}

bool MyController::configure_safety_parameters(std::string * reason)
{
  const auto configured_min = get_node()->get_parameter("min_positions").as_double_array();
  const auto configured_max = get_node()->get_parameter("max_positions").as_double_array();

  start_tolerance_ = get_node()->get_parameter("start_tolerance").as_double();
  max_segment_delta_ = get_node()->get_parameter("max_segment_delta").as_double();
  safety_limits_configured_ = false;

  if (!std::isfinite(start_tolerance_) || (start_tolerance_ < 0.0))
  {
    if (reason != nullptr)
    {
      *reason = "start_tolerance must be finite and non-negative";
    }
    return false;
  }

  if (!std::isfinite(max_segment_delta_) || (max_segment_delta_ <= 0.0))
  {
    if (reason != nullptr)
    {
      *reason = "max_segment_delta must be finite and positive";
    }
    return false;
  }

  const bool has_min = !configured_min.empty();
  const bool has_max = !configured_max.empty();
  if (has_min != has_max)
  {
    if (reason != nullptr)
    {
      *reason = "min_positions and max_positions must both be provided or both be absent";
    }
    return false;
  }

  if (has_min)
  {
    min_positions_ = configured_min;
    max_positions_ = configured_max;
  }
  else if (is_default_three_joint_arm(joints_))
  {
    min_positions_ = {kDefaultMinJoint1, kDefaultMinJoint2, kDefaultMinJoint3};
    max_positions_ = {kDefaultMaxJoint1, kDefaultMaxJoint2, kDefaultMaxJoint3};
  }
  else
  {
    if (reason != nullptr)
    {
      *reason = "min_positions and max_positions are required for non-default joint layouts";
    }
    return false;
  }

  if ((min_positions_.size() != joints_.size()) || (max_positions_.size() != joints_.size()))
  {
    if (reason != nullptr)
    {
      *reason = "min_positions and max_positions must match the number of configured joints";
    }
    return false;
  }

  for (std::size_t index = 0U; index < joints_.size(); ++index)
  {
    if (!std::isfinite(min_positions_[index]) || !std::isfinite(max_positions_[index]) ||
      (min_positions_[index] >= max_positions_[index]))
    {
      if (reason != nullptr)
      {
        *reason = "each min_positions entry must be finite and less than max_positions";
      }
      return false;
    }
  }

  safety_limits_configured_ = true;
  return true;
}

bool MyController::configure_envelope_parameters(std::string * reason)
{
  const auto configured_velocities = get_node()->get_parameter("max_velocities").as_double_array();
  const auto configured_accelerations =
    get_node()->get_parameter("max_accelerations").as_double_array();

  envelope_tolerance_ = get_node()->get_parameter("envelope_tolerance").as_double();
  if (!std::isfinite(envelope_tolerance_) || (envelope_tolerance_ < 0.0))
  {
    if (reason != nullptr)
    {
      *reason = "envelope_tolerance must be finite and non-negative";
    }
    return false;
  }

  if (configured_velocities.empty())
  {
    max_velocities_.assign(joints_.size(), kDefaultMaxVelocity);
  }
  else
  {
    max_velocities_ = configured_velocities;
  }

  if (configured_accelerations.empty())
  {
    max_accelerations_.assign(joints_.size(), kDefaultMaxAcceleration);
  }
  else
  {
    max_accelerations_ = configured_accelerations;
  }

  if ((max_velocities_.size() != joints_.size()) ||
    (max_accelerations_.size() != joints_.size()))
  {
    if (reason != nullptr)
    {
      *reason = "max_velocities and max_accelerations must match configured joints";
    }
    return false;
  }

  for (std::size_t index = 0U; index < joints_.size(); ++index)
  {
    if (!std::isfinite(max_velocities_[index]) || (max_velocities_[index] <= 0.0))
    {
      if (reason != nullptr)
      {
        *reason = "each max_velocities entry must be finite and positive";
      }
      return false;
    }

    if (!std::isfinite(max_accelerations_[index]) || (max_accelerations_[index] <= 0.0))
    {
      if (reason != nullptr)
      {
        *reason = "each max_accelerations entry must be finite and positive";
      }
      return false;
    }
  }

  return true;
}

bool MyController::validate_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & start_positions, std::string * reason) const
{
  if (trajectory.joint_names != joints_)
  {
    if (reason != nullptr)
    {
      *reason = "joint_names must exactly match configured joints";
    }
    return false;
  }

  if (trajectory.points.empty())
  {
    if (reason != nullptr)
    {
      *reason = "trajectory has no points";
    }
    return false;
  }

  if (!positions_within_limits(start_positions, "start position", reason))
  {
    return false;
  }

  if (start_positions.size() != joints_.size())
  {
    if (reason != nullptr)
    {
      *reason = "start position vector size does not match configured joints";
    }
    return false;
  }

  rclcpp::Duration previous_time(0, 0);
  bool first_point = true;
  std::vector<double> previous_positions;
  for (const auto & point : trajectory.points)
  {
    if (point.positions.size() != joints_.size())
    {
      if (reason != nullptr)
      {
        *reason = "each point must contain one position per configured joint";
      }
      return false;
    }

    if (!positions_within_limits(point.positions, "trajectory point", reason))
    {
      return false;
    }

    if (!validate_point_derivatives(point, "trajectory point", reason))
    {
      return false;
    }

    if ((point.time_from_start.sec < 0) ||
      (point.time_from_start.nanosec >= kNanosecondsPerSecond))
    {
      if (reason != nullptr)
      {
        *reason = "time_from_start must be non-negative and normalized";
      }
      return false;
    }

    const rclcpp::Duration current_time = point_time(point);
    if (!first_point && current_time <= previous_time)
    {
      if (reason != nullptr)
      {
        *reason = "point time_from_start values must be strictly increasing";
      }
      return false;
    }

    if (first_point)
    {
      if (is_zero_time(point))
      {
        for (std::size_t index = 0U; index < joints_.size(); ++index)
        {
          if (std::abs(point.positions[index] - start_positions[index]) > start_tolerance_)
          {
            if (reason != nullptr)
            {
              *reason =
                "first point at time zero differs from current position beyond start_tolerance";
            }
            return false;
          }
        }
      }

      if (!segment_delta_within_limit(
          start_positions, point.positions, "start-to-first segment", reason))
      {
        return false;
      }
    }
    else if (!segment_delta_within_limit(
        previous_positions, point.positions, "trajectory segment", reason))
    {
      return false;
    }

    previous_time = current_time;
    previous_positions = point.positions;
    first_point = false;
  }

  return validate_speed_envelope(trajectory, start_positions, reason);
}

bool MyController::positions_within_limits(
  const std::vector<double> & positions, const std::string & context, std::string * reason) const
{
  if (positions.size() != joints_.size())
  {
    if (reason != nullptr)
    {
      *reason = context + " size does not match configured joints";
    }
    return false;
  }

  if (!safety_limits_configured_)
  {
    if (reason != nullptr)
    {
      *reason = "safety limits are not configured";
    }
    return false;
  }

  for (std::size_t index = 0U; index < joints_.size(); ++index)
  {
    const double position = positions[index];
    if (!std::isfinite(position))
    {
      if (reason != nullptr)
      {
        *reason = context + " positions must be finite";
      }
      return false;
    }

    if ((position < min_positions_[index]) || (position > max_positions_[index]))
    {
      if (reason != nullptr)
      {
        *reason = context + " position exceeds configured joint limits";
      }
      return false;
    }
  }

  return true;
}

bool MyController::segment_delta_within_limit(
  const std::vector<double> & start, const std::vector<double> & finish,
  const std::string & context, std::string * reason) const
{
  if ((start.size() != joints_.size()) || (finish.size() != joints_.size()))
  {
    if (reason != nullptr)
    {
      *reason = context + " size does not match configured joints";
    }
    return false;
  }

  for (std::size_t index = 0U; index < joints_.size(); ++index)
  {
    if (std::abs(finish[index] - start[index]) > max_segment_delta_)
    {
      if (reason != nullptr)
      {
        *reason = context + " delta exceeds max_segment_delta";
      }
      return false;
    }
  }

  return true;
}

bool MyController::validate_point_derivatives(
  const trajectory_msgs::msg::JointTrajectoryPoint & point, const std::string & context,
  std::string * reason) const
{
  if (!point.velocities.empty())
  {
    if (point.velocities.size() != joints_.size())
    {
      if (reason != nullptr)
      {
        *reason = context + " velocities size does not match configured joints";
      }
      return false;
    }

    for (std::size_t index = 0U; index < joints_.size(); ++index)
    {
      const double velocity = point.velocities[index];
      if (!std::isfinite(velocity))
      {
        if (reason != nullptr)
        {
          *reason = context + " velocities must be finite";
        }
        return false;
      }

      if (std::abs(velocity) > (max_velocities_[index] + envelope_tolerance_))
      {
        if (reason != nullptr)
        {
          *reason = context + " velocity exceeds max_velocities for " + joints_[index];
        }
        return false;
      }
    }
  }

  if (!point.accelerations.empty())
  {
    if (point.accelerations.size() != joints_.size())
    {
      if (reason != nullptr)
      {
        *reason = context + " accelerations size does not match configured joints";
      }
      return false;
    }

    for (std::size_t index = 0U; index < joints_.size(); ++index)
    {
      const double acceleration = point.accelerations[index];
      if (!std::isfinite(acceleration))
      {
        if (reason != nullptr)
        {
          *reason = context + " accelerations must be finite";
        }
        return false;
      }

      if (std::abs(acceleration) > (max_accelerations_[index] + envelope_tolerance_))
      {
        if (reason != nullptr)
        {
          *reason = context + " acceleration exceeds max_accelerations for " + joints_[index];
        }
        return false;
      }
    }
  }

  return true;
}

bool MyController::validate_speed_envelope(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & start_positions, std::string * reason) const
{
  std::vector<double> previous_positions = start_positions;
  rclcpp::Duration previous_time(0, 0);
  std::vector<double> previous_segment_velocities;
  double previous_segment_duration = 0.0;
  bool has_positive_duration_segment = false;

  for (std::size_t point_index = 0U; point_index < trajectory.points.size(); ++point_index)
  {
    const auto & point = trajectory.points[point_index];
    const rclcpp::Duration current_time = point_time(point);

    if ((point_index == 0U) && is_zero_time(point))
    {
      previous_positions = point.positions;
      previous_time = current_time;
      continue;
    }

    const double segment_duration = (current_time - previous_time).seconds();
    if (!std::isfinite(segment_duration) || (segment_duration <= 0.0))
    {
      if (reason != nullptr)
      {
        *reason = "trajectory contains a non-positive segment duration";
      }
      return false;
    }

    std::vector<double> segment_velocities(joints_.size(), 0.0);
    for (std::size_t joint_index = 0U; joint_index < joints_.size(); ++joint_index)
    {
      const double velocity =
        (point.positions[joint_index] - previous_positions[joint_index]) / segment_duration;
      if (!std::isfinite(velocity))
      {
        if (reason != nullptr)
        {
          *reason = "trajectory segment velocity must be finite";
        }
        return false;
      }

      if (std::abs(velocity) > (max_velocities_[joint_index] + envelope_tolerance_))
      {
        if (reason != nullptr)
        {
          *reason = "trajectory segment velocity exceeds max_velocities for " +
            joints_[joint_index];
        }
        return false;
      }

      segment_velocities[joint_index] = velocity;
    }

    for (std::size_t joint_index = 0U; joint_index < joints_.size(); ++joint_index)
    {
      double acceleration = segment_velocities[joint_index] / segment_duration;
      if (!previous_segment_velocities.empty())
      {
        const double acceleration_window =
          (previous_segment_duration + segment_duration) * 0.5;
        acceleration =
          (segment_velocities[joint_index] - previous_segment_velocities[joint_index]) /
          acceleration_window;
      }

      if (!std::isfinite(acceleration))
      {
        if (reason != nullptr)
        {
          *reason = "trajectory segment acceleration must be finite";
        }
        return false;
      }

      if (std::abs(acceleration) > (max_accelerations_[joint_index] + envelope_tolerance_))
      {
        if (reason != nullptr)
        {
          *reason = "trajectory segment acceleration exceeds max_accelerations for " +
            joints_[joint_index];
        }
        return false;
      }
    }

    previous_segment_velocities = segment_velocities;
    previous_segment_duration = segment_duration;
    previous_positions = point.positions;
    previous_time = current_time;
    has_positive_duration_segment = true;
  }

  if (!has_positive_duration_segment)
  {
    if (reason != nullptr)
    {
      *reason = "trajectory must contain at least one positive-duration segment";
    }
    return false;
  }

  for (std::size_t joint_index = 0U; joint_index < joints_.size(); ++joint_index)
  {
    const double acceleration =
      previous_segment_velocities[joint_index] / previous_segment_duration;
    if (!std::isfinite(acceleration))
    {
      if (reason != nullptr)
      {
        *reason = "trajectory terminal acceleration must be finite";
      }
      return false;
    }

    if (std::abs(acceleration) > (max_accelerations_[joint_index] + envelope_tolerance_))
    {
      if (reason != nullptr)
      {
        *reason = "trajectory terminal acceleration exceeds max_accelerations for " +
          joints_[joint_index];
      }
      return false;
    }
  }

  return true;
}

std::vector<double> MyController::sample_trajectory(
  const ActiveTrajectory & active_trajectory, const rclcpp::Duration & elapsed) const
{
  std::vector<double> positions;
  const auto & trajectory = active_trajectory.trajectory;
  const auto & start_positions = active_trajectory.start_positions;
  if (trajectory.points.empty())
  {
    return positions;
  }

  const auto & points = trajectory.points;
  const rclcpp::Duration first_time = point_time(points.front());
  if (elapsed <= first_time)
  {
    if (start_positions.size() != joints_.size())
    {
      return positions;
    }

    double ratio = 1.0;
    const double first_segment_duration = first_time.seconds();
    if (first_segment_duration > 0.0)
    {
      ratio = std::clamp(elapsed.seconds() / first_segment_duration, 0.0, 1.0);
    }

    positions.resize(joints_.size(), 0.0);
    for (std::size_t joint_index = 0U; joint_index < joints_.size(); ++joint_index)
    {
      const double start = start_positions[joint_index];
      const double finish = points.front().positions[joint_index];
      positions[joint_index] = start + ((finish - start) * ratio);
    }
    return positions;
  }

  if (points.size() == 1U)
  {
    return points.front().positions;
  }

  for (std::size_t index = 1U; index < points.size(); ++index)
  {
    const rclcpp::Duration next_time = point_time(points[index]);
    if (elapsed <= next_time)
    {
      const rclcpp::Duration previous_time = point_time(points[index - 1U]);
      const double segment_duration = (next_time - previous_time).seconds();
      const double segment_elapsed = (elapsed - previous_time).seconds();
      double ratio = 1.0;
      if (segment_duration > 0.0)
      {
        ratio = std::clamp(segment_elapsed / segment_duration, 0.0, 1.0);
      }

      positions.resize(joints_.size(), 0.0);
      for (std::size_t joint_index = 0U; joint_index < joints_.size(); ++joint_index)
      {
        const double start = points[index - 1U].positions[joint_index];
        const double finish = points[index].positions[joint_index];
        positions[joint_index] = start + ((finish - start) * ratio);
      }
      return positions;
    }
  }

  return points.back().positions;
}

std::vector<double> MyController::read_current_positions() const
{
  std::vector<double> positions = hold_positions_;
  if (positions.size() != joints_.size())
  {
    positions.assign(joints_.size(), 0.0);
  }

  if (state_interfaces_.size() == joints_.size())
  {
    for (std::size_t index = 0U; index < joints_.size(); ++index)
    {
      const double value = state_interfaces_[index].get_value();
      if (std::isfinite(value))
      {
        positions[index] = value;
      }
    }
  }

  return positions;
}

std::shared_ptr<MyController::FollowJointTrajectory::Feedback> MyController::make_feedback(
  const rclcpp::Time & time, const rclcpp::Duration & elapsed,
  const std::vector<double> & desired) const
{
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->header.stamp = time;
  feedback->joint_names = joints_;
  feedback->desired.positions = desired;
  feedback->desired.time_from_start = elapsed;
  feedback->actual.positions = read_current_positions();
  feedback->actual.time_from_start = elapsed;
  feedback->error.positions.resize(joints_.size(), 0.0);

  if (feedback->actual.positions.size() == joints_.size())
  {
    for (std::size_t index = 0U; index < joints_.size(); ++index)
    {
      feedback->error.positions[index] = desired[index] - feedback->actual.positions[index];
    }
  }

  return feedback;
}

MyController::FollowJointTrajectory::Result::SharedPtr MyController::make_result(
  int32_t error_code, const std::string & message)
{
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = error_code;
  result->error_string = message;
  return result;
}

std::vector<std::string> MyController::position_interface_names() const
{
  std::vector<std::string> names;
  names.reserve(joints_.size());
  for (const auto & joint : joints_)
  {
    names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return names;
}

void MyController::write_position_commands(const std::vector<double> & positions)
{
  if (positions.size() != command_interfaces_.size())
  {
    return;
  }

  for (std::size_t index = 0U; index < positions.size(); ++index)
  {
    command_interfaces_[index].set_value(positions[index]);
  }
}

}  // namespace my_controller

PLUGINLIB_EXPORT_CLASS(my_controller::MyController, controller_interface::ControllerInterface)
