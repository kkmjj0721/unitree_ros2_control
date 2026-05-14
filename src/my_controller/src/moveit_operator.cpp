#include <map>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
constexpr char kPlanningGroup[] = "arm";
constexpr char kTipLink[] = "link3_tip_sphere";
constexpr char kTargetPoseTopic[] = "~/target_pose";
constexpr char kStatusTopic[] = "~/status";
constexpr char kResetZeroService[] = "~/reset_zero";
constexpr char kEnableMotionService[] = "~/enable_motion";
constexpr char kDisableMotionService[] = "~/disable_motion";
constexpr char kControllerMotionGateService[] =
  "/arm_controller/follow_joint_trajectory/set_motion_enabled";
constexpr char kDisplayTrajectoryTopic[] = "/display_planned_path";
constexpr double kDefaultScaling = 0.8;
constexpr std::size_t kTargetPoseQueueDepth = 1;
constexpr auto kGateServiceWait = std::chrono::milliseconds(500);
constexpr auto kGateResponseWait = std::chrono::milliseconds(1000);
constexpr auto kStartupGateReconcileDelay = std::chrono::milliseconds(100);

int64_t system_time_now_ns()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

std::optional<int64_t> stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if ((stamp.sec == 0) && (stamp.nanosec == 0U))
  {
    return std::nullopt;
  }

  return (static_cast<int64_t>(stamp.sec) * 1000000000LL) +
    static_cast<int64_t>(stamp.nanosec);
}

double bounded_scaling(double value)
{
  if (!std::isfinite(value))
  {
    return kDefaultScaling;
  }
  return std::clamp(value, 0.01, 1.0);
}
}  // namespace

class UnitreeMoveItOperator
{
public:
  explicit UnitreeMoveItOperator(const rclcpp::Node::SharedPtr & node)
  : node_(node), move_group_(node_, kPlanningGroup)
  {
    double velocity_scaling = kDefaultScaling;
    double acceleration_scaling = kDefaultScaling;
    node_->get_parameter_or("velocity_scaling", velocity_scaling, velocity_scaling);
    node_->get_parameter_or("acceleration_scaling", acceleration_scaling, acceleration_scaling);
    velocity_scaling = bounded_scaling(velocity_scaling);
    acceleration_scaling = bounded_scaling(acceleration_scaling);

    move_group_.setEndEffectorLink(kTipLink);
    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scaling);
    move_group_.startStateMonitor();

    RCLCPP_INFO(
      node_->get_logger(), "MoveIt scaling: velocity=%.3f acceleration=%.3f",
      velocity_scaling, acceleration_scaling);

    motion_command_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    enable_service_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reset_service_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    disable_service_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    controller_gate_client_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    startup_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    status_pub_ = node_->create_publisher<std_msgs::msg::String>(kStatusTopic, 10);
    display_trajectory_pub_ =
      node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(kDisplayTrajectoryTopic, 10);
    controller_gate_client_ = node_->create_client<std_srvs::srv::SetBool>(
      kControllerMotionGateService, rmw_qos_profile_services_default, controller_gate_client_group_);

    rclcpp::SubscriptionOptions pose_subscription_options;
    pose_subscription_options.callback_group = motion_command_group_;
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      kTargetPoseTopic, rclcpp::QoS(kTargetPoseQueueDepth),
      [this](
        const geometry_msgs::msg::PoseStamped::SharedPtr message,
        const rclcpp::MessageInfo & message_info)
      {
        handle_target_pose(*message, message_info);
      },
      pose_subscription_options);
    reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      kResetZeroService,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handle_reset_zero(request, response);
      },
      rmw_qos_profile_services_default,
      reset_service_group_);
    enable_motion_service_ = node_->create_service<std_srvs::srv::Trigger>(
      kEnableMotionService,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handle_enable_motion(request, response);
      },
      rmw_qos_profile_services_default,
      enable_service_group_);
    disable_motion_service_ = node_->create_service<std_srvs::srv::Trigger>(
      kDisableMotionService,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handle_disable_motion(request, response);
      },
      rmw_qos_profile_services_default,
      disable_service_group_);

    startup_reconcile_timer_ = node_->create_wall_timer(
      kStartupGateReconcileDelay,
      [this]()
      {
        startup_reconcile_timer_->cancel();
        reconcile_startup_controller_gate();
      },
      startup_group_);

    publish_status("startup: operator gate disabled; reconciling controller gate disabled");
  }

private:
  using GateFuture = rclcpp::Client<std_srvs::srv::SetBool>::FutureAndRequestId;

  struct ArmSnapshot
  {
    bool armed{false};
    uint64_t generation{0U};
    int64_t ros_enabled_after_ns{0};
    int64_t wall_enabled_after_ns{0};
  };

  struct ControllerGateResult
  {
    bool success{false};
    std::string message;
  };

  class MotionCommandGuard
  {
  public:
    explicit MotionCommandGuard(std::atomic_bool * active) : active_(active) {}
    MotionCommandGuard(const MotionCommandGuard &) = delete;
    MotionCommandGuard & operator=(const MotionCommandGuard &) = delete;
    ~MotionCommandGuard()
    {
      if (active_ != nullptr)
      {
        active_->store(false);
      }
    }

  private:
    std::atomic_bool * active_;
  };

  ArmSnapshot get_arm_snapshot() const
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    ArmSnapshot snapshot;
    snapshot.armed = motion_armed_.load() && controller_gate_reconciled_.load() &&
      !controller_gate_fault_latched_.load();
    snapshot.generation = arm_generation_;
    snapshot.ros_enabled_after_ns = arm_enabled_ros_time_ns_;
    snapshot.wall_enabled_after_ns = arm_enabled_wall_time_ns_;
    return snapshot;
  }

  bool arm_generation_is_current(const uint64_t generation) const
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    return motion_armed_.load() && controller_gate_reconciled_.load() &&
      !controller_gate_fault_latched_.load() && (arm_generation_ == generation);
  }

  void disarm_operator_gate()
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    motion_armed_.store(false);
  }

  void arm_operator_gate()
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    ++arm_generation_;
    arm_enabled_ros_time_ns_ = node_->now().nanoseconds();
    arm_enabled_wall_time_ns_ = system_time_now_ns();
    motion_armed_.store(true);
  }

  void latch_controller_gate_fault()
  {
    disarm_operator_gate();
    controller_gate_reconciled_.store(false);
    controller_gate_fault_latched_.store(true);
  }

  void mark_controller_gate_confirmed()
  {
    controller_gate_fault_latched_.store(false);
    controller_gate_reconciled_.store(true);
  }

  bool try_begin_motion_command(const uint64_t generation)
  {
    bool expected = false;
    if (!motion_command_active_.compare_exchange_strong(expected, true))
    {
      return false;
    }

    if (!arm_generation_is_current(generation))
    {
      motion_command_active_.store(false);
      return false;
    }

    return true;
  }

  bool target_pose_matches_arm_generation(
    const geometry_msgs::msg::PoseStamped & target,
    const rclcpp::MessageInfo & message_info,
    const ArmSnapshot & arm_snapshot,
    std::string * reason) const
  {
    const auto & rmw_info = message_info.get_rmw_message_info();
    if (rmw_info.received_timestamp > 0)
    {
      if (rmw_info.received_timestamp < arm_snapshot.wall_enabled_after_ns)
      {
        if (reason != nullptr)
        {
          *reason = "received before current arm generation";
        }
        return false;
      }
      return true;
    }

    const auto header_stamp_ns = stamp_to_nanoseconds(target.header.stamp);
    if (!header_stamp_ns.has_value())
    {
      if (reason != nullptr)
      {
        *reason = "missing receipt/header timestamp";
      }
      return false;
    }

    if (header_stamp_ns.value() < arm_snapshot.ros_enabled_after_ns)
    {
      if (reason != nullptr)
      {
        *reason = "stamped before current arm generation";
      }
      return false;
    }

    return true;
  }

  void handle_target_pose(
    const geometry_msgs::msg::PoseStamped & target,
    const rclcpp::MessageInfo & message_info)
  {
    const auto arm_snapshot = get_arm_snapshot();
    if (!arm_snapshot.armed)
    {
      publish_status("operator gate disabled: pose target rejected");
      return;
    }

    std::string stale_reason;
    if (!target_pose_matches_arm_generation(target, message_info, arm_snapshot, &stale_reason))
    {
      publish_status("operator gate enabled: stale pose target rejected: " + stale_reason);
      return;
    }

    if (!try_begin_motion_command(arm_snapshot.generation))
    {
      publish_status("motion busy or operator gate changed: pose target rejected");
      return;
    }
    MotionCommandGuard motion_command_guard(&motion_command_active_);

    std::unique_lock<std::mutex> move_group_lock(move_group_mutex_, std::try_to_lock);
    if (!move_group_lock.owns_lock())
    {
      publish_status("move group busy: pose target rejected");
      return;
    }

    if (!arm_generation_is_current(arm_snapshot.generation))
    {
      publish_status("operator gate changed: pose target rejected before planning");
      return;
    }

    publish_status("planning pose target");
    move_group_.clearPoseTargets();

    if (!move_group_.setPoseTarget(target, kTipLink))
    {
      publish_status("pose target rejected");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_.plan(plan);
    if (!static_cast<bool>(plan_result))
    {
      publish_status("planning failed: " + moveit::core::error_code_to_string(plan_result));
      return;
    }

    publish_display_trajectory(plan);
    if (!arm_generation_is_current(arm_snapshot.generation))
    {
      publish_status("operator gate disabled: pose target execution rejected");
      return;
    }

    publish_status("executing pose target");
    const auto execute_result = move_group_.execute(plan);
    if (!static_cast<bool>(execute_result))
    {
      publish_status("execution failed: " + moveit::core::error_code_to_string(execute_result));
      return;
    }

    if (!arm_generation_is_current(arm_snapshot.generation))
    {
      publish_status("pose target stopped: operator gate changed during execution");
      return;
    }

    publish_status("pose target complete");
  }

  void handle_reset_zero(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    const auto arm_snapshot = get_arm_snapshot();
    if (!arm_snapshot.armed)
    {
      response->success = false;
      response->message = "operator gate disabled";
      publish_status("operator gate disabled: reset_zero rejected");
      return;
    }

    if (!try_begin_motion_command(arm_snapshot.generation))
    {
      response->success = false;
      response->message = "motion active or operator gate changed";
      publish_status("reset_zero rejected: " + response->message);
      return;
    }
    MotionCommandGuard motion_command_guard(&motion_command_active_);

    std::unique_lock<std::mutex> move_group_lock(move_group_mutex_, std::try_to_lock);
    if (!move_group_lock.owns_lock())
    {
      response->success = false;
      response->message = "move group busy";
      publish_status("reset_zero rejected: " + response->message);
      return;
    }

    if (!arm_generation_is_current(arm_snapshot.generation))
    {
      response->success = false;
      response->message = "reset_zero request is stale for current arm generation";
      publish_status(response->message);
      return;
    }

    publish_status("planning reset_zero");
    move_group_.clearPoseTargets();

    const std::map<std::string, double> zero_target = {
      {"joint1", 0.0},
      {"joint2", 0.0},
      {"joint3", 0.0},
    };

    if (!move_group_.setJointValueTarget(zero_target))
    {
      response->success = false;
      response->message = "joint target rejected";
      publish_status(response->message);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_.plan(plan);
    if (!static_cast<bool>(plan_result))
    {
      response->success = false;
      response->message = "planning failed: " + moveit::core::error_code_to_string(plan_result);
      publish_status(response->message);
      return;
    }

    publish_display_trajectory(plan);
    if (!arm_generation_is_current(arm_snapshot.generation))
    {
      response->success = false;
      response->message = "operator gate changed before execution";
      publish_status("operator gate changed: reset_zero execution rejected");
      return;
    }

    publish_status("executing reset_zero");
    const auto execute_result = move_group_.execute(plan);
    if (!static_cast<bool>(execute_result))
    {
      response->success = false;
      response->message = "execution failed: " + moveit::core::error_code_to_string(execute_result);
      publish_status(response->message);
      return;
    }

    response->success = true;
    response->message = "reset_zero complete";
    if (!arm_generation_is_current(arm_snapshot.generation))
    {
      response->success = false;
      response->message = "reset_zero stopped: operator gate changed during execution";
      publish_status(response->message);
      return;
    }

    publish_status(response->message);
  }

  void handle_enable_motion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    const uint64_t command_generation = gate_command_generation_.fetch_add(1U) + 1U;
    disarm_operator_gate();

    if (controller_gate_fault_latched_.load())
    {
      response->success = false;
      response->message =
        "operator gate disabled; controller gate fault latched, call disable_motion to recover";
      RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
      publish_status(response->message);
      return;
    }

    if (!controller_gate_reconciled_.load())
    {
      const auto reconcile_result = set_controller_motion_gate(false, kGateServiceWait);
      if (!reconcile_result.success)
      {
        latch_controller_gate_fault();
        response->success = false;
        response->message = "operator gate disabled; controller gate not reconciled: " +
          reconcile_result.message;
        RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
        publish_status(response->message);
        return;
      }
      mark_controller_gate_confirmed();
    }

    if (gate_command_generation_.load() != command_generation)
    {
      const auto disable_result = set_controller_motion_gate(false, std::chrono::milliseconds(0));
      response->success = false;
      response->message = "operator gate disabled; enable superseded by newer gate command";
      if (disable_result.success)
      {
        mark_controller_gate_confirmed();
      }
      else
      {
        latch_controller_gate_fault();
        response->message += "; controller gate state unknown: " + disable_result.message;
      }
      publish_status(response->message);
      return;
    }

    const auto gate_result = set_controller_motion_gate(true, kGateServiceWait);
    if (!gate_result.success)
    {
      disarm_operator_gate();
      const auto disable_result = set_controller_motion_gate(false, std::chrono::milliseconds(0));
      response->success = false;
      response->message = "operator gate disabled; controller gate enable failed: " +
        gate_result.message;
      if (disable_result.success)
      {
        mark_controller_gate_confirmed();
        response->message += "; controller gate disabled after enable failure";
      }
      else
      {
        latch_controller_gate_fault();
        response->message += "; controller gate state unknown: disable after enable failure failed: " +
          disable_result.message;
      }
      RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
      publish_status(response->message);
      return;
    }

    if (gate_command_generation_.load() != command_generation)
    {
      disarm_operator_gate();
      const auto disable_result = set_controller_motion_gate(false, std::chrono::milliseconds(0));
      response->success = false;
      response->message = "operator gate disabled; controller enable response ignored after newer disable";
      if (disable_result.success)
      {
        mark_controller_gate_confirmed();
      }
      else
      {
        latch_controller_gate_fault();
        response->message += "; controller gate state unknown: " + disable_result.message;
      }
      publish_status(response->message);
      return;
    }

    arm_operator_gate();
    mark_controller_gate_confirmed();
    response->success = true;
    response->message = "operator gate enabled; controller gate enabled";
    publish_status(response->message);
  }

  void handle_disable_motion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    (void)(gate_command_generation_.fetch_add(1U) + 1U);
    disarm_operator_gate();
    publish_status("operator gate disabled; requesting controller gate disabled");

    std::string send_error;
    auto pending_gate_request = send_controller_motion_gate_request(
      false, std::chrono::milliseconds(0), &send_error);

    ControllerGateResult gate_result;
    if (pending_gate_request.has_value())
    {
      gate_result = finish_controller_motion_gate_request(false, pending_gate_request.value());
    }
    else
    {
      gate_result.message = send_error;
    }

    if (gate_result.success)
    {
      mark_controller_gate_confirmed();
    }
    else
    {
      latch_controller_gate_fault();
    }

    {
      std::lock_guard<std::mutex> move_group_lock(move_group_mutex_);
      move_group_.stop();
      move_group_.clearPoseTargets();
    }

    if (!gate_result.success)
    {
      response->success = false;
      response->message = "operator gate disabled; controller gate state unknown: disable failed: " +
        gate_result.message;
      RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
      publish_status(response->message);
      return;
    }

    response->success = true;
    response->message = "operator gate disabled; controller gate disabled";
    publish_status(response->message);
  }

  std::optional<GateFuture> send_controller_motion_gate_request(
    const bool enable_motion,
    const std::chrono::milliseconds service_wait,
    std::string * error_message)
  {
    const char * const action = enable_motion ? "enable" : "disable";

    if (!controller_gate_client_->wait_for_service(service_wait))
    {
      if (error_message != nullptr)
      {
        *error_message = "service unavailable";
      }
      return std::nullopt;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable_motion;
    try
    {
      return controller_gate_client_->async_send_request(request);
    }
    catch (const std::exception & exception)
    {
      if (error_message != nullptr)
      {
        *error_message = std::string(action) + " request send failed: " + exception.what();
      }
      return std::nullopt;
    }
  }

  ControllerGateResult finish_controller_motion_gate_request(
    const bool enable_motion,
    GateFuture & future)
  {
    ControllerGateResult result;
    const char * const action = enable_motion ? "enable" : "disable";

    if (future.wait_for(kGateResponseWait) != std::future_status::ready)
    {
      (void)controller_gate_client_->remove_pending_request(future);
      result.message = std::string(action) + " request timed out";
      return result;
    }

    try
    {
      const auto gate_response = future.get();
      if (!gate_response)
      {
        result.message = std::string(action) + " request returned no response";
        return result;
      }

      if (!gate_response->success)
      {
        result.message = std::string("controller rejected ") + action + ": " +
          gate_response->message;
        return result;
      }

      result.success = true;
      result.message = gate_response->message;
      return result;
    }
    catch (const std::exception & exception)
    {
      result.message = std::string(action) + " request failed: " + exception.what();
      return result;
    }
  }

  ControllerGateResult set_controller_motion_gate(
    const bool enable_motion,
    const std::chrono::milliseconds service_wait)
  {
    ControllerGateResult result;
    std::string send_error;
    auto pending_gate_request = send_controller_motion_gate_request(
      enable_motion, service_wait, &send_error);
    if (!pending_gate_request.has_value())
    {
      result.message = send_error;
      return result;
    }

    return finish_controller_motion_gate_request(enable_motion, pending_gate_request.value());
  }

  void reconcile_startup_controller_gate()
  {
    const uint64_t command_generation = gate_command_generation_.fetch_add(1U) + 1U;
    disarm_operator_gate();

    const auto gate_result = set_controller_motion_gate(false, kGateServiceWait);
    if (!gate_result.success)
    {
      latch_controller_gate_fault();
      const std::string message =
        "fault: operator gate disabled; controller gate state unknown at startup: " +
        gate_result.message;
      RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
      publish_status(message);
      return;
    }

    if (gate_command_generation_.load() != command_generation)
    {
      publish_status("startup reconcile superseded by newer gate command");
      return;
    }

    mark_controller_gate_confirmed();
    publish_status("ready: operator gate disabled; controller gate disabled");
  }

  void publish_status(const std::string & text) const
  {
    std_msgs::msg::String message;
    message.data = text;
    status_pub_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "%s", text.c_str());
  }

  void publish_display_trajectory(const moveit::planning_interface::MoveGroupInterface::Plan & plan) const
  {
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    display_trajectory_pub_->publish(display_trajectory);

    if (!plan.trajectory_.joint_trajectory.points.empty())
    {
      const auto & final_time = plan.trajectory_.joint_trajectory.points.back().time_from_start;
      const double seconds = static_cast<double>(final_time.sec) +
        (static_cast<double>(final_time.nanosec) * 1e-9);
      RCLCPP_INFO(
        node_->get_logger(), "Planned trajectory: %zu points, %.3f seconds",
        plan.trajectory_.joint_trajectory.points.size(), seconds);
    }
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  mutable std::mutex move_group_mutex_;
  std::atomic_bool motion_command_active_{false};
  std::atomic_bool motion_armed_{false};
  mutable std::mutex arm_state_mutex_;
  uint64_t arm_generation_{0U};
  int64_t arm_enabled_ros_time_ns_{0};
  int64_t arm_enabled_wall_time_ns_{0};
  std::atomic_uint64_t gate_command_generation_{0U};
  std::atomic_bool controller_gate_reconciled_{false};
  std::atomic_bool controller_gate_fault_latched_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
  rclcpp::CallbackGroup::SharedPtr motion_command_group_;
  rclcpp::CallbackGroup::SharedPtr enable_service_group_;
  rclcpp::CallbackGroup::SharedPtr reset_service_group_;
  rclcpp::CallbackGroup::SharedPtr disable_service_group_;
  rclcpp::CallbackGroup::SharedPtr controller_gate_client_group_;
  rclcpp::CallbackGroup::SharedPtr startup_group_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr controller_gate_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_motion_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_motion_service_;
  rclcpp::TimerBase::SharedPtr startup_reconcile_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
    "unitree_moveit_operator",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto unitree_operator = std::make_shared<UnitreeMoveItOperator>(node);
  (void)unitree_operator;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
