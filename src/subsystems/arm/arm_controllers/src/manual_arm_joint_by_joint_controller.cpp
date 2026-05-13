#include "athena_arm_controllers/manual_arm_joint_by_joint_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

}  // namespace

namespace arm_controllers
{
ManualArmJointByJointController::ManualArmJointByJointController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ManualArmJointByJointController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<manual_arm_joint_by_joint_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ManualArmJointByJointController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  num_joints = static_cast<int>(params_.joints.size());
  joint_velocities_.resize(num_joints, 0.0); // Output
  max_velocities_ = params_.joint_max_velocities;
  virtual_four_bar_coupling_ratio_ = params_.virtual_four_bar_coupling_ratio;

  // Reference-input freshness watchdog. Default 0.5 s; overridable via
  // the "controller_input_timeout" parameter (seconds, 0 disables).
  input_watchdog_.init(get_node(), 0.5);

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "controller_input", subscribers_qos,
    std::bind(&ManualArmJointByJointController::reference_callback, this, std::placeholders::_1));

  // Pre-fill the RT buffer with an empty message so the RT loop never
  // dereferences a null shared_ptr before the first real message arrives.
  // Freshness is tracked separately by input_watchdog_.
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  msg->axes.assign(joystick_axes, 0.0);
  msg->buttons.assign(joystick_buttons, 0);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void ManualArmJointByJointController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
  input_watchdog_.notify(get_node()->now());
}

controller_interface::InterfaceConfiguration ManualArmJointByJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(num_joints);

  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/velocity");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ManualArmJointByJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(num_joints);
  for (const auto & joint : params_.joints) {
    state_interfaces_config.names.push_back(joint + "/velocity");
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn ManualArmJointByJointController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Treat freshly activated controllers as "no input received yet" so the
  // safe-stop default holds until the operator publishes.
  input_watchdog_.reset();
  safe_stopper_.prepare(command_interfaces_);

  // Pre-zero command interfaces so the very first update tick (before
  // command logic runs) reads finite values for the position-latch path.
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ManualArmJointByJointController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ManualArmJointByJointController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const bool fresh = input_watchdog_.is_fresh(time);
  input_watchdog_.log_state_transition(fresh, get_node()->get_logger());

  if (!fresh) {
    safe_stopper_.apply(command_interfaces_);

    if (state_publisher_ && state_publisher_->trylock())
    {
      state_publisher_->msg_.header.stamp = time;
      state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
      state_publisher_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

  // We're fresh: drop any latched safe-stop state so the next stale
  // episode latches afresh.
  safe_stopper_.reset();

  auto current_ref = input_ref_.readFromRT();

  // Base Yaw: L/R Left Stick
  joint_velocities_[0] = ((*current_ref)->buttons[1] == 1) ? 0.0 : (*current_ref)->axes[0] * max_velocities_[0];

  // Shoulder Pitch: U/D Left Stick
  joint_velocities_[1] = ((*current_ref)->buttons[1] == 1) ? 0.0 : (*current_ref)->axes[1] * max_velocities_[1];

  // Elbow Pitch: U/D Right Stick
  joint_velocities_[2] = (*current_ref)->axes[3] * max_velocities_[2];

  // Virtual four-bar compensation. With the current 1.0 ratio, shoulder-only motion commands
  // the elbow motor equally in the opposite direction so the net elbow joint angle stays fixed.
  joint_velocities_[2] += virtual_four_bar_coupling_ratio_ * joint_velocities_[1];

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
    {
      joint_velocities_[i] /= 2;
    }
    command_interfaces_[i].set_value(joint_velocities_[i]);
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::ManualArmJointByJointController, controller_interface::ControllerInterface)
