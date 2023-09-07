#include "riptide_controllers/depth_controller.hpp"
#include "depth_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <riptide_msgs/action/depth.hpp>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>


namespace riptide_controllers {

    controller_interface::CallbackReturn DepthController::on_init() {
        try {
            param_listener_ = std::make_shared<depth_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DepthController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();

        if (params_.imu_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'imu_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.pressure_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'pressure_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.orientation_reference_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'pressure_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        // Configuring controller state publisher
        controller_state_publisher_ = get_node()->create_publisher<ControllerStateType>("~/controller_state", rclcpp::SystemDefaultsQoS());
        rt_controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateType>>(controller_state_publisher_);

        // Init Depth action
        action_server_ = rclcpp_action::create_server<Action>(
            get_node(),
            "~/depth",
            std::bind(&DepthController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DepthController::handle_cancel, this, std::placeholders::_1),
            std::bind(&DepthController::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DepthController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Adding prefix if specified
        std::string prefix;
        if (params_.prefix.empty()) {
            prefix = "";
        }
        else {
            prefix = params_.prefix + "_";
        }

        command_interfaces_config.names.push_back(prefix + params_.orientation_reference_joint + "/linear_velocity.x");
        command_interfaces_config.names.push_back(prefix + params_.orientation_reference_joint + "/orientation.x");
        command_interfaces_config.names.push_back(prefix + params_.orientation_reference_joint + "/orientation.y");
        command_interfaces_config.names.push_back(prefix + params_.orientation_reference_joint + "/orientation.z");
        command_interfaces_config.names.push_back(prefix + params_.orientation_reference_joint + "/orientation.w");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration DepthController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Adding prefix if specified
        std::string prefix;
        if (params_.prefix.empty()) {
            prefix = "";
        }
        else {
            prefix = params_.prefix + "_";
        }

        state_interfaces_config.names.push_back(prefix + params_.pressure_name + "/depth");
        state_interfaces_config.names.push_back(prefix + params_.imu_name + "/orientation.x");
        state_interfaces_config.names.push_back(prefix + params_.imu_name + "/orientation.y");
        state_interfaces_config.names.push_back(prefix + params_.imu_name + "/orientation.z");
        state_interfaces_config.names.push_back(prefix + params_.imu_name + "/orientation.w");
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn DepthController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        
        // Setting command interfaces to 0
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(0);
        command_interfaces_[2].set_value(0);

        // Resetting goal handle
        std::scoped_lock<std::mutex> lock_(goal_mutex_);
        goal_handle_ = nullptr;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DepthController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {

        // Setting command interfaces to Nan
        command_interfaces_[0].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[1].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[2].set_value(std::numeric_limits<double>::quiet_NaN());

        // Resetting goal handle
        std::scoped_lock<std::mutex> lock_(goal_mutex_);
        goal_handle_ = nullptr;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DepthController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        std::lock_guard<std::mutex> lock_(goal_mutex_);

        // Current time storage
        current_time_ = time;

        // Checking if the goal_handle is not nullptr
        if (goal_handle_ != nullptr) {
            
            // Checking if goal is cancelling
            if (goal_handle_->is_canceling()) {
                // Setting null velocity and identity orientation
                command_interfaces_[0].set_value(0);
                command_interfaces_[1].set_value(state_interfaces_[1].get_value());
                command_interfaces_[2].set_value(state_interfaces_[2].get_value());
                command_interfaces_[3].set_value(state_interfaces_[3].get_value());
                command_interfaces_[4].set_value(state_interfaces_[4].get_value());

                auto result = std::make_shared<Action::Result>();
                result->depth = state_interfaces_[0].get_value();
                rclcpp::Duration total_duration = time - action_start_time_;
                result->duration.sec = total_duration.seconds();
                result->duration.nanosec = total_duration.nanoseconds();
                goal_handle_->canceled(result);
                goal_handle_ = nullptr;

                // Publishing controller state
                rt_controller_state_publisher_->lock();
                rt_controller_state_publisher_->msg_.header.stamp = time;
                rt_controller_state_publisher_->msg_.reference_depth = 0.;
                rt_controller_state_publisher_->msg_.feedback_depth = state_interfaces_[0].get_value();
                rt_controller_state_publisher_->msg_.error_depth = - state_interfaces_[0].get_value();
                rt_controller_state_publisher_->msg_.velocity_output = 0.;
                rt_controller_state_publisher_->msg_.output.x = state_interfaces_[1].get_value();
                rt_controller_state_publisher_->msg_.output.y = state_interfaces_[2].get_value();
                rt_controller_state_publisher_->msg_.output.z = state_interfaces_[3].get_value();
                rt_controller_state_publisher_->msg_.output.w = state_interfaces_[4].get_value();
                rt_controller_state_publisher_->unlockAndPublish();

                return controller_interface::return_type::OK;
            }

            // If the goal is still executing
            if (goal_handle_->is_executing()) {

                // Check if the timeout is expired -> if so, succeed the goal
                if (time - action_start_time_ > rclcpp::Duration(goal_handle_->get_goal()->timeout.sec, goal_handle_->get_goal()->timeout.nanosec)) {
                    // Setting null velocity and identity orientation
                    command_interfaces_[0].set_value(0);
                    command_interfaces_[1].set_value(state_interfaces_[1].get_value());
                    command_interfaces_[2].set_value(state_interfaces_[2].get_value());
                    command_interfaces_[3].set_value(state_interfaces_[3].get_value());
                    command_interfaces_[4].set_value(state_interfaces_[4].get_value());

                    auto result = std::make_shared<Action::Result>();
                    result->depth = state_interfaces_[0].get_value();
                    rclcpp::Duration total_duration = time - action_start_time_;
                    result->duration.sec = total_duration.seconds();
                    result->duration.nanosec = total_duration.nanoseconds();
                    goal_handle_->succeed(result);
                    goal_handle_ = nullptr;

                    // Publishing controller state
                    rt_controller_state_publisher_->lock();
                    rt_controller_state_publisher_->msg_.header.stamp = time;
                    rt_controller_state_publisher_->msg_.reference_depth = 0.;
                    rt_controller_state_publisher_->msg_.feedback_depth = state_interfaces_[0].get_value();
                    rt_controller_state_publisher_->msg_.error_depth = - state_interfaces_[0].get_value();
                    rt_controller_state_publisher_->msg_.velocity_output = 0.;
                    rt_controller_state_publisher_->msg_.output.x = state_interfaces_[1].get_value();
                    rt_controller_state_publisher_->msg_.output.y = state_interfaces_[2].get_value();
                    rt_controller_state_publisher_->msg_.output.z = state_interfaces_[3].get_value();
                    rt_controller_state_publisher_->msg_.output.w = state_interfaces_[4].get_value();
                    rt_controller_state_publisher_->unlockAndPublish();

                    return controller_interface::return_type::OK;
                }

                else {
                    // Computing depth error (positive value is go downwards, positive value is go upwards)
                    double depth_error = goal_handle_->get_goal()->depth - state_interfaces_[0].get_value();

                    // Computing wanted pitch (- comes from orientation convention, positive error -> going down, negative error -> going up)
                    double wanted_pitch = - params_.K * std::atan(depth_error / params_.r);

                    // Building wanted command orientation from euler angles
                    Eigen::AngleAxisd yawAngle(params_.yaw, Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchAngle(wanted_pitch, Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollAngle(params_.roll, Eigen::Vector3d::UnitX());
                    
                    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

                    // Setting command interfaces
                    command_interfaces_[0].set_value(params_.thruster_velocity);
                    command_interfaces_[1].set_value(q.x());
                    command_interfaces_[2].set_value(q.y());
                    command_interfaces_[3].set_value(q.z());
                    command_interfaces_[4].set_value(q.w());

                    // Publish feedback
                    auto feedback = std::make_shared<Action::Feedback>();
                    feedback->depth_error = depth_error;
                    rclcpp::Duration reamining_time = action_start_time_ + rclcpp::Duration(goal_handle_->get_goal()->timeout.sec, goal_handle_->get_goal()->timeout.nanosec) - time;
                    feedback->remaining_time.sec = reamining_time.seconds();
                    feedback->remaining_time.nanosec = reamining_time.nanoseconds();

                    goal_handle_->publish_feedback(feedback);

                    // Publishing controller state
                    rt_controller_state_publisher_->lock();
                    rt_controller_state_publisher_->msg_.header.stamp = time;
                    rt_controller_state_publisher_->msg_.reference_depth = goal_handle_->get_goal()->depth;
                    rt_controller_state_publisher_->msg_.feedback_depth = state_interfaces_[0].get_value();
                    rt_controller_state_publisher_->msg_.error_depth = depth_error;
                    rt_controller_state_publisher_->msg_.velocity_output = params_.thruster_velocity;
                    rt_controller_state_publisher_->msg_.output.x = q.x();
                    rt_controller_state_publisher_->msg_.output.y = q.y();
                    rt_controller_state_publisher_->msg_.output.z = q.z();
                    rt_controller_state_publisher_->msg_.output.w = q.w();
                    rt_controller_state_publisher_->unlockAndPublish();

                    return controller_interface::return_type::OK;
                }
            }
        }

        // Setting null velocity and identity orientation
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(state_interfaces_[1].get_value());
        command_interfaces_[2].set_value(state_interfaces_[2].get_value());
        command_interfaces_[3].set_value(state_interfaces_[3].get_value());
        command_interfaces_[4].set_value(state_interfaces_[4].get_value());

        // Publishing controller state
        rt_controller_state_publisher_->lock();
        rt_controller_state_publisher_->msg_.header.stamp = time;
        rt_controller_state_publisher_->msg_.reference_depth = 0.;
        rt_controller_state_publisher_->msg_.feedback_depth = state_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.error_depth = - state_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.velocity_output = 0.;
        rt_controller_state_publisher_->msg_.output.x = state_interfaces_[1].get_value();
        rt_controller_state_publisher_->msg_.output.y = state_interfaces_[2].get_value();
        rt_controller_state_publisher_->msg_.output.z = state_interfaces_[3].get_value();
        rt_controller_state_publisher_->msg_.output.w = state_interfaces_[4].get_value();
        rt_controller_state_publisher_->unlockAndPublish();

        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse DepthController::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal) {
        std::lock_guard<std::mutex> lock_(goal_mutex_);

        // Checking requested depth > 0
        if (goal->depth < 0.) {
            RCLCPP_ERROR(get_node()->get_logger(), "Requested depth must be positive");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Checking duration > 0
        if (static_cast<double>(goal->timeout.sec) + 1e-9 * static_cast<double>(goal->timeout.nanosec) < 0.) {
            RCLCPP_ERROR(get_node()->get_logger(), "Requested duration must be positive");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Requested action: depth=%fm, duration=%fs", goal->depth, static_cast<double>(goal->timeout.sec) + 1e-9 * static_cast<double>(goal->timeout.nanosec));
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DepthController::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");

        // Resetting goal_handle
        goal_handle_ = nullptr;

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void DepthController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Handle accepted");

        // Saving goal handle
        std::lock_guard<std::mutex> lock_(goal_mutex_);
        goal_handle_ = goal_handle;
        action_start_time_ = current_time_;
    }

} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::DepthController, controller_interface::ControllerInterface)
