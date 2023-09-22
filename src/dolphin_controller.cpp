#include "riptide_controllers/dolphin_controller.hpp"
#include "dolphin_controller_parameters.hpp"
#include "controller_interface/chainable_controller_interface.hpp"


#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <chrono>
#include <cmath>
#include <string>

namespace riptide_controllers {

    DolphinController::DolphinController() : controller_interface::ControllerInterface(), dolphin_start_time_(0, 0, rcl_clock_type_t::RCL_ROS_TIME) {}

    controller_interface::CallbackReturn DolphinController::on_init() {
        try {
            param_listener_ = std::make_shared<dolphin_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DolphinController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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
    }

    controller_interface::InterfaceConfiguration DolphinController::command_interface_configuration() const {
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

    controller_interface::InterfaceConfiguration DolphinController::state_interface_configuration() const {
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

    controller_interface::CallbackReturn DolphinController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        goal_handle_ = nullptr;

        // Setting null commands
        command_interfaces_[0].set_value(0.);
        command_interfaces_[1].set_value(0.);
        command_interfaces_[2].set_value(0.);
        command_interfaces_[3].set_value(0.);

        dolphin_start_time_ = get_node()->get_clock()->now();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DolphinController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        goal_handle_ = nullptr;

        // Setting quiet_nan commands
        command_interfaces_[0].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[1].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[2].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[3].set_value(std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DolphinController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        // Check for new params
        if (param_listener_->is_old(params_)) {
            param_listener_->refresh_dynamic_parameters();
            params_ = param_listener_->get_params();
        }

        // Computing remaining_time
        rclcpp::Duration remaining_time = rclcpp::Duration::from_seconds(params_.timeout) - (time - dolphin_start_time_);

        // Case when there is still time
        if (remaining_time > rclcpp::Duration::from_seconds(0)) {
            // Building wanted command orientation from euler angles
            Eigen::AngleAxisd yawAngle(params_.yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchAngle(params_pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollAngle(params_.roll, Eigen::Vector3d::UnitX());
            Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

            command_interfaces_[0].set_value(params_.thruster_velocity);
            command_interfaces_[1].set_value(q.x());
            command_interfaces_[2].set_value(q.y());
            command_interfaces_[3].set_value(q.z());
            command_interfaces_[4].set_value(q.w());
        }
        // Case when timeout is reached
        else {
            RCLCPP_WARN(get_node()->get_logger(), "Timeout reached: Publiching null commands");

            // Setting null commands
            command_interfaces_[0].set_value(0);
            command_interfaces_[1].set_value(state_interfaces_[1].get_value());
            command_interfaces_[2].set_value(state_interfaces_[2].get_value());
            command_interfaces_[3].set_value(state_interfaces_[3].get_value());
            command_interfaces_[4].set_value(state_interfaces_[4].get_value());
            return controller_interface::return_type::OK;
        }

        // Publishing controller state
        rt_controller_state_publisher_->lock();
        rt_controller_state_publisher_->msg_.header.stamp = time;
        rt_controller_state_publisher_->msg_.remaining_time = remaining_time;

        // Reference
        rt_controller_state_publisher_->msg_.depth_reference = params_.depth_reference;

        // Feedback
        rt_controller_state_publisher_->msg_.depth_feedback = state_interfaces_[0].get_value();

        // Error
        rt_controller_state_publisher_->msg_.depth_error = params_.depth_reference - state_interfaces_[0].get_value();
        
        // Output
        rt_controller_state_publisher_->msg_.velocity_output = command_interfaces_[0];
        rt_controller_state_publisher_->msg_.orientation_output.x = command_interfaces_[1];
        rt_controller_state_publisher_->msg_.orientation_output.y = command_interfaces_[2];
        rt_controller_state_publisher_->msg_.orientation_output.z = command_interfaces_[3];
        rt_controller_state_publisher_->msg_.orientation_output.w = command_interfaces_[4];

        // Publishing controller state
        rt_controller_state_publisher_->unlockAndPublish();
        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::DolphinController, controller_interface::ControllerInterface)