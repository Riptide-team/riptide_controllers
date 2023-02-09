#include "riptide_controllers/depth_controller.hpp"
#include "depth_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace riptide_controllers {

    DepthController::DepthController() : controller_interface::ControllerInterface() {}

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
        if (params_.d_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'d_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.p_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'p_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.s_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'s_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.thruster_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'thruster_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.pressure_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'pressure_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        feedback_ = std::make_shared<riptide_msgs::action::Depth::Feedback>();
        result_ = std::make_shared<riptide_msgs::action::Depth::Result>();

        action_server_ = rclcpp_action::create_server<riptide_msgs::action::Depth>(
            get_node(),
            "depth",
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
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        command_interfaces_config.names.push_back(prefix + "_" + params_.thruster_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration DepthController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        state_interfaces_config.names.push_back(prefix + "_" + params_.pressure_name + "/pressure");
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn DepthController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DepthController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DepthController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::lock_guard<std::mutex> lock_(depth_mutex_);
        double error = 0;

        // Only if a goal has been provided
        if (goal_handle_ != nullptr) {
            error = requested_depth_ - state_interfaces_[0].get_value();

            // Check if the goal is canceled
            if (goal_handle_->is_canceling()) {
                requested_depth_ = state_interfaces_[0].get_value();
                result_->depth = state_interfaces_[0].get_value();
                goal_handle_->canceled(result_);
                RCLCPP_INFO(get_node()->get_logger(), "Goal canceled");
            }
            // Check if the goal is validated
            else if (std::abs(error) < 0.1) {
                result_->depth = state_interfaces_[0].get_value();
                goal_handle_->succeed(result_);
            }
            // Publish the feedback
            else {
                feedback_->error = state_interfaces_[0].get_value() - requested_depth_;
                goal_handle_->publish_feedback(feedback_);
            }
        }

        double alpha = std::atan(error);

        // double Kp = 1.;
        double u0 = 1.;
        command_interfaces_[0].set_value(u0);
        command_interfaces_[1].set_value(alpha);
        command_interfaces_[2].set_value(alpha);
        
        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse DepthController::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Depth::Goal> goal) {
        std::lock_guard<std::mutex> lock_(depth_mutex_);
        requested_depth_ = goal->requested_depth;
        RCLCPP_INFO(get_node()->get_logger(), "Received depth request with order %f", goal->requested_depth);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DepthController::handle_cancel(const std::shared_ptr<GoalHandleDepth> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void DepthController::handle_accepted(const std::shared_ptr<GoalHandleDepth> goal_handle) {
        goal_handle_ = goal_handle;
    }

} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::DepthController, controller_interface::ControllerInterface)