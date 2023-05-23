#include "riptide_controllers/depth_controller.hpp"
#include "depth_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

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

        // Init Depth action
        feedback_ = std::make_shared<Action::Feedback>();
        result_ = std::make_shared<Action::Result>();

        action_server_ = rclcpp_action::create_server<Action>(
            get_node(),
            "depth",
            std::bind(&DepthController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DepthController::handle_cancel, this, std::placeholders::_1),
            std::bind(&DepthController::handle_accepted, this, std::placeholders::_1)
        );

        K_inf_ = params_.K_inf;
        K_fin_ = params_.K_fin;
        r_fin_ = params_.Range_fin;
        r_ = params_.r;

        // Init Immersion action
        immerse_feedback_ = std::make_shared<ImmerseAction::Feedback>();
        immerse_result_ = std::make_shared<ImmerseAction::Result>();

        immerse_action_server_ = rclcpp_action::create_server<ImmerseAction>(
            get_node(),
            "immerse",
            std::bind(&DepthController::immerse_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DepthController::immerse_handle_cancel, this, std::placeholders::_1),
            std::bind(&DepthController::immerse_handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_node()->get_logger(), "Parameters: %f, %f, %f %f", params_.thruster_velocity, K_inf_, K_fin_, r_);

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
        state_interfaces_config.names.push_back(prefix + "_" + params_.pressure_name + "/depth");
        state_interfaces_config.names.push_back(prefix + "_" + params_.imu_name + "/linear_acceleration.x");
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn DepthController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(0);
        command_interfaces_[2].set_value(0);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DepthController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(0);
        command_interfaces_[2].set_value(0);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DepthController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::lock_guard<std::mutex> lock_(depth_mutex_);
        current_depth_ = state_interfaces_[0].get_value();

        pitch_ = - std::asin(state_interfaces_[1].get_value() / 9.8);

        if (mode_ == "DEPTH") {
            command_interfaces_[0].set_value(params_.thruster_velocity);
            command_interfaces_[1].set_value(alpha);
            command_interfaces_[2].set_value(-alpha);
        }
        else if (mode_ == "IMMERSE") {
            command_interfaces_[0].set_value(requested_velocity_);
            command_interfaces_[1].set_value(-alpha);
            command_interfaces_[2].set_value(alpha);
        }
        else {
            command_interfaces_[0].set_value(0.);
            command_interfaces_[1].set_value(0.);
            command_interfaces_[2].set_value(0.);
        }

        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse DepthController::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal) {
        std::lock_guard<std::mutex> lock_(depth_mutex_);
        requested_depth_ = goal->requested_depth;
        duration_ = goal->duration;
        RCLCPP_INFO(get_node()->get_logger(), "Action: d=%fm, d=%fs", goal->requested_depth, goal->duration);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DepthController::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void DepthController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Handle accepted");
        std::thread{std::bind(&DepthController::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void DepthController::execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Executing goal");

        rclcpp::Rate loop_rate(10);

        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Action::Feedback>();
        auto result = std::make_shared<Action::Result>();

        mode_ = "DEPTH";
        reached_flag_ = false;
        starting_time_ = get_node()->get_clock()->now().seconds();

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock_(depth_mutex_);

                // Variables getting
                depth_error_ = current_depth_ - requested_depth_;
                feedback->depth_error = depth_error_;

                // Command creating
		        double thetab = (pitch_ - K_inf_ * std::atan(depth_error_ / r_) * 2. / M_PI);
                alpha = K_fin_ * std::atan(2 *std::atan(std::tan(thetab/2)) / r_fin_) * 2. / M_PI;

                RCLCPP_INFO(get_node()->get_logger(), "pitch=%f pitch_error=%f alpha=%f", pitch_, thetab, alpha);

                // Time
                double elapsed_time = get_node()->get_clock()->now().seconds() - starting_time_;
                feedback->elapsed_time = elapsed_time;

                // Check if the goal is canceled
                if (goal_handle->is_canceling()) {
                    mode_ = "IDLE";
                    result_->depth = current_depth_;
                    result->elapsed_time = elapsed_time;
                    goal_handle->canceled(result_);
                    RCLCPP_INFO(get_node()->get_logger(), "Goal canceled");
                }

                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(get_node()->get_logger(), "Goal Duration: %f, %f", requested_depth_, duration_);
                RCLCPP_INFO(get_node()->get_logger(), "Publish Feedback: [%f, %f]", depth_error_, elapsed_time);

                // Check if the goal is depth validated
                // TODO put 0.25 as parameter
                if (std::abs(feedback->depth_error) < 0.25 && !reached_flag_) {
                    reaching_time_ = get_node()->get_clock()->now().seconds();
                    reached_flag_ = true;
                }

                // Check duration
                if (goal->duration < elapsed_time) {
                    mode_ = "IDLE";
                    result_->depth = current_depth_;
                    result->elapsed_time = elapsed_time;

                    if (reached_flag_) {
                        goal_handle->succeed(result_);
                    }
                    else {
                        goal_handle->abort(result_);
                    }
                    break;
                }
            } // mutex scope

            // Loop rate
            loop_rate.sleep();
        }
    }




    rclcpp_action::GoalResponse DepthController::immerse_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ImmerseAction::Goal> goal) {
        std::lock_guard<std::mutex> lock_(immerse_mutex_);
        requested_duration_ = goal->duration;
        requested_depth_ = goal->depth;
        requested_velocity_ = goal->velocity;
        requested_pitch_ = goal->pitch;
        RCLCPP_INFO(get_node()->get_logger(), "Action Immersion: d=%fm, t=%fs, v=%f, a=%f", requested_depth_, duration_, requested_velocity_, requested_pitch_);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DepthController::immerse_handle_cancel(const std::shared_ptr<ImmerseGoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void DepthController::immerse_handle_accepted(const std::shared_ptr<ImmerseGoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Handle accepted");
        std::thread{std::bind(&DepthController::immerse_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void DepthController::immerse_execute(const std::shared_ptr<ImmerseGoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Executing goal");

        rclcpp::Rate loop_rate(10);

        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ImmerseAction::Feedback>();
        auto result = std::make_shared<ImmerseAction::Result>();

        mode_ = "IMMERSE";
        reached_flag_ = false;
        starting_time_ = get_node()->get_clock()->now().seconds();

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock_(depth_mutex_);

                // Variables getting
                depth_error_ = current_depth_ - requested_depth_;
                feedback->depth_error = depth_error_;

                // Command creating
                double thetab = (pitch_ - K_inf_ * std::atan(depth_error_ / r_) * 2. / M_PI);
                alpha = K_fin_ * 2 *std::atan(std::tan(thetab/2));

                // Time
                double elapsed_time = get_node()->get_clock()->now().seconds() - starting_time_;
                feedback->remaining_time = elapsed_time;

                // Check if the goal is canceled
                if (goal_handle->is_canceling()) {
                    mode_ = "IDLE";
                    immerse_result_->final_depth = current_depth_;
                    immerse_result_->final_duration = elapsed_time;
                    goal_handle->canceled(immerse_result_);
                    RCLCPP_INFO(get_node()->get_logger(), "Goal canceled");
                }

                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(get_node()->get_logger(), "Goal Duration: %f, %f", requested_depth_, duration_);
                RCLCPP_INFO(get_node()->get_logger(), "Publish Feedback: [%f, %f]", depth_error_, elapsed_time);

                // Check if the goal is depth validated
                // TODO put 0.25 as parameter
                if (std::abs(feedback->depth_error) < 0.25 && !reached_flag_) {
                    reaching_time_ = get_node()->get_clock()->now().seconds();
                    reached_flag_ = true;
                }

                // Check duration
                if (goal->duration < elapsed_time) {
                    mode_ = "IDLE";
                    immerse_result_->final_depth = current_depth_;
                    result->final_duration = elapsed_time;

                    if (reached_flag_) {
                        goal_handle->succeed(immerse_result_);
                    }
                    else {
                        goal_handle->abort(immerse_result_);
                    }
                    break;
                }
            } // mutex scope

            // Loop rate
            loop_rate.sleep();
        }
    }

} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::DepthController, controller_interface::ControllerInterface)
