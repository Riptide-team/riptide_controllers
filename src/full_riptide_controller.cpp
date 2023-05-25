#include "riptide_controllers/full_riptide_controller.hpp"
#include "full_riptide_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>


namespace riptide_controllers {

    controller_interface::CallbackReturn FullDepthController::on_init() {
        try {
            param_listener_ = std::make_shared<full_riptide_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FullDepthController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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
            std::bind(&FullDepthController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FullDepthController::handle_cancel, this, std::placeholders::_1),
            std::bind(&FullDepthController::handle_accepted, this, std::placeholders::_1)
        );

        K_inf_ = params_.K_inf;
        K_fin_ = params_.K_fin;
        r_fin_ = params_.Range_fin;
        r_ = params_.r;

       running_ = false;

       R_ = Eigen::Matrix3d::Identity();
       Rw_ = Eigen::Matrix3d::Identity();

        RCLCPP_INFO(get_node()->get_logger(), "Parameters: %f, %f, %f %f", params_.thruster_velocity, K_inf_, K_fin_, r_);

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration FullDepthController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        command_interfaces_config.names.push_back(prefix + "_" + params_.thruster_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.d_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration FullDepthController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);

        // Getting depth
        state_interfaces_config.names.push_back(prefix + "_" + params_.pressure_name + "/depth");

        // Getting quaternion
        std::vector<std::string> ifaces = {"orientation"};
        std::vector<std::string> coords = {"x", "y", "z", "w"};
        for (const auto &iface: ifaces) {
            for (const auto &c: coords) {
                state_interfaces_config.names.push_back(prefix + "_" + params_.imu_name + "/" + iface + "." + c);
            }
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn FullDepthController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(0);
        command_interfaces_[2].set_value(0);
        command_interfaces_[3].set_value(0);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FullDepthController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(0);
        command_interfaces_[2].set_value(0);
        command_interfaces_[3].set_value(0);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type FullDepthController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Appliying commands when running
        if (running_) {
            std::lock_guard<std::mutex> lock_(depth_mutex_);

            // Depth storage
            current_depth_ = state_interfaces_[0].get_value();

            // Rotation matrix storage
            Eigen::Quaterniond q;
            q.x() = state_interfaces_[1].get_value();
            q.y() = state_interfaces_[2].get_value();
            q.z() = state_interfaces_[3].get_value();
            q.w() = state_interfaces_[4].get_value();
            R_ = q.normalized().toRotationMatrix();

            // Computing the desired rotation matrix Rw_
            double pitch_w =  K_inf_ * std::atan((requested_depth_ - current_depth_) / r_) * 2. / M_PI;

            // Wanted rotation matrix computation
            Eigen::AngleAxisd rollAngle(requested_roll_, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd yawAngle(pitch_w, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd pitchAngle(requested_yaw_, Eigen::Vector3d::UnitZ());

            Eigen::Quaternion<double> qr = yawAngle * pitchAngle * rollAngle;
            Rw_ = qr.toRotationMatrix();

            // Rotation matrix desired to be applied on the Riptide
            Eigen::Vector3d w_ = SkewInv((R_.transpose() * Rw_).log());

            w_(0) = 0.;
            w_(1) = 0.;
            w_(2) = 1.;

            // Fin's angle computation
            double v_ = 1.0;
            Eigen::Vector3d u_ = 1. / v_ * inv_B * w_;

            // Applying saturated commands
            command_interfaces_[0].set_value(params_.thruster_velocity);
            command_interfaces_[1].set_value(K_fin_ * std::atan( u_(0) / r_fin_) * 2. / M_PI);
            command_interfaces_[2].set_value(K_fin_ * std::atan( u_(1) / r_fin_) * 2. / M_PI);
            command_interfaces_[3].set_value(K_fin_ * std::atan( u_(2) / r_fin_) * 2. / M_PI);

            RCLCPP_DEBUG(get_node()->get_logger(), "Angles %f, %f, %f", K_fin_ * std::atan( u_(0) / r_fin_) * 2. / M_PI, K_fin_ * std::atan( u_(1) / r_fin_) * 2. / M_PI, K_fin_ * std::atan( u_(2) / r_fin_) * 2. / M_PI);
        }
        else {
            command_interfaces_[0].set_value(0.);
            command_interfaces_[1].set_value(0.);
            command_interfaces_[2].set_value(0.);
            command_interfaces_[3].set_value(0.);
        }

        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse FullDepthController::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal) {
        RCLCPP_INFO(get_node()->get_logger(), "Action: d=%fm, d=%fs", goal->depth, goal->duration);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FullDepthController::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FullDepthController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Handle accepted");
        std::thread{std::bind(&FullDepthController::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void FullDepthController::execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Executing goal");

        rclcpp::Rate loop_rate(10);

        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Action::Feedback>();
        auto result = std::make_shared<Action::Result>();

        running_ = true;
        reached_flag_ = false;
        starting_time_ = get_node()->get_clock()->now().seconds();

        requested_roll_ = goal->roll;
        requested_yaw_ = goal->yaw;
        requested_depth_ = goal->depth;

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock_(depth_mutex_);

                // Feedback message fill
                feedback->remaining_time = starting_time_ + goal->duration - get_node()->get_clock()->now().seconds();
                depth_error_ = current_depth_ - requested_depth_;
                feedback->depth_error = depth_error_;
                Eigen::Vector3d ea = R_.eulerAngles(2, 1, 0);
                feedback->yaw_error = ea(0);
                feedback->pitch_error = ea(1);
                feedback->roll_error = ea(2);

                // Check if the goal is canceled
                if (goal_handle->is_canceling()) {
                    running_ = false;
                    result_->final_depth = current_depth_;
                    result->final_duration = get_node()->get_clock()->now().seconds() - starting_time_;
                    goal_handle->canceled(result_);
                    RCLCPP_INFO(get_node()->get_logger(), "Goal canceled");
                }

                // Publish feedback
                goal_handle->publish_feedback(feedback);

                // Check if the goal is depth validated
                // TODO put 0.25 as parameter
                if (std::abs(feedback->depth_error) < 0.25 && !reached_flag_) {
                    reaching_time_ = get_node()->get_clock()->now().seconds();
                    reached_flag_ = true;
                }

                // Check duration
                if (feedback->remaining_time < 0.) {
                    running_ = false;
                    result_->final_depth = current_depth_;
                    result->final_duration = get_node()->get_clock()->now().seconds() - starting_time_;
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

} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::FullDepthController, controller_interface::ControllerInterface)
