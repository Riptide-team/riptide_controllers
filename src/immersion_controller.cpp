#include "riptide_controllers/immersion_controller.hpp"
#include "immersion_controller_parameters.hpp"
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

    ImmersionController::ImmersionController() : controller_interface::ControllerInterface(), immersion_start_time_(0, 0, rcl_clock_type_t::RCL_ROS_TIME) {}

    controller_interface::CallbackReturn ImmersionController::on_init() {
        try {
            param_listener_ = std::make_shared<immersion_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImmersionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        if (params_.thruster_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'thruster_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
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

        // Init phase_1_duration
        RCLCPP_INFO(get_node()->get_logger(), "Phase 1 duration: %f", params_.phase_1_duration);
        phase_1_duration_ = std::make_unique<rclcpp::Duration>(std::chrono::nanoseconds(static_cast<std::int64_t>(params_.phase_1_duration * 1e9)));

        // Init phase_2_duration
        RCLCPP_INFO(get_node()->get_logger(), "Phase 2 duration: %f", params_.phase_2_duration);
        phase_2_duration_ = std::make_unique<rclcpp::Duration>(std::chrono::nanoseconds(static_cast<std::int64_t>(params_.phase_2_duration * 1e9)));
        
        // Init phase_3_duration
        RCLCPP_INFO(get_node()->get_logger(), "Phase 3 duration: %f", params_.phase_3_duration);
        phase_3_duration_ = std::make_unique<rclcpp::Duration>(std::chrono::nanoseconds(static_cast<std::int64_t>(params_.phase_3_duration * 1e9)));

        // Init goal handle
        goal_handle_ = nullptr;

        // Init Depth action
        action_server_ = rclcpp_action::create_server<Action>(
            get_node(),
            "~/immerse",
            std::bind(&ImmersionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ImmersionController::handle_cancel, this, std::placeholders::_1),
            std::bind(&ImmersionController::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ImmersionController::command_interface_configuration() const {
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

        command_interfaces_config.names.push_back(prefix + params_.thruster_joint + "/velocity");
        command_interfaces_config.names.push_back(prefix + params_.d_joint + "/position");
        command_interfaces_config.names.push_back(prefix + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration ImmersionController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn ImmersionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        goal_handle_ = nullptr;

        // Setting null commands
        command_interfaces_[0].set_value(0.);
        command_interfaces_[1].set_value(0.);
        command_interfaces_[2].set_value(0.);
        command_interfaces_[3].set_value(0.);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImmersionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        goal_handle_ = nullptr;

        // Setting quiet_nan commands
        command_interfaces_[0].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[1].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[2].set_value(std::numeric_limits<double>::quiet_NaN());
        command_interfaces_[3].set_value(std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ImmersionController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        if (param_listener_->is_old(params_)) {
            param_listener_->refresh_dynamic_parameters();
            params_ = param_listener_->get_params();
        }
            
        std::scoped_lock lock(goal_mutex_);

        if (goal_handle_ != nullptr) {

            // Check if the goal is canceled
            if (goal_handle_->is_canceling()) {

                // Publishing result
                auto result = std::make_shared<Action::Result>();
                result->final_duration = time - immersion_start_time_;
                goal_handle_->canceled(result);

                // Publishing null commands
                command_interfaces_[0].set_value(0.);
                command_interfaces_[1].set_value(0.);
                command_interfaces_[2].set_value(0.);
                command_interfaces_[3].set_value(0.);

                return controller_interface::return_type::OK;
            }

            if (goal_handle_->is_executing()) {

                rclcpp::Duration time_since_immersion = time - immersion_start_time_;

                // Publish feedback
                auto feedback = std::make_shared<Action::Feedback>();
                feedback->remaining_time = std::max(rclcpp::Duration(0, 0), *phase_1_duration_ + *phase_2_duration_ + *phase_3_duration_ - time_since_immersion);
                goal_handle_->publish_feedback(feedback);
                
                // Phase 1
                if (time_since_immersion < *phase_1_duration_) {
                    // Publishing null commands
                    command_interfaces_[0].set_value(params_.thruster_velocity);
                    command_interfaces_[1].set_value(0.);
                    command_interfaces_[2].set_value(0.);
                    command_interfaces_[3].set_value(0.);
                }

                // Phase 2
                else if (time_since_immersion < *phase_1_duration_ + *phase_2_duration_) {
                    command_interfaces_[0].set_value(params_.thruster_velocity);
                    command_interfaces_[1].set_value(0.);

                    // double angle = params_.fin_angle * std::exp(- std::pow(((time_since_immersion - *phase_1_duration_).seconds() - phase_2_duration_->seconds() / 2) / (phase_2_duration_->seconds() / 4), 2));
                    double t = (time_since_immersion - *phase_1_duration_).seconds();
                    double angle = - params_.fin_angle * (t - phase_2_duration_->seconds()/2) * std::exp(- std::pow((t - phase_2_duration_->seconds()/2), 2) / phase_2_duration_->seconds()/2);
                    command_interfaces_[2].set_value(-angle);
                    command_interfaces_[3].set_value(angle);
                }

                // // Phase 2
                // else if (time_since_immersion < *phase_1_duration_ + *phase_2_duration_) {
                //     command_interfaces_[0].set_value(params_.thruster_velocity);
                //     command_interfaces_[1].set_value(0.);

                //     double angle = params_.fin_angle * (1 - std::exp(- std::pow((time_since_immersion - *phase_1_duration_).seconds() / phase_2_duration_->seconds(), 2)));
                //     command_interfaces_[2].set_value(-angle);
                //     command_interfaces_[3].set_value(angle);
                // }

                // // Phase 3
                // else if (time_since_immersion < *phase_1_duration_ + *phase_2_duration_ + *phase_3_duration_) {
                //     command_interfaces_[0].set_value(params_.thruster_velocity);
                //     command_interfaces_[1].set_value(0.);

                //     double angle = params_.fin_angle * 2 * (std::exp(- std::pow(2 * (time_since_immersion - *phase_1_duration_ - *phase_2_duration_).seconds() / phase_3_duration_->seconds(), 2)) - 0.5);
                //     command_interfaces_[2].set_value(-angle);
                //     command_interfaces_[3].set_value(angle);
                // }

                else {
                    // Publishing result
                    auto result = std::make_shared<Action::Result>();
                    result->final_duration = time_since_immersion;
                    goal_handle_->succeed(result);

                    // Publishing null commands
                    command_interfaces_[0].set_value(0.);
                    command_interfaces_[1].set_value(0.);
                    command_interfaces_[2].set_value(0.);
                    command_interfaces_[3].set_value(0.);
                }

                return controller_interface::return_type::OK;
            }
        }

        // Publishing null commands
        command_interfaces_[0].set_value(0.);
        command_interfaces_[1].set_value(0.);
        command_interfaces_[2].set_value(0.);
        command_interfaces_[3].set_value(0.);
        return controller_interface::return_type::OK;
    }

    rclcpp_action::GoalResponse ImmersionController::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> /*goal*/) {
        // Reject goal if the controller is not active
        if (get_node()->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ImmersionController::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ImmersionController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_node()->get_logger(), "Handle accepted");
        std::scoped_lock lock(goal_mutex_);
        goal_handle_ = goal_handle;
        immersion_start_time_ = get_node()->get_clock()->now();
    }
    
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::ImmersionController, controller_interface::ControllerInterface)