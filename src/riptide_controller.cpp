#include "riptide_controllers/riptide_controller.hpp"
#include "riptide_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

namespace riptide_controllers {

    RiptideController::RiptideController() :
        controller_interface::ControllerInterface(), inv_B(make_inv_B()),
        rt_command_ptr_(nullptr), twist_command_subscriber_(nullptr) {}

    controller_interface::CallbackReturn RiptideController::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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
        if (params_.imu_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'imu_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        twist_command_subscriber_ = get_node()->create_subscription<CmdType>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); }
        );

        twist_feedback_subscriber_ = get_node()->create_subscription<FeedbackType>(
            std::string(get_node()->get_namespace()) + "/state_estimator/twist", rclcpp::SystemDefaultsQoS(),
            [this](const FeedbackType::SharedPtr msg) { rt_feedback_ptr_.writeFromNonRT(msg); }
        );

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names.push_back(params_.thruster_joint + "/velocity");
        command_interfaces_config.names.push_back(params_.d_joint + "/position");
        command_interfaces_config.names.push_back(params_.p_joint + "/position");
        command_interfaces_config.names.push_back(params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::vector<std::string> coords = {"x", "y", "z"};

        for (const auto &c: coords) {
            state_interfaces_config.names.push_back(params_.imu_name + "/angular_velocity." + c);
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn RiptideController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Getting the twist command
        auto twist_command = rt_command_ptr_.readFromRT();

        // no command received yet
        if (!twist_command || !(*twist_command)) {
            return controller_interface::return_type::OK;
        }

        // Getting the twist feedback
        auto twist_feedback = rt_feedback_ptr_.readFromRT();

        // no command received yet
        if (!twist_feedback || !(*twist_feedback)) {
            return controller_interface::return_type::OK;
        }

        w_(0) = (*twist_command)->angular.x;
        w_(1) = (*twist_command)->angular.y;
        w_(2) = (*twist_command)->angular.z;

        // Generating command
        double v = 1.;
        u_ = 1. / v * inv_B * w_;

        double Kp = 1.;
        double u0 = 120. * M_PI * Kp * ((*twist_command)->linear.x - v);
        double u0 = (*twist_feedback)->twist.linear.x;
        command_interfaces_[0].set_value(u0);
        command_interfaces_[1].set_value(u_(0));
        command_interfaces_[2].set_value(u_(1));
        command_interfaces_[3].set_value(u_(2));
        
        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::RiptideController, controller_interface::ControllerInterface)