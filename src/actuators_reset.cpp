#include "riptide_controllers/actuators_reset.hpp"
#include "actuators_reset_parameters.hpp"
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

    ActuatorsReset::ActuatorsReset() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn ActuatorsReset::on_init() {
        try {
            param_listener_ = std::make_shared<actuators_reset::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ActuatorsReset::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ActuatorsReset::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        command_interfaces_config.names.push_back(prefix + "_" + params_.thruster_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration ActuatorsReset::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn ActuatorsReset::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ActuatorsReset::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ActuatorsReset::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        command_interfaces_[0].set_value(0);
        command_interfaces_[1].set_value(0);
        command_interfaces_[2].set_value(0);
        return controller_interface::return_type::OK;
    }

} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::ActuatorsReset, controller_interface::ControllerInterface)