#include "riptide_controllers/riptide_actuators_tester.hpp"
#include "riptide_actuators_tester_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace riptide_controllers {

    RiptideActuatorsTester::RiptideActuatorsTester() :
        controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn RiptideActuatorsTester::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_actuators_tester::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideActuatorsTester::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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
        
        period_ = params_.period;
        damping_ = params_.damping;
        amplitude_ = params_.amplitude;
        duration_ = params_.duration;

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideActuatorsTester::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names.push_back(std::string(get_node()->get_namespace()) + "_" + params_.thruster_joint + "/position");
        command_interfaces_config.names.push_back(std::string(get_node()->get_namespace()) + "_" + params_.d_joint + "/position");
        command_interfaces_config.names.push_back(std::string(get_node()->get_namespace()) + "_" + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(std::string(get_node()->get_namespace()) + "_" + params_.s_joint + "/position");

        RCLCPP_INFO(get_logger()->get_namespace(), "Command interface: %s", (std::string(get_node()->get_namespace()) + "_" + params_.s_joint + "/position").c_str);
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideActuatorsTester::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn RiptideActuatorsTester::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideActuatorsTester::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideActuatorsTester::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
        time_ += period.seconds();

        // Thruster test
        double value;
        if (time_ < duration_) {
            value = amplitude_ * std::sin(2 * M_PI * time_ / duration_);
        }
        command_interfaces_[0].set_value(value);

        // Oscillating fins
        for (std::size_t i=1; i<4; ++i) {
            float u = M_PI_2 * std::sin(period_ * time_ + 2 * M_PI / 3 * (i - 1)) * (1 - std::exp(- time_ / damping_));
            command_interfaces_[i].set_value(u);
        }

        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::RiptideActuatorsTester, controller_interface::ControllerInterface)