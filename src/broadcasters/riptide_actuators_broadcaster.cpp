#include "riptide_controllers/testers/riptide_actuators_broadcaster.hpp"
#include "riptide_actuators_broadcaster_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <riptide_msgs/msg/Actuators.msg>

#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace riptide_broadcaster {

    RiptideActuatorsTester::RiptideActuatorsTester() :
        controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn RiptideActuatorsTester::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_actuators_broadcaster::ParamListener>(get_node());
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

        // Publisher
        try {
            // register ft sensor data publisher
            actuators_publisher_ = get_node()->create_publisher<Msg>(params_.topic.c_str(), rclcpp::SystemDefaultsQoS());
            realtime_actuators_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Msg>>(actuators_publisher_);
            RCLCPP_INFO(get_node()->get_logger(), "Publishing status on %s", params_.topic.c_str());
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s", e.what());
            return CallbackReturn::ERROR;
        }
        
        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideActuatorsTester::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideActuatorsTester::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names.push_back(prefix + "_" + params_.thruster_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.d_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.s_joint + "/position");
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn RiptideActuatorsTester::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideActuatorsTester::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideActuatorsTester::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        if (realtime_actuators_publisher_ && realtime_actuators_publisher_->trylock()) {
            // Header
            realtime_actuators_publisher_->msg_.header.stamp = get_node()->now();

            // Data
            realtime_actuators_publisher_->msg_.thruster = state_interfaces_[0].get_value();
            realtime_actuators_publisher_->msg_.d_fin = state_interfaces_[1].get_value();
            realtime_actuators_publisher_->msg_.p_fin = state_interfaces_[2].get_value();
            realtime_actuators_publisher_->msg_.s_fin = state_interfaces_[3].get_value();
            realtime_actuators_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
} // riptide_testers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_testers::RiptideActuatorsTester, controller_interface::ControllerInterface)