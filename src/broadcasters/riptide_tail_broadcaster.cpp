#include "riptide_controllers/broadcasters/riptide_tail_broadcaster.hpp"
#include "riptide_tail_broadcaster_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <riptide_msgs/msg/actuators.hpp>
#include <riptide_msgs/msg/multiplexer.hpp>

#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace riptide_broadcasters {

    TailBroadcaster::TailBroadcaster() :
        controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn TailBroadcaster::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_tail_broadcaster::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TailBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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

        // Actuators publisher
        try {
            actuators_publisher_ = get_node()->create_publisher<ActuatorMsg>(params_.actuators_topic_name.c_str(), rclcpp::SystemDefaultsQoS());
            realtime_actuators_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<ActuatorMsg>>(actuators_publisher_);
            RCLCPP_INFO(get_node()->get_logger(), "Publishing status on %s", params_.actuators_topic_name.c_str());
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s", e.what());
            return CallbackReturn::ERROR;
        }

        // RC publisher
        try {
            // register ft sensor data publisher
            rc_publisher_ = get_node()->create_publisher<RCMsg>(params_.rc_topic_name.c_str(), rclcpp::SystemDefaultsQoS());
            realtime_rc_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<RCMsg>>(rc_publisher_);
            RCLCPP_INFO(get_node()->get_logger(), "Publishing status on %s", params_.rc_topic_name.c_str());
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Resizing the Joy axes array
        realtime_rc_publisher_->msg_.axes.resize(params_.rc_channels.size());

         // Multiplexer publisher
        try {
            // register ft sensor data publisher
            multiplexer_publisher_ = get_node()->create_publisher<MultiplexerMsg>(params_.multiplexer_topic_name.c_str(), rclcpp::SystemDefaultsQoS());
            realtime_multiplexer_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<MultiplexerMsg>>(multiplexer_publisher_);
            RCLCPP_INFO(get_node()->get_logger(), "Publishing status on %s", params_.multiplexer_topic_name.c_str());
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s", e.what());
            return CallbackReturn::ERROR;
        }
        
        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration TailBroadcaster::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration TailBroadcaster::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Actuators interfaces
        state_interfaces_config.names.push_back(prefix + "_" + params_.thruster_joint + "/position");
        state_interfaces_config.names.push_back(prefix + "_" + params_.d_joint + "/position");
        state_interfaces_config.names.push_back(prefix + "_" + params_.p_joint + "/position");
        state_interfaces_config.names.push_back(prefix + "_" + params_.s_joint + "/position");

        // RC interfaces
        for (std::string rc_channel: params_.rc_channels) {
            state_interfaces_config.names.push_back(prefix + "_" + params_.rc_name + "/" + rc_channel);
        }

        // Multiplexer interfaces
        state_interfaces_config.names.push_back(prefix + "_" + params_.multiplexer_name + "/control");
        state_interfaces_config.names.push_back(prefix + "_" + params_.multiplexer_name + "/remaining_time");

        return state_interfaces_config;
    }

    controller_interface::CallbackReturn TailBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TailBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type TailBroadcaster::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        if (realtime_actuators_publisher_ && realtime_actuators_publisher_->trylock()) {
            // Header
            realtime_actuators_publisher_->msg_.header.stamp = get_node()->now();

            // Data
            realtime_actuators_publisher_->msg_.thruster = state_interfaces_[0].get_value();
            realtime_actuators_publisher_->msg_.d_fin = state_interfaces_[1].get_value();
            realtime_actuators_publisher_->msg_.p_fin = state_interfaces_[2].get_value();
            realtime_actuators_publisher_->msg_.s_fin = state_interfaces_[3].get_value();

            RCLCPP_DEBUG(get_node()->get_logger(), "%f %f %f %f", state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(), state_interfaces_[3].get_value());
            realtime_actuators_publisher_->unlockAndPublish();
        }

        if (realtime_rc_publisher_ && realtime_rc_publisher_->trylock()) {
            // Header
            realtime_rc_publisher_->msg_.header.stamp = get_node()->now();

            // Data
            for (std::size_t i=0; i<(realtime_rc_publisher_->msg_).axes.size(); ++i) {
                realtime_rc_publisher_->msg_.axes[i] = state_interfaces_[i+4].get_value();
            }
            realtime_rc_publisher_->unlockAndPublish();
        }

        if (realtime_multiplexer_publisher_ && realtime_multiplexer_publisher_->trylock()) {
            // Header
            realtime_multiplexer_publisher_->msg_.header.stamp = get_node()->now();

            // Data
            realtime_multiplexer_publisher_->msg_.automatic = (state_interfaces_[10].get_value()>0.5 ? true : false);
            realtime_multiplexer_publisher_->msg_.remaining_time = state_interfaces_[11].get_value();
            realtime_multiplexer_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
} // riptide_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_broadcasters::TailBroadcaster, controller_interface::ControllerInterface)