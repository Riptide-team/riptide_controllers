#include "riptide_controllers/riptide_echosounder.hpp"
#include "riptide_echosounder_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_publisher.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace riptide_controllers {

    controller_interface::CallbackReturn RiptideEchoSounder::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_echosounder::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideEchoSounder::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();
        if (params_.echosounder_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'echosounder_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        range_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Range>("~/range", rclcpp::SystemDefaultsQoS());
        realtime_range_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<RiptideEchoSounder::RangeType>>(range_publisher_);

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideEchoSounder::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideEchoSounder::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        state_interfaces_config.names.push_back(params_.echosounder_name + "/range");
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn RiptideEchoSounder::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideEchoSounder::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideEchoSounder::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Publishing realtime range
        if (realtime_range_publisher_->trylock()) {
            auto & range_message = realtime_range_publisher_->msg_;
            range_message.header.stamp = get_node()->now();
            range_message.header.frame_id = std::string(get_node()->get_namespace()) + "_echosounder";
            range_message.field_of_view = 0.1;
            range_message.range = state_interfaces_[0].get_value();
            range_message.min_range = 0.5;
            range_message.max_range = 60;
            realtime_range_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::RiptideEchoSounder, controller_interface::ControllerInterface)