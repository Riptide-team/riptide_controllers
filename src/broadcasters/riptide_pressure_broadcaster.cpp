#include "riptide_controllers/broadcasters/riptide_pressure_broadcaster.hpp"
#include "riptide_pressure_broadcaster_parameters.hpp"

#include "controller_interface/controller_interface.hpp"

#include <string>
#include <vector>

#include "riptide_msgs/msg/pressure.hpp"


namespace riptide_broadcasters {

    controller_interface::CallbackReturn PressureBroadcaster::on_init() {
        try {
            param_listener_ = std::make_shared<pressure_broadcaster::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PressureBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();
        if (params_.sensor_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        
        // Publisher
        pressure_publisher_ = get_node()->create_publisher<Msg>("~/pressure_status", rclcpp::SystemDefaultsQoS());
        realtime_pressure_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Msg>>(pressure_publisher_);

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PressureBroadcaster::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration PressureBroadcaster::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        std::vector<std::string> iface = {"pressure", "temperature", "depth", "altitude"};
        for (const auto &i: iface) {
            state_interfaces_config.names.push_back(prefix + "_" + params_.sensor_name + "/" + i);
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn PressureBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PressureBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PressureBroadcaster::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        if (realtime_pressure_publisher_->trylock()) {
            // Getting the message
            auto pressure_message = realtime_pressure_publisher_->msg_;

            // Header
            pressure_message.header.stamp = get_node()->now();
            pressure_message.header.frame_id = "base_link";

            // Data
            pressure_message.pressure = state_interfaces_[0].get_value();
            pressure_message.temperature = state_interfaces_[1].get_value();
            pressure_message.depth = state_interfaces_[2].get_value();
            pressure_message.altitude = state_interfaces_[3].get_value();
            realtime_pressure_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
} // riptide_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_broadcasters::PressureBroadcaster, controller_interface::ControllerInterface)