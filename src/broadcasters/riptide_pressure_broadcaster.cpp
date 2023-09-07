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
        try {
            // register ft sensor data publisher
            pressure_publisher_ = get_node()->create_publisher<Msg>("~/pressure_status", rclcpp::SystemDefaultsQoS());
            realtime_pressure_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Msg>>(pressure_publisher_);
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Filling header of the message
        realtime_pressure_publisher_->lock();
        realtime_pressure_publisher_->msg_.header.frame_id = "base_link";
        realtime_pressure_publisher_->unlock();

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

        // Adding prefix if specified
        std::string prefix;
        if (params_.prefix.empty()) {
            prefix = "";
        }
        else {
            prefix = params_.prefix + "_";
        }

        std::vector<std::string> iface = {"pressure", "temperature", "depth", "altitude"};
        for (const auto &i: iface) {
            state_interfaces_config.names.push_back(prefix + params_.sensor_name + "/" + i);
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

        if (realtime_pressure_publisher_ && realtime_pressure_publisher_->trylock()) {
            // Header
            realtime_pressure_publisher_->msg_.header.stamp = get_node()->now();

            // Data
            realtime_pressure_publisher_->msg_.pressure = state_interfaces_[0].get_value();
            realtime_pressure_publisher_->msg_.temperature = state_interfaces_[1].get_value();
            realtime_pressure_publisher_->msg_.depth = state_interfaces_[2].get_value();
            realtime_pressure_publisher_->msg_.altitude = state_interfaces_[3].get_value();
            realtime_pressure_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
} // riptide_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_broadcasters::PressureBroadcaster, controller_interface::ControllerInterface)