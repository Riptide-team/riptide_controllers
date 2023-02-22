#include "riptide_controllers/broadcasters/riptide_battery_card_broadcaster.hpp"
#include "riptide_battery_card_broadcaster_parameters.hpp"

#include "controller_interface/controller_interface.hpp"

#include <string>
#include <vector>
#include <sensor_msgs/msg/battery_state.hpp>


namespace riptide_broadcasters {

    controller_interface::CallbackReturn BatteryCardBroadcaster::on_init() {
        try {
            param_listener_ = std::make_shared<battery_card_broadcaster::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BatteryCardBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();
        if (params_.sensor_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        
        // Publisher
        try {
            // register ft sensor data publisher
            battery_card_publisher_ = get_node()->create_publisher<Msg>("~/battery_status", rclcpp::SystemDefaultsQoS());
            realtime_battery_card_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Msg>>(battery_card_publisher_);
        }
        catch (const std::exception & e) {
            RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message : %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Filling header of the message
        realtime_battery_card_publisher_->lock();
        realtime_battery_card_publisher_->msg_.header.frame_id = "base_link";
        realtime_battery_card_publisher_->msg_.design_capacity = 12;
        realtime_battery_card_publisher_->msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        realtime_battery_card_publisher_->msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        realtime_battery_card_publisher_->msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        realtime_battery_card_publisher_->msg_.present = true;
        realtime_battery_card_publisher_->unlock();

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BatteryCardBroadcaster::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration BatteryCardBroadcaster::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        std::vector<std::string> iface = {"tension", "current"};
        for (const auto &i: iface) {
            state_interfaces_config.names.push_back(prefix + "_" + params_.sensor_name + "/" + i);
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn BatteryCardBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BatteryCardBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type BatteryCardBroadcaster::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        if (realtime_battery_card_publisher_ && realtime_battery_card_publisher_->trylock()) {
            // Header
            realtime_battery_card_publisher_->msg_.header.stamp = get_node()->now();

            // Data
            realtime_battery_card_publisher_->msg_.voltage = state_interfaces_[0].get_value();
            realtime_battery_card_publisher_->msg_.current = state_interfaces_[1].get_value();
            realtime_battery_card_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
} // riptide_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_broadcasters::BatteryCardBroadcaster, controller_interface::ControllerInterface)