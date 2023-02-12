#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_battery_card_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <sensor_msgs/msg/battery_state.hpp>
#include "realtime_tools/realtime_publisher.h"

#include <string>
#include <memory>

namespace riptide_broadcasters {
    class BatteryCardBroadcaster : public controller_interface::ControllerInterface {
        public:

            using Msg = sensor_msgs::msg::BatteryState;

            BatteryCardBroadcaster() {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);

        private:
            std::shared_ptr<battery_card_broadcaster::ParamListener> param_listener_;
            battery_card_broadcaster::Params params_;

            // Pressure publisher
            std::shared_ptr<rclcpp::Publisher<Msg>> battery_card_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<Msg>> realtime_battery_card_publisher_ = nullptr;
    };
} // riptide_broadcasters