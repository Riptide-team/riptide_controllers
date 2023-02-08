#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_pressure_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "riptide_msgs/msg/pressure.hpp"
#include "realtime_tools/realtime_publisher.h"

#include <string>
#include <memory>

namespace riptide_broadcasters {
    class PressureBroadcaster : public controller_interface::ControllerInterface {
        public:

            using Msg = riptide_msgs::msg::Pressure;

            PressureBroadcaster() {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);

        private:
            std::shared_ptr<pressure_broadcaster::ParamListener> param_listener_;
            pressure_broadcaster::Params params_;

            // Pressure publisher
            std::shared_ptr<rclcpp::Publisher<Msg>> pressure_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<Msg>> realtime_pressure_publisher_ = nullptr;
    };
} // riptide_broadcasters