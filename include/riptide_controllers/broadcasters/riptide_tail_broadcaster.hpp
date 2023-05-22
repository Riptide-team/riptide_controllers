#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_tail_broadcaster_parameters.hpp"
#include <string>

#include <riptide_msgs/msg/actuators.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <realtime_tools/realtime_publisher.h>


namespace riptide_broadcasters {

    class TailBroadcaster : public controller_interface::ControllerInterface {
        public:

            using ActuatorMsg = riptide_msgs::msg::Actuators;
            using RCMsg = sensor_msgs::msg::Joy;

            TailBroadcaster();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_tail_broadcaster::ParamListener> param_listener_;
            riptide_tail_broadcaster::Params params_;

            // Actuators publisher
            std::shared_ptr<rclcpp::Publisher<ActuatorMsg>> actuators_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<ActuatorMsg>> realtime_actuators_publisher_ = nullptr;

            // RC publisher
            std::shared_ptr<rclcpp::Publisher<RCMsg>> rc_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<RCMsg>> realtime_rc_publisher_ = nullptr;
    };

} // riptide_broadcasters