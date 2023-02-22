#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_imu_broadcaster_parameters.hpp"
#include <string>

#include <sensor_msgs/msg/imu.hpp>

namespace riptide_testers {

    class RiptideIMUBroadcaster : public controller_interface::ControllerInterface {
        public:

            using Msg = sensor_msgs::msg::Imu;

            RiptideIMUBroadcaster();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_imu_broadcaster::ParamListener> param_listener_;
            riptide_imu_broadcaster::Params params_;

            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

            // Pressure publisher
            std::shared_ptr<rclcpp::Publisher<Msg>> imu_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<Msg>> realtime_imu_publisher_ = nullptr;
    };

} // riptide_testers