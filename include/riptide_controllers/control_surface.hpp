#pragma once

#include "controller_interface/chainable_controller_interface.hpp"
#include "control_surface_parameters.hpp"
#include <string>

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include "riptide_msgs/msg/control_surface_state.hpp"

#include <eigen3/Eigen/Dense>

namespace riptide_lionel {

    class ControlSurface : public controller_interface::ChainableControllerInterface {
        public:
            using ControllerStateType = riptide_msgs::msg::ControlSurfaceState;
            using ReferenceType = std_msgs::msg::Float64;

            ControlSurface();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            // Chained controller specific methods
            controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

            bool on_set_chained_mode(bool chained_mode) override;

            controller_interface::return_type update_reference_from_subscribers() override;

        private:
            std::shared_ptr<control_surface::ParamListener> param_listener_;
            control_surface::Params params_;

            // Reference subscriber
            realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>> rt_reference_ptr_;
            rclcpp::Subscription<ReferenceType>::SharedPtr reference_subscriber_;

            // Controller state publisher
            rclcpp::Publisher<ControllerStateType>::SharedPtr controller_state_publisher_;
            std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateType>> rt_controller_state_publisher_;
    };

} // riptide_lionel