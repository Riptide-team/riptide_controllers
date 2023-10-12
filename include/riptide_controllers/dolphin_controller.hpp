#pragma once

#include "controller_interface/controller_interface.hpp"
#include "dolphin_controller_parameters.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs/action/immerse.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <riptide_msgs/msg/dolphin_controller_state.hpp>


namespace riptide_controllers {

    class DolphinController : public controller_interface::ControllerInterface {
        public:

            using ControllerStateType = riptide_msgs::msg::DolphinControllerState;

            DolphinController();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            // Parameters
            std::shared_ptr<dolphin_controller::ParamListener> param_listener_;
            dolphin_controller::Params params_;

            // Immersion start time
            rclcpp::Time dolphin_start_time_;

            // Controller state publisher
            rclcpp::Publisher<ControllerStateType>::SharedPtr controller_state_publisher_;
            std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateType>> rt_controller_state_publisher_;
    };

} // riptide_controllers