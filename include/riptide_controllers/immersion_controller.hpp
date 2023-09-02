#pragma once

#include "controller_interface/controller_interface.hpp"
#include "immersion_controller_parameters.hpp"
#include <string>

#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/quaternion.hpp"

#include <eigen3/Eigen/Dense>


namespace riptide_controllers {

    class ImmersionController : public controller_interface::ControllerInterface {
        public:

            using CmdType = geometry_msgs::msg::Quaternion;

            ImmersionController();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            std::shared_ptr<immersion_controller::ParamListener> param_listener_;
            immersion_controller::Params params_;

            // Wanted orientation
            Eigen::Quaterniond qw_;

            // Current orientation
            Eigen::Quaterniond q_;

            // Desired orientation subscriber
            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
            rclcpp::Subscription<CmdType>::SharedPtr quaternion_command_subscriber_;
    };

} // riptide_controllers