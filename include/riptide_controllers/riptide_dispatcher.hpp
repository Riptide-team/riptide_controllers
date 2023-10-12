#pragma once

#include "controller_interface/chainable_controller_interface.hpp"
#include "control_surface_parameters.hpp"
#include <string>

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/wrench.hpp"
#include "riptide_msgs/msg/riptide_dispatcher_state.hpp"

#include <eigen3/Eigen/Dense>

namespace riptide_lionel {

    class RiptideDispatcher : public controller_interface::ChainableControllerInterface {
        public:
            using ControllerStateType = riptide_msgs::msg::RiptideDispatcherState;
            using ReferenceType = geometry_msgs::msg::Wrench;

            RiptideDispatcher();

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
            std::shared_ptr<riptide_dispatcher::ParamListener> param_listener_;
            riptide_dispatcher::Params params_;

            // Concentrator matrix
            Eigen::MatrixX<double, 5, 3> C_;

            // Dispatcher matrix
            Eigen::MatrixX<double, 3, 5> D_;

            // Reference subscriber
            realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>> rt_reference_ptr_;
            rclcpp::Subscription<ReferenceType>::SharedPtr reference_subscriber_;

            // Controller state publisher
            rclcpp::Publisher<ControllerStateType>::SharedPtr controller_state_publisher_;
            std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateType>> rt_controller_state_publisher_;
    };

} // riptide_lionel