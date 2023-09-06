#pragma once

#include "controller_interface/controller_interface.hpp"
#include "log_controller_parameters.hpp"
#include <string>

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "riptide_msgs/msg/log_controller_state.hpp"

#include <eigen3/Eigen/Dense>


namespace riptide_controllers {
    
    template<class Derived>
    inline Eigen::Vector<typename Derived::Scalar, 3> SkewInv(const Eigen::MatrixBase<Derived> & mat) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
        return (Eigen::Vector<typename Derived::Scalar, 3>() << mat(2, 1), mat(0, 2), mat(1, 0)).finished();
    }

    class LogController : public controller_interface::ControllerInterface {
        public:

            using ControllerStateType = riptide_msgs::msg::LogControllerState;
            using CmdType = geometry_msgs::msg::Quaternion;

            LogController();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            std::shared_ptr<log_controller::ParamListener> param_listener_;
            log_controller::Params params_;

            // Wanted orientation
            Eigen::Quaterniond qw_;

            // Current orientation
            Eigen::Quaterniond q_;

            // Desired orientation subscriber
            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
            rclcpp::Subscription<CmdType>::SharedPtr quaternion_command_subscriber_;

            // Controller state publisher
            rclcpp::Publisher<ControllerStateType>::SharedPtr controller_state_publisher_;
            std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateType>> rt_controller_state_publisher_;
    };

} // riptide_controllers