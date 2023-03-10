#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_controller_parameters.hpp"
#include <string>

#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <eigen3/Eigen/Dense>

namespace riptide_controllers {

    class RiptideController : public controller_interface::ControllerInterface {
        public:
            using CmdType = geometry_msgs::msg::Twist;
            using FeedbackType = geometry_msgs::msg::TwistStamped;

            RiptideController();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_controller::ParamListener> param_listener_;
            riptide_controller::Params params_;

            // Wanted rotation velocity
            Eigen::Vector3d w_;

            // Wanted control vector
            Eigen::Vector3d u_;

            // Fin matrix
            const Eigen::Matrix3d inv_B;

            // Helper to construct B
            Eigen::Matrix3d make_inv_B() {
                return (Eigen::Matrix3d() << -5, -5, -5,
                                0, 10*std::sin(2*M_PI/3), -10*std::sin(2*M_PI/3),
                                10, 10*std::cos(2*M_PI/3), 10*std::cos(2*M_PI/3)
                        ).finished().inverse() ;
            };

            // cmd_vel subscriber
            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
            rclcpp::Subscription<CmdType>::SharedPtr twist_command_subscriber_;

            // feedback loop subscriber
            realtime_tools::RealtimeBuffer<std::shared_ptr<FeedbackType>> rt_feedback_ptr_;
            rclcpp::Subscription<FeedbackType>::SharedPtr twist_feedback_subscriber_;
    };

} // riptide_controllers