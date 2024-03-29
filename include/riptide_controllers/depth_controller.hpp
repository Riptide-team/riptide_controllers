#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>
#include "riptide_msgs/action/depth.hpp"
#include "riptide_msgs/action/immerse.hpp"
#include "riptide_msgs/msg/depth_controller_state.hpp"
#include "depth_controller_parameters.hpp"
#include "realtime_tools/realtime_publisher.h"

#include <memory>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <string>

namespace riptide_controllers {

    class DepthController : public controller_interface::ControllerInterface {

        public:

            using Action = riptide_msgs::action::Depth;
            using ControllerStateType = riptide_msgs::msg::DepthControllerState;
            using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

            DepthController() : controller_interface::ControllerInterface(), action_start_time_(0, 0, rcl_clock_type_t::RCL_ROS_TIME) {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            // Parameters
            std::shared_ptr<depth_controller::ParamListener> param_listener_;
            depth_controller::Params params_;

            // Starting time
            bool set_start_time_ = false;
            rclcpp::Time action_start_time_;

            // Depth mutex
            std::mutex goal_mutex_;

            // Goal handle
            std::shared_ptr<GoalHandle> goal_handle_;

            // Action server
            rclcpp_action::Server<riptide_msgs::action::Depth>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

            // Controller state publisher
            rclcpp::Publisher<ControllerStateType>::SharedPtr controller_state_publisher_;
            std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateType>> rt_controller_state_publisher_;

            

    };

} // riptide_controllers