#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>
#include "riptide_msgs/action/depth.hpp"
#include "depth_controller_parameters.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace riptide_controllers {

    class DepthController : public controller_interface::ControllerInterface {
        public:

            using Depth = riptide_msgs::action::Depth;
            using GoalHandleDepth = rclcpp_action::ServerGoalHandle<Depth>;

            DepthController();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<depth_controller::ParamListener> param_listener_;
            depth_controller::Params params_;

            rclcpp_action::Server<riptide_msgs::action::Depth>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Depth::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDepth> goal_handle);

            void execute(const std::shared_ptr<GoalHandleDepth> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandleDepth> goal_handle);

            std::mutex depth_mutex_;

            double error_;
            double requested_depth_;
            double current_depth_;

            // Action handle
            std::shared_ptr<Depth::Feedback> feedback_;
            std::shared_ptr<Depth::Result> result_;
    };

} // riptide_controllers