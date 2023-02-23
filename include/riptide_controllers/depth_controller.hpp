#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>
#include "riptide_msgs/action/depth.hpp"
#include "depth_controller_parameters.hpp"

#include <memory>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <string>

namespace riptide_controllers {

    class DepthController : public controller_interface::ControllerInterface {

        public:

            using Action = riptide_msgs::action::Depth;
            using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

            DepthController() : controller_interface::ControllerInterface() {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            // Action server
            rclcpp_action::Server<riptide_msgs::action::Depth>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

            void execute(const std::shared_ptr<GoalHandle> goal_handle);

            // Depth control
            double alpha;
            double K_inf_;
            double K_fin_;
            double r_;

            std::mutex depth_mutex_;

            double duration_;
            double requested_depth_;

            double current_depth_;
            Eigen::Vector3d euler_angles_; // yaw = euler[0]; pitch = euler[1]; roll = euler[2]

            double depth_error_;

            double starting_time_;
            double reaching_time_;
            bool reached_flag_;

            bool running_;

            // Action handle
            std::shared_ptr<Action::Feedback> feedback_;
            std::shared_ptr<Action::Result> result_;

            // Parameters
            std::shared_ptr<depth_controller::ParamListener> param_listener_;
            depth_controller::Params params_;
    };

} // riptide_controllers