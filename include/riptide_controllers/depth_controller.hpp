#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>
#include "riptide_msgs/action/depth.hpp"
#include "riptide_msgs/action/immerse.hpp"
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

            using ImmerseAction = riptide_msgs::action::Immersea;
            using ImmerseGoalHandle = rclcpp_action::ServerGoalHandle<ImmerseAction>;

            DepthController() : controller_interface::ControllerInterface() {};

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

            std::string mode_ = "IDLE";

            // Depth control
            double alpha;
            double K_inf_;
            double K_fin_;
            double r_fin_;
            double r_;

            std::mutex depth_mutex_;

            double duration_;
            double requested_depth_;

            double current_depth_;
            // Eigen::Vector3d euler_angles_; // yaw = euler[0]; pitch = euler[1]; roll = euler[2]
            double pitch_;

            double depth_error_;

            double starting_time_;
            double reaching_time_;
            bool reached_flag_;

            // bool running_;



            // Action server
            rclcpp_action::Server<riptide_msgs::action::Depth>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

            void execute(const std::shared_ptr<GoalHandle> goal_handle);

            // Action handle
            std::shared_ptr<Action::Feedback> feedback_;
            std::shared_ptr<Action::Result> result_;



            // Immerse Action server
            rclcpp_action::Server<riptide_msgs::action::Immerse>::SharedPtr immerse_action_server_;

            rclcpp_action::GoalResponse immerse_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ImmerseAction::Goal> goal);

            rclcpp_action::CancelResponse immerse_handle_cancel(const std::shared_ptr<ImmerseGoalHandle> goal_handle);

            void immerse_handle_accepted(const std::shared_ptr<ImmerseGoalHandle> goal_handle);

            void immerse_execute(const std::shared_ptr<ImmerseGoalHandle> goal_handle);

            // Immerse Action handle
            std::shared_ptr<ImmerseAction::Feedback> immerse_feedback_;
            std::shared_ptr<ImmerseAction::Result> immerse_result_;

            // Mutex for immersion
            std::mutex immerse_mutex_;

            // Immersion data
            double requested_duration_;
            // double requested_depth_;
            double requested_velocity_;
            double requested_pitch_;
    };

} // riptide_controllers