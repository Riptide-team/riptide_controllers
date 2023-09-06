#pragma once

#include "controller_interface/controller_interface.hpp"
#include "immersion_controller_parameters.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs/action/immerse.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <riptide_msgs/msg/immersion_controller_state.hpp>

#include <string>


namespace riptide_controllers {

    class ImmersionController : public controller_interface::ControllerInterface {
        public:

            using ControllerStateType = riptide_msgs::msg::ImmersionControllerState;
            using Action = riptide_msgs::action::Immerse;
            using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

            ImmersionController();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            // Parameters
            std::shared_ptr<immersion_controller::ParamListener> param_listener_;
            immersion_controller::Params params_;

            // Action server
            rclcpp_action::Server<Action>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

            void execute(const std::shared_ptr<GoalHandle> goal_handle);

            // Goal mutex
            std::mutex goal_mutex_;

            // Goal handle
            std::shared_ptr<GoalHandle> goal_handle_;

            // Current time
            rclcpp::Time current_time_;

            // Immersion start time
            rclcpp::Time immersion_start_time_;

            // Phase 1 duration
            std::unique_ptr<rclcpp::Duration> phase_1_duration_;

            // Phase 2 duration
            std::unique_ptr<rclcpp::Duration> phase_2_duration_;
    };

} // riptide_controllers