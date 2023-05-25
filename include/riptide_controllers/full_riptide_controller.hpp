#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include "full_riptide_controller_parameters.hpp"

#include <controller_interface/controller_interface.hpp>

#include "riptide_msgs/action/full_depth.hpp"

#include <memory>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <string>

namespace riptide_controllers {
    /// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
    template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived> & vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    template<class Derived>
    inline Eigen::Vector<typename Derived::Scalar, 3> SkewInv(const Eigen::MatrixBase<Derived> & mat) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
        return (Eigen::Vector<typename Derived::Scalar, 3>() << mat(2, 1), mat(0, 2), mat(1, 0)).finished();
    }

    class FullDepthController : public controller_interface::ControllerInterface {

        public:

            using Action = riptide_msgs::action::FullDepth;
            using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

            FullDepthController() : controller_interface::ControllerInterface(), inv_B(make_inv_B()) {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:

            // Fin matrix
            const Eigen::Matrix3d inv_B;

            // Helper to construct B
            Eigen::Matrix3d make_inv_B() {
                return (Eigen::Matrix3d() << -5, -5, -5,
                                0, 10*std::sin(M_PI/3), 10*std::sin(M_PI/3),
                                -10, 10*std::cos(M_PI/3), 10*std::cos(M_PI/3)
                        ).finished().inverse() ;
            };

            // Parameters
            std::shared_ptr<full_riptide_controller::ParamListener> param_listener_;
            full_riptide_controller::Params params_;

            bool running_;

            // Depth control
            double K_inf_;
            double K_fin_;
            double r_fin_;
            double r_;

            // Eigen::Vector3d euler_angles_; // yaw = euler[0]; pitch = euler[1]; roll = euler[2]
            double pitch_;

            double depth_error_;

            // Time variables
            double starting_time_;
            double reaching_time_;
            bool reached_flag_;

            // Current depth
            double current_depth_;

            // Rotation matrix
            Eigen::Matrix3d R_;

            // Wanted rotation matrix
            Eigen::Matrix3d Rw_;

            // Action server
            rclcpp_action::Server<Action>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

            void execute(const std::shared_ptr<GoalHandle> goal_handle);

            // Action handle
            std::shared_ptr<Action::Feedback> feedback_;
            std::shared_ptr<Action::Result> result_;

            // Data mutex
            std::mutex depth_mutex_;

            // FullDepth action data
            double requested_duration_;
            double requested_depth_;
            double requested_velocity_;
            double requested_yaw_;
            double requested_roll_;
    };

} // riptide_controllers