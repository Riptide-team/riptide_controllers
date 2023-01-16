#pragma once

#include "controller_interface/controller_interface.hpp"
#include "state_estimator_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "realtime_tools/realtime_publisher.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <string>
#include <memory>

namespace riptide_controllers {
    /// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
    template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived> & vec) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    class StateEstimator : public controller_interface::ControllerInterface {
        public:
            StateEstimator() {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);

        private:
            std::shared_ptr<state_estimator::ParamListener> param_listener_;
            state_estimator::Params params_;

            // Inputs
            // Linear accelerations
            Eigen::Vector3d a_;

            // Angular velocities
            Eigen::Vector3d w_;

            // Outputs
            // Position
            Eigen::Vector3d p_;

            // Velocity
            Eigen::Vector3d v_;

            // Rotation matrix
            Eigen::Matrix3d R_;

            // Tf broadcaster
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            // Tf base link message
            std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_;

            // Twist publisher
            std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> twist_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> realtime_twist_publisher_ = nullptr;

            // // Tf d fin message
            // std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_d;

            // // Tf p fin message
            // std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_p;

            // // Tf s fin message
            // std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_s;

            // // Tf thruster message
            // std::shared_ptr<geometry_msgs::msg::TransformStamped> tf_t;
    };
} // riptide_controllers