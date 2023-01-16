#include "riptide_controllers/state_estimator.hpp"
#include "state_estimator_parameters.hpp"

#include "controller_interface/controller_interface.hpp"

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/twist_stamped.hpp>


namespace riptide_controllers {

    controller_interface::CallbackReturn StateEstimator::on_init() {
        try {
            param_listener_ = std::make_shared<state_estimator::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn StateEstimator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();
        if (params_.sensor_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        if (params_.frame_id.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter has to be provided.");
            return CallbackReturn::ERROR;
        }

        // Initialize input vectors
        a_ = Eigen::Vector3d::Zero();
        w_ = Eigen::Vector3d::Zero();

        // Initialize output matrixes
        std::vector<double> p = params_.position;
        p_ << p[0], p[1], p[2];
        std::vector<double> v = params_.velocity;
        v_ << v[0], v[1], v[2];

        std::vector<double> o = params_.orientation;
        Eigen::Quaterniond q;
        q.x() = o[0];
        q.y() = o[1];
        q.z() = o[2];
        q.w() = o[3];
        R_ = q.normalized().toRotationMatrix();

        // Initialize tf message
        tf_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
        tf_->header.frame_id = "world";
        tf_->child_frame_id = params_.frame_id;

        // Initialize tf broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

        twist_publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", rclcpp::SystemDefaultsQoS());
        realtime_twist_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(twist_publisher_);

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration StateEstimator::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;

        // std::string prefix = "tf_broadcaster/joint1";
        // std::vector<std::string> iface = {"position", "orientation"};
        // std::vector<std::string> coords = {"x", "y", "z"};
        // for (const auto &i: iface) {
        //     for (const auto &c: coords) {
        //         command_interfaces_config.names.push_back(prefix + "/" + i + "." + c);
        //     }
        // }
        // command_interfaces_config.names.push_back(prefix + "/orientation.w");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration StateEstimator::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::vector<std::string> iface = {"linear_acceleration", "angular_velocity"};
        std::vector<std::string> coords = {"x", "y", "z"};
        for (const auto &i: iface) {
            for (const auto &c: coords) {
                state_interfaces_config.names.push_back(params_.sensor_name + "/" + i + "." + c);
            }
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn StateEstimator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn StateEstimator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type StateEstimator::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
        // Time step
        double dt = period.seconds();

        // Linear acceleration reading
        a_(0) = state_interfaces_[0].get_value();
        a_(1) = state_interfaces_[1].get_value();
        a_(2) = state_interfaces_[2].get_value();

        // Angular velocities reading
        w_(0) = state_interfaces_[3].get_value();
        w_(1) = state_interfaces_[4].get_value();
        w_(2) = state_interfaces_[5].get_value();

        // Euler Integration
        p_ += dt * R_ * v_;
        v_ += dt * (R_.transpose() * (Eigen::Vector3d() << 0, 0, -9.8).finished() + a_ - w_.cross(v_));
        R_ = R_ * (Skew(dt * w_)).exp();

        if (false) {
            // Filling the tf message
            tf_->header.stamp = get_node()->get_clock()->now();
            tf_->transform.translation.x = p_(0);
            tf_->transform.translation.y = p_(1);
            tf_->transform.translation.z = p_(2);

            // Get quaternion from rotation matrix
            Eigen::Quaterniond q(R_);
            tf_->transform.rotation.x = q.x();
            tf_->transform.rotation.y = q.y();
            tf_->transform.rotation.z = q.z();
            tf_->transform.rotation.w = q.w();

            // Sending the tf message
            tf_broadcaster_->sendTransform(*tf_);

            RCLCPP_DEBUG(get_node()->get_logger(), "%f %f %f %f %f %f %f",
            tf_->transform.translation.x, tf_->transform.translation.y, tf_->transform.translation.z,
            tf_->transform.rotation.x, tf_->transform.rotation.y, tf_->transform.rotation.z, tf_->transform.rotation.w);
        }

        if (realtime_twist_publisher_->trylock()) {
            auto & twist_message = realtime_twist_publisher_->msg_;
            twist_message.header.stamp = get_node()->now();
            twist_message.header.frame_id = "base_link";
            twist_message.twist.linear.x = v_(0);
            twist_message.twist.linear.y = v_(1);
            twist_message.twist.linear.z = v_(2);
            twist_message.twist.angular.x = w_(0);
            twist_message.twist.angular.y = w_(1);
            twist_message.twist.angular.z = w_(2);
            realtime_twist_publisher_->unlockAndPublish();
        }

        // RCLCPP_INFO(get_node()->get_logger(), "%f %f %f",
        // w_(0), w_(1), w_(2));

        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::StateEstimator, controller_interface::ControllerInterface)