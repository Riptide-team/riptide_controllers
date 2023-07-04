#include "riptide_controllers/riptide_controller.hpp"
#include "riptide_controller_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

namespace riptide_controllers {

    RiptideController::RiptideController() :
        controller_interface::ControllerInterface(), inv_B(make_inv_B()),
        rt_command_ptr_(nullptr), twist_command_subscriber_(nullptr) {}

    controller_interface::CallbackReturn RiptideController::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();
        if (params_.d_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'d_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.p_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'p_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.s_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'s_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.thruster_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'thruster_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.imu_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'imu_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        twist_command_subscriber_ = get_node()->create_subscription<CmdType>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); }
        );

        last_received_command_time = get_node()->get_clock()->now();

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        std::string prefix = std::string(get_node()->get_namespace()).substr(1);

        command_interfaces_config.names.push_back(prefix + "_" + params_.thruster_joint + "/velocity");
        command_interfaces_config.names.push_back(prefix + "_" + params_.d_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + "_" + params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::vector<std::string> coords = {"x", "y", "z"};

        std::string prefix = std::string(get_node()->get_namespace()).substr(1);

        for (const auto &c: coords) {
            state_interfaces_config.names.push_back(prefix + "_" + params_.imu_name + "/angular_velocity." + c);
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn RiptideController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {

        if (time - last_received_command_time > rclcpp::Duration::from_seconds(params_.command_timeout)) {
            command_interfaces_[0].set_value(0.);
            command_interfaces_[1].set_value(0.);
            command_interfaces_[2].set_value(0.);
            command_interfaces_[3].set_value(0.);

            RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 3000, "No Twist received, publishing null control!");
            return controller_interface::return_type::OK;
        }


        // Getting the twist command
        auto twist_command = rt_command_ptr_.readFromRT();

        // no command received yet
        if (!twist_command || !(*twist_command)) {
            return controller_interface::return_type::OK;
        }

        // Getting the time of the received message
        last_received_command_time = (*twist_command)->header.stamp;

        // Getting the twist command
        wc_(0) = -(*twist_command)->twist.angular.x;
        wc_(1) = -(*twist_command)->twist.angular.y;
        wc_(2) = -(*twist_command)->twist.angular.z;

        // Getting actual twist
        Eigen::Vector3d wm_;
        wm_(0) = state_interfaces_[0].get_value();
        wm_(1) = state_interfaces_[1].get_value();
        wm_(2) = state_interfaces_[2].get_value();

        // TODO publish error on a topic ~/error of type float32
        // Computing angular velocity error
        // double K = 0.5;
        // w_ = w_ - K * period.seconds() * (wc_ - wm_);

        w_ = wc_;

        RCLCPP_DEBUG(get_node()->get_logger(), "Rceived %f / %f %f %f", (*twist_command)->twist.linear.x, (*twist_command)->twist.angular.x, (*twist_command)->twist.angular.y, (*twist_command)->twist.angular.z);

        // Generating command
        double v = 1.;
        u_ = 1. / v * inv_B * w_;

        double K_fin = M_PI / 4.;
        double r_fin = M_PI / 10.;
        for (double &u: u_) {
            u = K_fin * 2. / M_PI * std::atan(u / r_fin);
        }

        // Generating the command
        double u0 = (*twist_command)->twist.linear.x;
        command_interfaces_[0].set_value(u0);
        command_interfaces_[1].set_value(u_(0));
        command_interfaces_[2].set_value(u_(1));
        command_interfaces_[3].set_value(u_(2));

        RCLCPP_DEBUG(get_node()->get_logger(), "Publishing %f %f %f %f", u0, u_(0), u_(1), u_(2));
        
        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::RiptideController, controller_interface::ControllerInterface)