#include "riptide_controllers/immersion_controller.hpp"
#include "immersion_controller_parameters.hpp"
#include "controller_interface/chainable_controller_interface.hpp"


#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <cmath>

namespace riptide_controllers {

    ImmersionController::ImmersionController() :
        controller_interface::ControllerInterface(),
        rt_command_ptr_(nullptr), quaternion_command_subscriber_(nullptr) {}

    controller_interface::CallbackReturn ImmersionController::on_init() {
        try {
            param_listener_ = std::make_shared<log_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImmersionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        if (params_.thruster_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'thruster_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
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
        if (params_.imu_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'imu_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.pressure_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'pressure_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ImmersionController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names.push_back(params_.thruster_joint);
        command_interfaces_config.names.push_back(params_.d_joint);
        command_interfaces_config.names.push_back(params_.p_joint);
        command_interfaces_config.names.push_back(params_.s_joint);
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration ImmersionController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::vector<std::string> coords = {"x", "y", "z", "w"};

        std::string prefix = std::string(get_node()->get_namespace()).substr(1);

        for (const auto &c: coords) {
            state_interfaces_config.names.push_back(params_.imu_name + "/orientation." + c);
        }

        state_interfaces_config.names.push_back(params_.pressure_name + "/depth");

        return state_interfaces_config;
    }

    controller_interface::CallbackReturn ImmersionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImmersionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ImmersionController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        // // Getting the desired orientation command
        // auto quaternion_desired_msg = rt_command_ptr_.readFromRT();

        // // no command received yet
        // if (!quaternion_desired_msg || !(*quaternion_desired_msg)) {
        //     return controller_interface::return_type::OK;
        // }

        // // Getting wanted orientation
        // qw_.x() = (*quaternion_desired_msg)->x;
        // qw_.y() = (*quaternion_desired_msg)->y;
        // qw_.z() = (*quaternion_desired_msg)->z;
        // qw_.w() = (*quaternion_desired_msg)->w;

        // RCLCPP_DEBUG(get_node()->get_logger(), "Desired quaternion: %f %f %f %f", qw_.x(), qw_.y(), qw_.z(), qw_.w());

        // // Getting current orientation
        // q_.x() = state_interfaces_[0].get_value();
        // q_.y() = state_interfaces_[1].get_value();
        // q_.z() = state_interfaces_[2].get_value();
        // q_.w() = state_interfaces_[3].get_value();

        // RCLCPP_DEBUG(get_node()->get_logger(), "Current quaternion: %f %f %f %f", q_.x(), q_.y(), q_.z(), q_.w());

        // // Log control coputing
        // Eigen::Matrix3d Rr(q_);
        // Eigen::Matrix3d Rw(qw_);
        // // TODO add skew inv
        // Eigen::Vector3d w = Rr.transpose() * SkewInv((Rw * Rr.transpose()).log());

        // // Generating the command
        // command_interfaces_[0].set_value(state_interfaces_[0].get_value());
        // command_interfaces_[1].set_value(w(0));
        // command_interfaces_[2].set_value(w(1));
        // command_interfaces_[3].set_value(w(2));

        RCLCPP_DEBUG(get_node()->get_logger(), "Publishing %f %f %f %f", state_interfaces_[0].get_value(), w(0), w(1), w(2));
        
        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::ImmersionController, controller_interface::ControllerInterface)