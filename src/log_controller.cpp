#include "riptide_controllers/log_controller.hpp"
#include "log_controller_parameters.hpp"
#include "controller_interface/chainable_controller_interface.hpp"


#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "riptide_msgs/msg/log_controller_state.hpp"

#include <string>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

namespace riptide_controllers {

    LogController::LogController() :
        controller_interface::ChainableControllerInterface(),
        rt_command_ptr_(nullptr), quaternion_command_subscriber_(nullptr) {}

    controller_interface::CallbackReturn LogController::on_init() {
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

    controller_interface::CallbackReturn LogController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        if (params_.linear_velocity_x_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'linear_velocity_x_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.angular_velocity_x_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'angular_velocity_x_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.angular_velocity_y_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'angular_velocity_y_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.angular_velocity_z_joint.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'angular_velocity_z_joint' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }
        if (params_.imu_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'imu_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        // Resizing reference_interfaces to 5 -> one linear velocity, 4 orientation quaternion
        reference_interfaces_.resize(8, std::numeric_limits<double>::quiet_NaN());

        // Configuring controller state publisher
        controller_state_publisher_ = get_node()->create_publisher<ControllerStateType>("~/controller_state", rclcpp::SystemDefaultsQoS());
        rt_controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateType>>(controller_state_publisher_);

        // Creating the control subscriber for non-chained mode
        vel_command_subscriber_ = get_node()->create_subscription<CmdVelType>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const CmdVelType::SharedPtr msg) { rt_vel_command_ptr_.writeFromNonRT(msg); }
        );

        // Creating the control subscriber for non-chained mode
        quaternion_command_subscriber_ = get_node()->create_subscription<CmdType>(
            "~/desired_orientation", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); }
        );

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration LogController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names.push_back(params_.linear_velocity_x_joint);
        command_interfaces_config.names.push_back(params_.angular_velocity_x_joint);
        command_interfaces_config.names.push_back(params_.angular_velocity_y_joint);
        command_interfaces_config.names.push_back(params_.angular_velocity_z_joint);
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration LogController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::vector<std::string> coords = {"x", "y", "z", "w"};

        // Adding prefix if specified
        std::string prefix;
        if (params_.prefix.empty()) {
            prefix = "";
        }
        else {
            prefix = params_.prefix + "_";
        }

        for (const auto &c: coords) {
            state_interfaces_config.names.push_back(prefix + params_.imu_name + "/orientation." + c);
        }
        return state_interfaces_config;
    }

    std::vector<hardware_interface::CommandInterface> LogController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "linear_velocity.x", &reference_interfaces_[0]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "orientation.x", &reference_interfaces_[1]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "orientation.y", &reference_interfaces_[2]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "orientation.z", &reference_interfaces_[3]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "orientation.w", &reference_interfaces_[4]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "angular_velocity.x", &reference_interfaces_[5]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "angular_velocity.y", &reference_interfaces_[6]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "angular_velocity.z", &reference_interfaces_[7]));
        return reference_interfaces;
    }

    controller_interface::CallbackReturn LogController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        reference_interfaces_[0] = 0;
        reference_interfaces_[1] = 0;
        reference_interfaces_[2] = 0;
        reference_interfaces_[3] = 0;
        reference_interfaces_[4] = 1;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LogController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
        reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
        reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();
        reference_interfaces_[3] = std::numeric_limits<double>::quiet_NaN();
        reference_interfaces_[4] = std::numeric_limits<double>::quiet_NaN();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type LogController::update_reference_from_subscribers() {

        // Getting the desired velocity command
        auto twist_desired_msg = rt_vel_command_ptr_.readFromRT();

        // no command received yet
        if (!twist_desired_msg || !(*twist_desired_msg)) {
            RCLCPP_DEBUG(get_node()->get_logger(), "No velocity command received");
        }
        else {
            reference_interfaces_[0] = (*twist_desired_msg)->linear.x;
        }

        // Getting the desired orientation command
        auto quaternion_desired_msg = rt_command_ptr_.readFromRT();

        // no command received yet
        if (!quaternion_desired_msg || !(*quaternion_desired_msg)) {
            RCLCPP_DEBUG(get_node()->get_logger(), "No orientation command received");
        }
        else {
            // Getting wanted orientation
            reference_interfaces_[1] = (*quaternion_desired_msg)->x;
            reference_interfaces_[2] = (*quaternion_desired_msg)->y;
            reference_interfaces_[3] = (*quaternion_desired_msg)->z;
            reference_interfaces_[4] = (*quaternion_desired_msg)->w;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type LogController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {

        // Getting wanted orientation
        Eigen::Quaterniond qw;
        qw.x() = reference_interfaces_[1];
        qw.y() = reference_interfaces_[2];
        qw.z() = reference_interfaces_[3];
        qw.w() = reference_interfaces_[4];

        RCLCPP_DEBUG(get_node()->get_logger(), "Desired quaternion: %f %f %f %f", qw.x(), qw.y(), qw.z(), qw.w());

        // Getting current orientation
        Eigen::Quaterniond q;
        q.x() = state_interfaces_[0].get_value();
        q.y() = state_interfaces_[1].get_value();
        q.z() = state_interfaces_[2].get_value();
        q.w() = state_interfaces_[3].get_value();

        RCLCPP_DEBUG(get_node()->get_logger(), "Current quaternion: %f %f %f %f", q.x(), q.y(), q.z(), q.w());

        // Log control coputing
        Eigen::Matrix3d Rr(q);
        Eigen::Matrix3d Rw(qw);
        Eigen::Vector3d w = Rr.transpose() * SkewInv((Rw * Rr.transpose()).log());

        // Setting the command
        command_interfaces_[0].set_value(reference_interfaces_[0]);
        command_interfaces_[1].set_value(params_.kp[0] * w(0) + params_.kd[0] * state_interfaces_[5].get_value());
        command_interfaces_[2].set_value(params_.kp[1] * w(1) + params_.kd[1] * state_interfaces_[6].get_value());
        command_interfaces_[3].set_value(params_.kp[2] * w(2) + params_.kd[2] * state_interfaces_[7].get_value());

        RCLCPP_DEBUG(get_node()->get_logger(), "Publishing %f %f %f %f", state_interfaces_[0].get_value(), w(0), w(1), w(2));

        rt_controller_state_publisher_->lock();
        rt_controller_state_publisher_->msg_.header.stamp = time;

        rt_controller_state_publisher_->msg_.reference.x = qw.x();
        rt_controller_state_publisher_->msg_.reference.y = qw.y();
        rt_controller_state_publisher_->msg_.reference.z = qw.z();
        rt_controller_state_publisher_->msg_.reference.w = qw.w();

        rt_controller_state_publisher_->msg_.feedback.x = q.x();
        rt_controller_state_publisher_->msg_.feedback.y = q.y();
        rt_controller_state_publisher_->msg_.feedback.z = q.z();
        rt_controller_state_publisher_->msg_.feedback.w = q.w();

        Eigen::Quaterniond error = q.inverse() * qw;
        rt_controller_state_publisher_->msg_.error.x = error.x();
        rt_controller_state_publisher_->msg_.error.y = error.y();
        rt_controller_state_publisher_->msg_.error.z = error.z();
        rt_controller_state_publisher_->msg_.error.w = error.w();
        rt_controller_state_publisher_->msg_.angular_error = 2 * std::atan2(error.vec().norm(), error.w());

        rt_controller_state_publisher_->msg_.velocity = command_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.output.x = command_interfaces_[1].get_value();
        rt_controller_state_publisher_->msg_.output.y = command_interfaces_[2].get_value();
        rt_controller_state_publisher_->msg_.output.z = command_interfaces_[3].get_value();

        rt_controller_state_publisher_->msg_.kp.x = params_.kp[0];
        rt_controller_state_publisher_->msg_.kp.y = params_.kp[1];
        rt_controller_state_publisher_->msg_.kp.z = params_.kp[2];

        rt_controller_state_publisher_->msg_.kp_error.x = params_.kp[0] * w(0);
        rt_controller_state_publisher_->msg_.kp_error.y = params_.kp[1] * w(1);
        rt_controller_state_publisher_->msg_.kp_error.z = params_.kp[2] * w(2);

        rt_controller_state_publisher_->msg_.kd.x = params_.kd[0];
        rt_controller_state_publisher_->msg_.kd.y = params_.kd[1];
        rt_controller_state_publisher_->msg_.kd.z = params_.kd[2];

        rt_controller_state_publisher_->msg_.kd_error.x = params_.kd[0] * state_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.kd_error.y = params_.kd[1] * state_interfaces_[1].get_value();
        rt_controller_state_publisher_->msg_.kd_error.z = params_.kd[2] * state_interfaces_[2].get_value();

        rt_controller_state_publisher_->unlockAndPublish();
        
        return controller_interface::return_type::OK;
    }

    bool LogController::on_set_chained_mode(bool chained_mode) {
        // we can set chained mode in any situation
        (void)chained_mode;
        return true;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::LogController, controller_interface::ChainableControllerInterface)