#include "riptide_controllers/riptide_controller.hpp"
#include "riptide_controller_parameters.hpp"
#include "controller_interface/chainable_controller_interface.hpp"


#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <string>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

namespace riptide_controllers {

    RiptideController::RiptideController() :
        controller_interface::ChainableControllerInterface(), inv_B(make_inv_B()),
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

        // Resizing reference interface to size 4 (1 linear velocity, 3 angular velocities)
        reference_interfaces_.resize(4, std::numeric_limits<double>::quiet_NaN());

        // Configuring controller state publisher
        controller_state_publisher_ = get_node()->create_publisher<ControllerStateType>("~/controller_state", rclcpp::SystemDefaultsQoS());
        rt_controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateType>>(controller_state_publisher_);

        // Creating the control subscriber for non-chained mode
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

        // Adding prefix if specified
        std::string prefix;
        if (params_.prefix.empty()) {
            prefix = "";
        }
        else {
            prefix = params_.prefix + "_";
        }

        command_interfaces_config.names.push_back(prefix + params_.thruster_joint + "/velocity");
        command_interfaces_config.names.push_back(prefix + params_.d_joint + "/position");
        command_interfaces_config.names.push_back(prefix + params_.p_joint + "/position");
        command_interfaces_config.names.push_back(prefix + params_.s_joint + "/position");
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::vector<std::string> coords = {"x", "y", "z"};

        // Adding prefix if specified
        std::string prefix;
        if (params_.prefix.empty()) {
            prefix = "";
        }
        else {
            prefix = params_.prefix + "_";
        }

        for (const auto &c: coords) {
            state_interfaces_config.names.push_back(prefix + params_.imu_name + "/angular_velocity." + c);
        }
        return state_interfaces_config;
    }

    std::vector<hardware_interface::CommandInterface> RiptideController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "linear_velocity.x", &reference_interfaces_[0]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "angular_velocity.x", &reference_interfaces_[1]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "angular_velocity.y", &reference_interfaces_[2]));
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "angular_velocity.z", &reference_interfaces_[3]));
        return reference_interfaces;
    }

    controller_interface::CallbackReturn RiptideController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        // Setting reference values to 0.
        for (std::size_t i=0; i<4; ++i) {
            reference_interfaces_[i] = 0.;
        }

        // Setting control values to 0.
        for (std::size_t i=0; i<4; ++i) {
            command_interfaces_[i].set_value(0.);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

        // Setting reference values to quiet_NaN.
        for (std::size_t i=0; i<4; ++i) {
            reference_interfaces_[i] = std::numeric_limits<double>::quiet_NaN();
        }

        // Setting control values to quiet_NaN.
        for (std::size_t i=0; i<4; ++i) {
            command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideController::update_reference_from_subscribers() {

        // Getting the twist command
        auto twist_command_msg = rt_command_ptr_.readFromRT();

        // no command received yet
        if (!twist_command_msg || !(*twist_command_msg)) {
            return controller_interface::return_type::OK;
        }

        // Store time of the received message
        last_received_command_time = rclcpp::Time((*twist_command_msg)->header.stamp);
       
        // Getting linear velocity
        reference_interfaces_[0] = (*twist_command_msg)->twist.linear.x;

        // Getting angular velocity
        reference_interfaces_[1] = (*twist_command_msg)->twist.angular.x;
        reference_interfaces_[2] = (*twist_command_msg)->twist.angular.y;
        reference_interfaces_[3] = (*twist_command_msg)->twist.angular.z;

        return controller_interface::return_type::OK;
    }

    bool RiptideController::on_set_chained_mode(bool chained_mode) {
        // we can set chained mode in any situation
        (void)chained_mode;
        return true;
    }

    controller_interface::return_type RiptideController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {

        // Checking in chained mode if the message is expired 
        if (is_in_chained_mode() && (time - last_received_command_time).nanoseconds() * 1e-9 > params_.command_timeout) {
            for (std::size_t i=0; i<4; ++i) {
                command_interfaces_[i].set_value(0.);
            }

            RCLCPP_DEBUG(get_node()->get_logger(), "Time difference: %f", (time - last_received_command_time).nanoseconds() * 1e-9);
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 5000, "No Twist received, publishing null control!");                        
            return controller_interface::return_type::OK;
        }

        // Getting the twist command
        wc_(0) = reference_interfaces_[1];
        wc_(1) = reference_interfaces_[2];
        wc_(2) = reference_interfaces_[3];

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

        RCLCPP_DEBUG(get_node()->get_logger(), "Rceived %f / %f %f %f", reference_interfaces_[0], reference_interfaces_[1], reference_interfaces_[2], reference_interfaces_[3]);

        // Generating command
        double v = 1.;
        u_ = 1. / v * inv_B * w_;

        for (double &u: u_) {
            u = params_.K_fin * std::atan(u / params_.r_fin);
        }

        // Generating the command
        command_interfaces_[0].set_value(reference_interfaces_[0]);
        command_interfaces_[1].set_value(u_(0));
        command_interfaces_[2].set_value(u_(1));
        command_interfaces_[3].set_value(u_(2));

        RCLCPP_DEBUG(get_node()->get_logger(), "Publishing %f %f %f %f", reference_interfaces_[0], u_(0), u_(1), u_(2));

        // Publishing controller state
        rt_controller_state_publisher_->lock();
        rt_controller_state_publisher_->msg_.header.stamp = time;
        rt_controller_state_publisher_->msg_.reference.linear.x = reference_interfaces_[0];
        rt_controller_state_publisher_->msg_.reference.angular.x = reference_interfaces_[1];
        rt_controller_state_publisher_->msg_.reference.angular.y = reference_interfaces_[2];
        rt_controller_state_publisher_->msg_.reference.angular.z = reference_interfaces_[3];
        rt_controller_state_publisher_->msg_.thruster_joint = command_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.d_joint = command_interfaces_[1].get_value();
        rt_controller_state_publisher_->msg_.p_joint = command_interfaces_[2].get_value();
        rt_controller_state_publisher_->msg_.s_joint = command_interfaces_[3].get_value();
        rt_controller_state_publisher_->unlockAndPublish();
        
        return controller_interface::return_type::OK;
    }
} // riptide_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_controllers::RiptideController, controller_interface::ChainableControllerInterface)