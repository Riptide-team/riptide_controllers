#include "riptide_controllers/riptide_dispatcher.hpp"
#include "riptide_dispatcher_parameters.hpp"
#include "controller_interface/chainable_controller_interface.hpp"


#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <cmath>

namespace riptide_lionel {

    RiptideDispatcher::RiptideDispatcher() :
        controller_interface::ChainableControllerInterface(), rt_reference_ptr_(nullptr), reference_subscriber_(nullptr) {}

    controller_interface::CallbackReturn RiptideDispatcher::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_dispatcher::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideDispatcher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();

        if (params_.joint_names.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'joint' parameter is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (params_.interface_names.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'interfaces' parameter is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (params_.joint_names.size() != params_.interface_names.size()) {
            RCLCPP_ERROR(get_node()->get_logger(), "`joint` and `interfaces` parameters must have the same size");
        }

        // Resizing reference interface to size 6 (3 forces and 3 torques)
        reference_interfaces_.resize(6, std::numeric_limits<double>::quiet_NaN());

        // Configuring controller state publisher
        controller_state_publisher_ = get_node()->create_publisher<ControllerStateType>("~/controller_state", rclcpp::SystemDefaultsQoS());
        rt_controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateType>>(controller_state_publisher_);

        // Computing concentrator matrix
        double hd, hp, hs = 0.08;
        double ld, lp, ls = 0.5;
        C_ << -1, - std::sin(30 * M_PI / 180), std::sin(30 * M_PI / 180),
            0, -std::cos(30 * M_PI / 180), std::cos(30 * M_PI / 180),
            hd, dp, hs,
            0, lp, ls,
            ld, 0, 0;
        
        // Computing dispatcher matrix
        D_ = C_.completeOrthogonalDecomposition().pseudoInverse();

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideDispatcher::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (size_t i = 0; i < params_.joint_names.size(); ++i) {
            command_interfaces_config.names.push_back(params_.joint_names[i] + "/" + params_.interface_names[i]);
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideDispatcher::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interfaces_config;
    }

    std::vector<hardware_interface::CommandInterface> RiptideDispatcher::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "force", &reference_interfaces_[0]));
        return reference_interfaces;
    }

    controller_interface::CallbackReturn RiptideDispatcher::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

        // Setting reference value to 0.
        for (auto & reference_interface : reference_interfaces_) {
            reference_interface = 0.;
        }

        // Setting control value to 0.
        for (auto & command_interface : command_interfaces_) {
            command_interface.set_value(0.);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideDispatcher::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

        // Setting reference values to quiet_NaN.
        reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();

        // Setting control values to quiet_NaN.
        command_interfaces_[0].set_value(std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideDispatcher::update_reference_from_subscribers() {

        // Getting the reference
        auto reference_msg = rt_reference_ptr_.readFromRT();

        // no command received yet
        if (!reference_msg || !(*reference_msg)) {
            return controller_interface::return_type::OK;
        }

        // Getting requested force velocity
        reference_interfaces_[0] = (*reference_msg)->data;

        return controller_interface::return_type::OK;
    }

    bool RiptideDispatcher::on_set_chained_mode(bool chained_mode) {
        // we can set chained mode in any situation
        (void)chained_mode;
        return true;
    }

    controller_interface::return_type RiptideDispatcher::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        // Getting the reference
        Eigen::Vector5d F;
        F << reference_interfaces_[1], reference_interfaces_[2], reference_interfaces_[3], reference_interfaces_[4], reference_interfaces_[5];

        // Computing requested force by actuators
        Eigen::Vector3d u = D_ * F;

        // Applying commands
        command_interfaces_[0].set_value(reference_interfaces_[0]);
        for (size_t i = 1; i < params_.joint_names.size(); ++i) {
            command_interfaces_[i].set_value(u(i));
        }

        // Publishing controller state
        rt_controller_state_publisher_->lock();
        rt_controller_state_publisher_->msg_.header.stamp = time;
        rt_controller_state_publisher_->msg_.reference.force.x = reference_interfaces_[0];
        rt_controller_state_publisher_->msg_.reference.force.y = reference_interfaces_[1];
        rt_controller_state_publisher_->msg_.reference.force.z = reference_interfaces_[2];
        rt_controller_state_publisher_->msg_.reference.torque.x = reference_interfaces_[3];
        rt_controller_state_publisher_->msg_.reference.torque.y = reference_interfaces_[4];
        rt_controller_state_publisher_->msg_.reference.torque.z = reference_interfaces_[5];
        rt_controller_state_publisher_->msg_.thruster_force = command_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.d_force = command_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.p_force = command_interfaces_[0].get_value();
        rt_controller_state_publisher_->msg_.s_force = command_interfaces_[0].get_value();
        rt_controller_state_publisher_->unlockAndPublish();
        
        return controller_interface::return_type::OK;
    }
} // riptide_lionel

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_lionel::RiptideDispatcher, controller_interface::ChainableControllerInterface)