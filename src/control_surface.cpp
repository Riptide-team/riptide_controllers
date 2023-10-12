#include "riptide_controllers/control_surface.hpp"
#include "control_surface_parameters.hpp"
#include "controller_interface/chainable_controller_interface.hpp"


#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/duration.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

namespace riptide_lionel {

    ControlSurface::ControlSurface() :
        controller_interface::ChainableControllerInterface(), rt_reference_ptr_(nullptr), reference_subscriber_(nullptr) {}

    controller_interface::CallbackReturn ControlSurface::on_init() {
        try {
            param_listener_ = std::make_shared<control_surface::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ControlSurface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();

        // Resizing reference interface to size 1
        reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());

        // Configuring controller state publisher
        controller_state_publisher_ = get_node()->create_publisher<ControllerStateType>("~/controller_state", rclcpp::SystemDefaultsQoS());
        rt_controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateType>>(controller_state_publisher_);

        RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ControlSurface::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names.push_back(params_.joint_name + "/" + params_.interface_name);
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration ControlSurface::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return state_interfaces_config;
    }

    std::vector<hardware_interface::CommandInterface> ControlSurface::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), "force", &reference_interfaces_[0]));
        return reference_interfaces;
    }

    controller_interface::CallbackReturn ControlSurface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer if a command came through callback when controller was inactive
        rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

        // Setting reference value to 0.
        reference_interfaces_[0] = 0.;

        // Setting control value to 0.
        command_interfaces_[0].set_value(0.);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ControlSurface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

        // Setting reference values to quiet_NaN.
        reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();

        // Setting control values to quiet_NaN.
        command_interfaces_[0].set_value(std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ControlSurface::update_reference_from_subscribers() {

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

    bool ControlSurface::on_set_chained_mode(bool chained_mode) {
        // we can set chained mode in any situation
        (void)chained_mode;
        return true;
    }

    controller_interface::return_type ControlSurface::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        // Fluid velocity to be updated with propeller force
        double v = 1;
        double u = 2 * reference_interfaces_[0] / (params_.s * params_.cla * params_.rho * std::pow(v, 2));

        // Generating the command
        command_interfaces_[0].set_value(u);

        // Publishing controller state
        rt_controller_state_publisher_->lock();
        rt_controller_state_publisher_->msg_.header.stamp = time;
        rt_controller_state_publisher_->msg_.reference = reference_interfaces_[0];
        rt_controller_state_publisher_->msg_.output = u;
        rt_controller_state_publisher_->unlockAndPublish();
        
        return controller_interface::return_type::OK;
    }
} // riptide_lionel

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_lionel::ControlSurface, controller_interface::ChainableControllerInterface)