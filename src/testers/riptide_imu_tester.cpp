#include "riptide_controllers/testers/riptide_imu_tester.hpp"
#include "riptide_imu_tester_parameters.hpp"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/qos.hpp"

#include "realtime_tools/realtime_buffer.h"

#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace riptide_testers {

    RiptideIMUTester::RiptideIMUTester() :
        controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn RiptideIMUTester::on_init() {
        try {
            param_listener_ = std::make_shared<riptide_imu_tester::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(
            get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideIMUTester::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        params_ = param_listener_->get_params();
        if (params_.sensor_name.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Params sensor name %s", params_.sensor_name.c_str());

        RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RiptideIMUTester::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration RiptideIMUTester::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        std::string prefix = std::string(get_node()->get_namespace()).substr(1);
        std::vector<std::string> iface = {"linear_acceleration", "angular_velocity"};
        std::vector<std::string> coords = {"x", "y", "z"};
        for (const auto &i: iface) {
            for (const auto &c: coords) {
                state_interfaces_config.names.push_back(prefix + "_" + params_.sensor_name + "/" + i + "." + c);
            }
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn RiptideIMUTester::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RiptideIMUTester::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RiptideIMUTester::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(
            get_node()->get_logger(), "[%f]s a=[%f, %f, %f], w=[%f, %f, %f]", time.seconds(),
            state_interfaces_[0].get_value(), state_interfaces_[1].get_value(), state_interfaces_[2].get_value(),
            state_interfaces_[3].get_value(), state_interfaces_[4].get_value(), state_interfaces_[5].get_value()
        );
        return controller_interface::return_type::OK;
    }
} // riptide_testers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_testers::RiptideIMUTester, controller_interface::ControllerInterface)