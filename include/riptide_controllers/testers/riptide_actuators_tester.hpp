#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_actuators_tester_parameters.hpp"
#include <string>

namespace riptide_testers {

    class RiptideActuatorsTester : public controller_interface::ControllerInterface {
        public:

            RiptideActuatorsTester();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_actuators_tester::ParamListener> param_listener_;
            riptide_actuators_tester::Params params_;

            // Time since beginning
            double time_;

            // Period coefficient
            double period_;

            // Exponential decrease cefficient
            double damping_;

            // Thruster amplitude
            double amplitude_;

            // Thruster duration
            double duration_;
    };

} // riptide_testers