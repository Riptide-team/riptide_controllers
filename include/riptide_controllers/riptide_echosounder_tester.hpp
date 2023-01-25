#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_echosounder_tester_parameters.hpp"
#include <string>

namespace riptide_controllers {

    class RiptideEchosounderTester : public controller_interface::ControllerInterface {
        public:

            RiptideEchosounderTester();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_echosounder_tester::ParamListener> param_listener_;
            riptide_echosounder_tester::Params params_;
    };

} // riptide_controllers