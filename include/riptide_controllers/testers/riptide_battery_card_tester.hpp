#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_battery_card_tester_parameters.hpp"
#include <string>

namespace riptide_testers {

    class RiptideBatteryCardTester : public controller_interface::ControllerInterface {
        public:

            RiptideBatteryCardTester();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_battery_card_tester::ParamListener> param_listener_;
            riptide_battery_card_tester::Params params_;
    };

} // riptide_testers