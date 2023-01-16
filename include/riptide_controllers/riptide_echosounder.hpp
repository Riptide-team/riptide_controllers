#pragma once

#include "controller_interface/controller_interface.hpp"
#include "riptide_echosounder_parameters.hpp"
#include <string>

#include <sensor_msgs/msg/range.hpp>
#include "realtime_tools/realtime_publisher.h"


#include <eigen3/Eigen/Dense>

namespace riptide_controllers {

    class RiptideEchoSounder : public controller_interface::ControllerInterface {
        public:
            using RangeType = sensor_msgs::msg::Range;

            RiptideEchoSounder() {};

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::shared_ptr<riptide_echosounder::ParamListener> param_listener_;
            riptide_echosounder::Params params_;

            std::shared_ptr<rclcpp::Publisher<RangeType>> range_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<RangeType>> realtime_range_publisher_ = nullptr;
    };

} // riptide_controllers