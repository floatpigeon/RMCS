#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::gimbal {

class SMC
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SMC()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        epsilon_ = get_parameter("epsilon").as_double();
        c_       = get_parameter("c").as_double();
        k_       = get_parameter("k").as_double();

        moment_of_inertia_    = get_parameter("J").as_double();
        damping_conefficient_ = get_parameter("B").as_double();
    }

    void update() override {}

private:
    double calc_control_value() {
        double control_value;
        // 公共项
        double error_angle    = *current_angle_ - *target_angle_;
        double error_velocity = *current_velocity_ - *target_velocity_;
        double s              = c_ * error_angle + error_velocity;

        // 阻尼项
        double damping_term = damping_conefficient_ * *current_velocity_;

        // 目标角加速度前馈项
        double feedforward_term = moment_of_inertia_ * *target_acceleration_;

        // 控制律
        control_value =
            damping_term + feedforward_term - moment_of_inertia_ * (c_ * error_velocity + epsilon_ * sgn(s) + k_ * s);

        return control_value;
    }

    static double sgn(double s) {
        if (s == 0) {
            return 0;
        } else if (s > 0) {
            return -1;
        } else {
            return 1;
        }
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double c_;       // sliding_surface_para_
    double epsilon_; // switch_gain_
    double k_;       // reaching_law_gain_

    double moment_of_inertia_;
    double damping_conefficient_;

    InputInterface<double> current_angle_;
    InputInterface<double> target_angle_;
    InputInterface<double> current_velocity_;
    InputInterface<double> target_velocity_;
    InputInterface<double> target_acceleration_;

    OutputInterface<double> control_value_;
};

} // namespace rmcs_core::controller::gimbal