#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::gimbal {

class SMC
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SMC()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        c_       = get_parameter("c").as_double();
        epsilon_ = get_parameter("epsilon").as_double();
        k_       = get_parameter("k").as_double();
        phi_     = get_parameter("phi").as_double();

        moment_of_inertia_    = get_parameter("J").as_double();
        damping_conefficient_ = get_parameter("B").as_double();
        torque_               = get_parameter("T").as_double();

        register_input("/SMC/test_motor/angle", current_angle_);
        register_input("/SMC/test_motor/target_angle", target_angle_);
        register_input("/SMC/test_motor/velocity", current_velocity_);
        register_input("/SMC/test_motor/target_velocity", target_velocity_);
        register_input("/SMC/test_motor/target_acceleration", target_acceleration_);
        register_input("/SMC/test_motor/torque", current_torque_);

        //
        register_output("/SMC/test_motor/control_torque", control_value_, nan);
        register_output("/angle_error", angle_error_, nan);
        register_output("/sliding_surface_value", sliding_surface_value_, nan);
        register_output("/target_angle/upper_limit", upper_limit_);
        register_output("/target_angle/lower_limit", lower_limit_);
    }

    void update() override {
        RCLCPP_INFO(
            logger_, "angle:%f:%f,velocity:%f:%f,torque:%f:%f", *current_angle_, *target_angle_, *current_velocity_,
            *target_velocity_, *current_torque_, *control_value_);

        if ((*switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::DOWN)
            || (*switch_left_ == rmcs_msgs::Switch::UNKNOWN && *switch_right_ == rmcs_msgs::Switch::UNKNOWN)) {
            *control_value_ = nan;
            return;
        } else if (*switch_left_ == rmcs_msgs::Switch::UP && *switch_right_ == rmcs_msgs::Switch::UP) {
            *control_value_ = torque_;
            return;
        }

        *control_value_ = calc_control_value();

        //
        debug();
    }
    double torque_;

private:
    double calc_control_value() {
        double control_value;
        // 公共项
        double error_angle    = *current_angle_ - *target_angle_;
        double error_velocity = *current_velocity_ - *target_velocity_;
        double s              = c_ * error_angle + error_velocity;

        // debug code
        *sliding_surface_value_ = s;
        // debug code

        // 阻尼项
        double damping_term = damping_conefficient_ * *current_velocity_;

        // 目标角加速度前馈项
        double feedforward_term = moment_of_inertia_ * *target_acceleration_;

        // 积分项，消除稳态误差
        sliding_surface_value_integral_ += s;
        // if (sliding_surface_value_integral_ < 0.01) {
        //     sliding_surface_value_integral_ = 0;
        // }

        // 控制律
        control_value =
            damping_term + feedforward_term
            - moment_of_inertia_ * (c_ * error_velocity + epsilon_ * sgn(s) + k_ * s + k_ * sliding_surface_value_integral_);

        // RCLCPP_INFO(logger_, "control_value:%lf", control_value);
        return control_value;
    }

    double sgn(double s) const {
        auto sp = s / phi_;
        if (sp > 1) {
            return 1;
        } else if (sp < -1) {
            return -1;
        } else {
            return sp;
        }
    }

    void debug() {
        *angle_error_ = *current_angle_ - *target_angle_;
        *upper_limit_ = *target_angle_ + 0.03;
        *lower_limit_ = *target_angle_ - 0.03;
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    double c_;       // sliding_surface_para_
    double epsilon_; // switch_gain_
    double k_;       // reaching_law_gain_
    double phi_;     // boundary_layer_thickness

    double moment_of_inertia_;
    double damping_conefficient_;
    double sliding_surface_value_integral_ = 0;

    InputInterface<double> current_angle_;
    InputInterface<double> target_angle_;
    InputInterface<double> current_velocity_;
    InputInterface<double> target_velocity_;
    InputInterface<double> target_acceleration_;
    InputInterface<double> current_torque_;

    OutputInterface<double> control_value_;
    //

    OutputInterface<double> angle_error_;
    OutputInterface<double> sliding_surface_value_;
    OutputInterface<double> upper_limit_, lower_limit_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::SMC, rmcs_executor::Component)