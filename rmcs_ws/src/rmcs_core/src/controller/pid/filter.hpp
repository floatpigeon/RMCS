#include <cmath>
#include <cstddef>
#include <queue>
namespace rmcs_core::controller::pid {

class LowPassFilter {
public:
    explicit LowPassFilter(double cutoff = 1000.0, double sample_rate = 1000.0)
        : cutoff_frequency_(cutoff)
        , sample_rate_(sample_rate) {
        last_output_value_ = 0.0;
        calculate_coefficients();
    }

    double cal_output(double input) {
        // 应用一阶低通滤波器: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        double output      = alpha_ * input + (1.0 - alpha_) * last_output_value_;
        last_output_value_ = output;
        return output;
    }

private:
    void calculate_coefficients() {
        // 计算时间常数和alpha系数
        // RC = 1 / (2 * pi * cutoff_frequency)
        // alpha = dt / (RC + dt), 其中 dt = 1/sample_rate

        double dt = 1.0 / sample_rate_;
        double RC = 1.0 / (2.0 * M_PI * cutoff_frequency_);
        alpha_    = dt / (RC + dt);
    }

    double cutoff_frequency_;  // 截止频率(Hz)
    double sample_rate_;       // 采样率(Hz)
    double alpha_;             // 滤波器系数
    double last_output_value_; // 前一个输出值
};

class HybridFilter {
public:
    explicit HybridFilter(double cutoff = 1000.0, double sample_rate = 1000.0)
        : cutoff_frequency_(cutoff)
        , sample_rate_(sample_rate) {
        last_output_value_ = 0.0;
        calculate_coefficients();
    }

    double cal_output(double input) {
        double lowpass_filtered = alpha_ * input + (1.0 - alpha_) * last_output_value_;

        input_value_queue_.push(lowpass_filtered);
        queue_value_sum_ += lowpass_filtered;
        if (input_value_queue_.size() > queue_size_) {
            queue_value_sum_ -= input_value_queue_.front();
            input_value_queue_.pop();
        }
        double avg = queue_value_sum_ / static_cast<double>(queue_size_);

        last_output_value_ = avg;
        return avg;
    }

private:
    void calculate_coefficients() {
        // RC = 1 / (2 * pi * cutoff_frequency)
        // alpha = dt / (RC + dt),  dt = 1/sample_rate

        double dt = 1.0 / sample_rate_;
        double RC = 1.0 / (2.0 * M_PI * cutoff_frequency_);
        alpha_    = dt / (RC + dt);
    }

    double cutoff_frequency_;
    double sample_rate_;
    double alpha_;
    double last_output_value_;

    std::queue<double> input_value_queue_;
    size_t queue_size_      = 25.0;
    double queue_value_sum_ = 0.0;
};
} // namespace rmcs_core::controller::pid