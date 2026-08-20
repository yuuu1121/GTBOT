#pragma once

#include <cmath>

namespace fast_lio::localization {

/**
 * @brief Simple 1D Kalman Filter for pose smoothing.
 *
 * Applied independently to X, Y, Z axes to reduce high-frequency noise
 * from ICP registration results. Header-only — no ROS2/Open3D dependency.
 */
class KalmanFilter
{
public:
    KalmanFilter() = default;

    void init(double process_var, double meas_var,
              double initial_value = 0.0, double initial_error = 1.0)
    {
        process_var_ = process_var;
        meas_var_ = meas_var;
        post_estimate_ = initial_value;
        post_error_estimate_ = initial_error;
    }

    void update(double measurement)
    {
        const double prior_estimate = post_estimate_;
        const double prior_error = post_error_estimate_ + process_var_;
        const double denominator = prior_error + meas_var_;

        // Prevent division by zero (numerically degenerate measurement variance).
        if (std::abs(denominator) < 1e-10)
        {
            post_estimate_ = measurement;
            post_error_estimate_ = 1.0;
            return;
        }

        const double kalman_gain = prior_error / denominator;
        post_estimate_ = prior_estimate + kalman_gain * (measurement - prior_estimate);
        post_error_estimate_ = (1.0 - kalman_gain) * prior_error;
    }

    double getEstimate() const { return post_estimate_; }

private:
    double process_var_ = 0.0;
    double meas_var_ = 0.0;
    double post_estimate_ = 0.0;
    double post_error_estimate_ = 1.0;
};

}  // namespace fast_lio::localization
