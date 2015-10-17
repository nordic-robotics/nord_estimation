#include "forrest_filter.hpp"
#include <cmath>

constexpr float forrest_filter::square(float n)
{
    return n * n;
}

// wrap an angle to [-pi, pi]
// assumes the angle is not more than 2pi away from that range
float forrest_filter::wrap(float angle)
{
    if (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    return angle;
}

// sample from normal distribution with zero mean and b^2 variance
float forrest_filter::sample(float b2) const
{
    static std::uniform_real_distribution<float> dist(-1, 1);
    auto b = std::sqrt(b2);
    float sum = 0;
    for (unsigned int i = 0; i < 12; i++)
    {
        sum += dist(gen) * b;
    }
    return sum / 2;
}

// probability of the value 'a' given a zero mean and b^2 variance
float forrest_filter::prob(float a, float b2) const
{
    auto numerator = std::exp(-0.5f * square(a) / b2);
    auto denominator = std::sqrt(2 * M_PI * b2);
    return numerator / denominator;
}

std::pair<float, pose> forrest_filter::odometry(const pose& state,
                                                const observation& obs) const
{
    // common to both motion and probability
    auto delta_rot1 = std::atan2(obs.odometry.y - obs.odometry_prev.y,
                                 obs.odometry.x - obs.odometry_prev.x)
                    - obs.odometry_prev.theta;
    auto delta_trans = std::hypot(obs.odometry_prev.x - obs.odometry.x,
                                  obs.odometry_prev.y - obs.odometry.y);
    auto delta_rot2 = wrap(obs.odometry.theta - obs.odometry_prev.theta - delta_rot1);

    // motion
    auto delta_rot1_hat = delta_rot1 - sample(alpha_1 * square(delta_rot1)
                                            + alpha_2 * square(delta_trans));
    auto delta_trans_hat = delta_trans - sample(alpha_3 * square(delta_trans)
                                              + alpha_4 * square(delta_rot1)
                                              + alpha_4 * square(delta_rot2));
    auto delta_rot2_hat = delta_rot2 - sample(alpha_1 * square(delta_rot2)
                                            + alpha_2 * square(delta_trans));

    pose next;
    next.x = state.x + delta_trans_hat * std::cos(state.theta + delta_rot1_hat);
    next.y = state.y + delta_trans_hat * std::sin(state.theta + delta_rot1_hat);
    next.theta = wrap(state.theta + delta_rot1_hat + delta_rot2_hat);

    // probability
    auto p_delta_rot1_hat = std::atan2(next.y - state.y, next.x - state.x)
                          - state.theta;
    auto p_delta_trans_hat = std::hypot(state.x - next.x, state.y - next.y);
    auto p_delta_rot2_hat = wrap(next.theta - state.theta - p_delta_rot1_hat);

    auto p1 = prob(wrap(delta_rot1 - p_delta_rot1_hat),
                   alpha_1 * square(p_delta_rot1_hat)
                 + alpha_2 * square(p_delta_trans_hat));
    auto p2 = prob(wrap(delta_trans - p_delta_trans_hat),
                   alpha_3 * square(p_delta_trans_hat)
                 + alpha_4 * square(p_delta_rot1_hat)
                 + alpha_2 * square(p_delta_trans_hat));
    auto p3 = prob(wrap(delta_rot2 - p_delta_rot2_hat),
                   alpha_1 * square(p_delta_rot2_hat)
                 + alpha_2 * square(p_delta_trans_hat));

    return {p1 * p2 * p3, next};
}

float forrest_filter::rangefinder(const pose& state, const line<2>& r) const
{
    return 1;
}

std::pair<float, pose> forrest_filter::motion(const pose& state,
                                              const observation& obs) const
{
    auto next = odometry(state, obs);
    for (auto& r : obs.ranges)
    {
        next.first *= rangefinder(state, r);
    }
    return next;
}

pose forrest_filter::uniform() const
{
    return pose();
}
