#include "forrest_filter.hpp"
#include <cmath>

namespace
{
    float square(float n)
    {
        return n * n;
    }

    // wrap an angle to [-pi, pi]
    // assumes the angle is not more than 2pi away from that range
    float wrap(float angle)
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

    // probability of the value 'a' given a zero mean and b^2 variance
    float prob(float a, float b2)
    {
        auto numerator = std::exp(-0.5f * square(a) / b2);
        auto denominator = std::sqrt(2 * M_PI * b2);
        return numerator / denominator;
    }

    // gaussian rangefinder distribution
    float p_hit(const range_settings& theta, float z, float z_star)
    {
        if (0 <= z && z <= theta.z_max)
            return (std::sqrt(2 / M_PI) * std::exp(-square(z - z_star)
                                                 / (2 * square(theta.sigma_hit))))
                 / (theta.sigma_hit * (erf(z_star / (sqrt(2) * theta.sigma_hit))
                                     - erf((z_star - theta.z_max)
                                         / (sqrt(2) * theta.sigma_hit))));
        else
            return 0.0f;
    }

    // exponential rangefinder distribution
    float p_short(const range_settings& theta, float z, float z_star)
    {
        if (0 <= z && z <= z_star)
            return (theta.lambda_short * std::exp(-theta.lambda_short * z))
                 / (1 - std::exp(-theta.lambda_short * z_star));
        else
            return 0.0f;
    }

    // discrete rangefinder distribution
    float p_max(const range_settings& theta, float z)
    {
        return float(z == theta.z_max);
    }

    // uniform rangefinder distribution
    float p_rand(const range_settings& theta, float z)
    {
        if (0 <= z && z <= theta.z_max)
            return 1.0f / theta.z_max;
        else
            return 0.0f;
    }
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
    auto delta_rot1_hat = delta_rot1 - sample(alpha[0] * square(delta_rot1)
                                            + alpha[1] * square(delta_trans));
    auto delta_trans_hat = delta_trans - sample(alpha[2] * square(delta_trans)
                                              + alpha[3] * square(delta_rot1)
                                              + alpha[3] * square(delta_rot2));
    auto delta_rot2_hat = delta_rot2 - sample(alpha[0] * square(delta_rot2)
                                            + alpha[1] * square(delta_trans));

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
                   alpha[0] * square(p_delta_rot1_hat)
                 + alpha[1] * square(p_delta_trans_hat));
    auto p2 = prob(wrap(delta_trans - p_delta_trans_hat),
                   alpha[2] * square(p_delta_trans_hat)
                 + alpha[3] * square(p_delta_rot1_hat)
                 + alpha[1] * square(p_delta_trans_hat));
    auto p3 = prob(wrap(delta_rot2 - p_delta_rot2_hat),
                   alpha[0] * square(p_delta_rot2_hat)
                 + alpha[1] * square(p_delta_trans_hat));

    return {p1 * p2 * p3, next};
}

float forrest_filter::rangefinder(const line<2>& r, const range_settings& theta) const
{
    auto dir = (r.end - r.start).normalized();
    auto ray = line<2>(r.start, r.start + dir * theta.z_max * 2);
    auto p = maze->raycast(ray);
    auto z_star = theta.z_max;
    if (p)
        z_star = (p.value() - r.start).length();
    auto z = r.length();

    return theta.z_p_hit * p_hit(theta, z, z_star)
         + theta.z_p_short * p_short(theta, z, z_star)
         + theta.z_p_max * p_max(theta, z)
         + theta.z_p_rand * p_rand(theta, z);
}

std::pair<float, pose> forrest_filter::motion(const pose& state,
                                              const observation& obs) const
{
    auto next = odometry(state, obs);
    for (size_t i = 0; i < obs.ir.size(); i++)
    {
        next.first *= rangefinder(obs.ir[i], ir_theta[i]);
    }
    return next;
}

pose forrest_filter::uniform() const
{
    return pose();
}
