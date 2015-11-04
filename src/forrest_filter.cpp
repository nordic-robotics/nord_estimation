#include "forrest_filter.hpp"
#include <cmath>

namespace
{
    float square(float n)
    {
        return n * n;
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
        return float(z >= theta.z_max);
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

// sample from normal distribution with zero mean and b variance
float forrest_filter::sample(float b) const
{
    float sum = 0;
    for (unsigned int i = 0; i < 12; i++)
    {
        sum += dist_sample(gen);
    }
    return b * sum / 6;
}

pose forrest_filter::motion_model(const pose& state, const observation& obs) const
{
    auto v_hat = obs.v + sample(alpha[0] * std::abs(obs.v)
                              + alpha[1] * std::abs(obs.w));
    auto w_hat = obs.w + sample(alpha[2] * std::abs(obs.v)
                              + alpha[3] * std::abs(obs.w));
    auto gamma_hat =     sample(alpha[4] * std::abs(obs.v)
                              + alpha[5] * std::abs(obs.w));

    auto vw = v_hat / w_hat;

    pose next = state;
    if (w_hat != 0)
    {
        next.x += - vw * std::sin(state.theta) + vw * sin(state.theta + w_hat * obs.dt);
        next.y += + vw * std::cos(state.theta) - vw * cos(state.theta + w_hat * obs.dt);
    }
    next.theta += w_hat * obs.dt + gamma_hat * obs.dt;

    return next;
}

// currently unused
float forrest_filter::motion_probability(const pose& state, const pose& next,
                                         const observation& obs) const
{
    auto mu = 0.5 * ((state.x - next.x) * std::cos(state.theta)
                   + (state.y - next.y) * std::sin(state.theta))
                  / ((state.y - next.y) * std::cos(state.theta)
                   - (state.x - next.x) * std::sin(state.theta));

    auto xs = (state.x + next.x) / 2 + mu * (state.y - next.y);
    auto ys = (state.y + next.y) / 2 + mu * (next.x - state.x);
    auto rs = std::hypot(state.x - xs, state.y - ys);

    auto d_theta = std::atan2(next.y - ys, next.x - xs)
                 - std::atan2(state.y - ys, state.x - xs);

    auto v_hat = rs * d_theta / obs.dt;
    auto w_hat = d_theta / obs.dt;
    auto gamma_hat = (next.theta - state.theta) / obs.dt - w_hat;

    return prob(obs.v - v_hat, alpha[0] * std::abs(obs.v) + alpha[1] * std::abs(obs.w))
         * prob(obs.w - w_hat, alpha[2] * std::abs(obs.v) + alpha[3] * std::abs(obs.w))
         * prob(gamma_hat,     alpha[4] * std::abs(obs.v) + alpha[5] * std::abs(obs.w));
}

float forrest_filter::rangefinder(const line<2>& r, const range_settings& theta) const
{
    // direction of the IR sensor
    auto dir = (r.end - r.start).normalized();
    // longest possible intersection ray
    auto ray = line<2>(r.start, r.start + dir * theta.z_max);
    auto p = maze.raycast(ray);

    // if no collision, set to z_max, otherwise set to distance between start and intersection point
    auto z_star = theta.z_max;
    if (p)
        z_star = (p.value() - r.start).length();
    auto z = r.length();

    return theta.z_p_hit * p_hit(theta, z, z_star)
         + theta.z_p_short * p_short(theta, z, z_star)
         + theta.z_p_max * p_max(theta, z)
         + theta.z_p_rand * p_rand(theta, z);
}

float forrest_filter::map_probability(const pose& state, const pose& next) const
{
    auto inside = float(maze.contains(point<2>(state.x, state.y)));
    auto hit_wall = bool(maze.raycast(line<2>(point<2>(state.x, state.y),
                                              point<2>(next.x, next.y))));
    return (inside * float(!hit_wall) + 0.000000001) / maze.get_area();
}

std::pair<float, pose> forrest_filter::motion(const pose& state,
                                              const observation& obs) const
{
    std::pair<float, pose> next;
    next.second = motion_model(state, obs);
    auto loc = point<2>(next.second.x, next.second.y);

    // rotate the IR sensor by current rotation and offset by current position before simulating
    float p_ir_long = 1.0f;
    for (size_t i = 0; i < 2; i++)
    {
        p_ir_long *= rangefinder(obs.ir[i].rotated(next.second.theta) + loc, ir_theta[i]);
    }

    float p_ir_short = 1.0f;
    for (size_t i = 2; i < obs.ir.size(); i++)
    {
        p_ir_short *= rangefinder(obs.ir[i].rotated(next.second.theta) + loc, ir_theta[i]);
    }

    float p_maze = map_probability(state, next.second);

    // should all be normalized, clear to multiply!
    next.first = (p_ir_long
                * p_ir_short
                * p_maze);
    return next;
}

pose forrest_filter::uniform() const
{
    return pose(dist_x(gen), dist_y(gen), dist_theta(gen));
}
