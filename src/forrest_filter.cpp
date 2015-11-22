#include "forrest_filter.hpp"
#include <cmath>

namespace
{
    double square(double n)
    {
        return n * n;
    }

    // wrap an angle to [-pi, pi]
    // assumes the angle is not more than 2pi away from that range
    double wrap(double angle)
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

    double ang_diff(double a, double b)
    {
        double diff = wrap(b) - wrap(a);
        return wrap(diff);
    }

    // probability of the value 'a' given a zero mean and b^2 variance
    double prob(double a, double b2, double mu = 0)
    {
        auto numerator = std::exp(-0.5f * square(a - mu) / b2);
        auto denominator = std::sqrt(2 * M_PI * b2);
        return numerator / denominator;
    }

    // gaussian rangefinder distribution
    double p_hit(const range_settings& theta, double z, double z_star)
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
    double p_short(const range_settings& theta, double z, double z_star)
    {
        if (0 <= z && z <= z_star)
            return (theta.lambda_short * std::exp(-theta.lambda_short * z))
                 / (1 - std::exp(-theta.lambda_short * z_star));
        else
            return 0.0f;
    }

    // discrete rangefinder distribution
    double p_max(const range_settings& theta, double z)
    {
        return double(z >= theta.z_max);
    }

    // uniform rangefinder distribution
    double p_rand(const range_settings& theta, double z)
    {
        if (0 <= z && z <= theta.z_max)
            return 1.0f / theta.z_max;
        else
            return 0.0f;
    }
}

// sample from normal distribution with zero mean and b variance
double forrest_filter::sample(double b) const
{
    double sum = 0;
    for (unsigned int i = 0; i < 12; i++)
    {
        sum += dist_sample(gen);
    }
    return b * sum / 6;
}

pose forrest_filter::motion_model_velocity(const pose& state, const observation& obs) const
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
    if (std::isnan(vw)) { std::cout << "vw broken" << std::endl; }
    }
    next.theta += w_hat * obs.dt + gamma_hat * obs.dt;
    next.theta = wrap(next.theta);
    if (std::isnan(next.theta)) { std::cout << "theta broken" << std::endl; }
    if (std::isnan(next.x)) { std::cout << "x broken" << std::endl; }
    if (std::isnan(next.y)) { std::cout << "y broken" << std::endl; }
    if (std::isnan(v_hat)) { std::cout << "v_hat broken" << std::endl; }
    if (std::isnan(w_hat)) { std::cout << "w_hat broken" << std::endl; }
    if (std::isnan(obs.dt)) { std::cout << "dt broken" << std::endl; }

    return next;
}

pose forrest_filter::motion_model_cool(const pose& state, const observation& obs) const
{
    auto l = 0.2015;
    auto r = 0.049675;
    auto Vr = (obs.w2 + sample(alpha[0] * std::abs(obs.w2))) * r;
    auto Vl = (obs.w1 + sample(alpha[1] * std::abs(obs.w1))) * r;
    auto w = (Vr - Vl) / l;
   // std::cout << "w1,w2: " << obs.w1 << " " << obs.w2 << std::endl;
  //  std::cout << "w,Vr,Vl: " << w << " " << Vr << " " << Vl << std::endl;
    auto R = (l / 2.0f) * ((Vl + Vr) / (Vr - Vl));
    if (Vr == Vl)
        R = 0; // danger zone
    auto ICCx = state.x - R * std::sin(state.theta);
    auto ICCy = state.y + R * std::cos(state.theta);

    auto A11 = std::cos(w * obs.dt);
    auto A21 = std::sin(w * obs.dt);
    auto A12 = -std::sin(w * obs.dt);
    auto A22 = std::cos(w * obs.dt);
    auto B1 = state.x - ICCx;
    auto B2 = state.y - ICCy;

    pose next = state;
    next.x = B1 * A11 + B2 * A12 + ICCx;
    next.y = B1 * A21 + B2 * A22 + ICCy;
    next.theta = wrap(state.theta + w * obs.dt);

    return next;
}

// currently unused
double forrest_filter::motion_probability(const pose& state, const pose& next,
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

double forrest_filter::rangefinder(const line<2>& r, const range_settings& theta) const
{
    // direction of the IR sensor
    auto dir = (r.end - r.start).normalized();
    // longest possible intersection ray
    auto ray = line<2>(r.start, r.start + dir * theta.z_max);
    auto p = maze.raycast(ray);

    // if no collision, set to z_max, otherwise set
    // to distance between start and intersection point
    auto z_star = theta.z_max;
    if (p)
        z_star = (p.value() - r.start).length();
    auto z = r.length();

    return theta.z_p_hit * p_hit(theta, z, z_star)
         + theta.z_p_short * p_short(theta, z, z_star)
         + theta.z_p_max * p_max(theta, z)
         + theta.z_p_rand * p_rand(theta, z);
}

double forrest_filter::map_probability(const pose& state, const pose& next) const
{
    auto inside = double(maze.contains(point<2>(state.x, state.y)));
    auto hit_wall = bool(maze.raycast(line<2>(point<2>(state.x, state.y),
                                              point<2>(next.x, next.y))));
    return (inside * double(!hit_wall) + 0.000000001) / maze.get_area();
}

double forrest_filter::imu_probability(const pose& state, const pose& next,
                                      const observation& obs) const
{
    return prob(wrap(obs.ang_z * obs.dt), imu_variance, ang_diff(state.theta, next.theta));
}

std::pair<double, pose> forrest_filter::motion(const pose& state,
                                              const observation& obs) const
{
    std::pair<double, pose> next;
    next.second = motion_model_cool(state, obs);
    auto loc = point<2>(next.second.x, next.second.y);

    // rotate the IR sensor by current rotation and offset
    // by current position before simulating
    double p_ir_long = 1.0f;
    for (size_t i = 0; i < 1; i++)
    {
        p_ir_long *= rangefinder(obs.ir[i].rotated(next.second.theta) + loc, ir_theta[i]);
    }

    double p_ir_short = 1.0f;
    for (size_t i = 2; i < obs.ir.size(); i++)
    {
        p_ir_short *= rangefinder(obs.ir[i].rotated(next.second.theta) + loc, ir_theta[i]);
    }

    double p_maze = map_probability(state, next.second);

    double p_imu = imu_probability(state, next.second, obs);

    // should all be normalized, clear to multiply!
    next.first = (p_ir_long
                * p_ir_short
                * p_maze
                * p_imu);
    return next;
}

pose forrest_filter::uniform() const
{
    return pose(dist_x(gen), dist_y(gen), dist_theta(gen));
}

void forrest_filter::bump(const nord_messages::PoseEstimate& current,
                          float bump_xy_multiplier, float bump_theta_multiplier)
{
    using dist_t = std::normal_distribution<double>;
    dist_t dist;

    reset(get_num_particles(), [&]() {
        return pose(dist(gen, dist_t::param_type(current.x.mean,
                                                 square(current.x.stddev)
                                               * bump_xy_multiplier)),
                    dist(gen, dist_t::param_type(current.y.mean,
                                                 square(current.y.stddev)
                                               * bump_xy_multiplier)),
                    dist(gen, dist_t::param_type(current.theta.mean,
                                                 square(current.theta.stddev)
                                               * bump_theta_multiplier)));
    });
}