#pragma once

#include "filter.hpp"
#include "line.hpp"
#include "map.hpp"
#include "nord_messages/PoseEstimate.h"

class pose
{
public:
    pose(double x, double y, double theta)
        : x(x), y(y), theta(theta) { };
    pose() { };

    double x = 0;
    double y = 0;
    double theta = 0;
};

class observation
{
public:
    observation(double v, double w, double w1, double w2,
                const std::array<line<2>, 6>& ir, double ang_z, double dt)
        : v(v), w(w), w1(w1), w2(w2), ir(ir), ang_z(ang_z), dt(dt) { };
    observation() { };

    double v;
    double w;
    double w1;
    double w2;
    std::array<line<2>, 6> ir;
    double ang_z;
    double dt;
};

class range_settings
{
public:
    range_settings(double z_max, double sigma_hit, double lambda_short,
                   double z_p_hit, double z_p_short, double z_p_max, double z_p_rand)
        : z_max(z_max), sigma_hit(sigma_hit), lambda_short(lambda_short),
          z_p_hit(z_p_hit), z_p_short(z_p_short),
          z_p_max(z_p_max), z_p_rand(z_p_rand) { };
    range_settings() { };

    double z_max;
    double sigma_hit;
    double lambda_short;
    double z_p_hit;
    double z_p_short;
    double z_p_max;
    double z_p_rand;
};

class forrest_filter : public dust::filter<pose, observation>
{
public:
    forrest_filter(const std::array<double, 2>& alpha,
                   const std::array<range_settings, 6>& ir_theta,
                   double imu_variance,
                   unsigned int num_particles, map& maze, const pose& init)
        : alpha(alpha), ir_theta(ir_theta), imu_variance(imu_variance), maze(maze),
          dist_x(maze.get_min_x(), maze.get_max_x()),
          dist_y(maze.get_min_y(), maze.get_max_y()),
          dist_theta(0, 2 * M_PI), dist_sample(-1, 1),
          filter(num_particles, init)
    {
    }

    void bump(const nord_messages::PoseEstimate& last_estimate,
              float bump_xy_multiplier, float bump_theta_multiplier);

protected:
    // moves a particle forward based on an observation, returns { probability, new_state }
    std::pair<double, pose> motion(const pose& state, const observation& obs) const override;

    // creates a random particle
    pose uniform() const override;

private:
    // sample from normal distribution with zero mean and b^2 variance
    double sample(double b2) const;

    // helper functions for motion and probability
    pose motion_model_velocity(const pose& state, const observation& obs) const;
    pose motion_model_cool(const pose& state, const observation& obs) const;
    double motion_probability(const pose& state, const pose& next,
                             const observation& obs) const;
    double rangefinder(const line<2>& r, const range_settings& theta) const;
    double map_probability(const pose& state, const pose& next) const;
    double imu_probability(const pose& state, const pose& next,
                          const observation& obs) const;

    map& maze;

    // odometry parameters
    std::array<double, 2> alpha;
    // IR parameters
    std::array<range_settings, 6> ir_theta;
    double imu_variance;

    // uniform map distributions
    mutable std::uniform_real_distribution<double> dist_x;
    mutable std::uniform_real_distribution<double> dist_y;
    mutable std::uniform_real_distribution<double> dist_theta;
    mutable std::uniform_real_distribution<double> dist_sample;
};
