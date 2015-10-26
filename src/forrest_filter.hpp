#pragma once

#include "filter.hpp"
#include "line.hpp"
#include "map.hpp"

class pose
{
public:
    pose(float x, float y, float theta)
        : x(x), y(y), theta(theta) { };
    pose() { };

    float x = 0;
    float y = 0;
    float theta = 0;
};

class observation
{
public:
    observation(const pose& odometry, const pose& odometry_prev,
                const std::array<line<2>, 6>& ir)
        : odometry(odometry), odometry_prev(odometry_prev),
          ir(ir) { };
    observation() { };

    pose odometry;
    pose odometry_prev;
    std::array<line<2>, 6> ir;
};

class range_settings
{
public:
    range_settings(float z_max, float sigma_hit, float lambda_short,
                   float z_p_hit, float z_p_short, float z_p_max, float z_p_rand)
        : z_max(z_max), sigma_hit(sigma_hit), lambda_short(lambda_short),
          z_p_hit(z_p_hit), z_p_short(z_p_short),
          z_p_max(z_p_max), z_p_rand(z_p_rand) { };
    range_settings() { };

    float z_max;
    float sigma_hit;
    float lambda_short;
    float z_p_hit;
    float z_p_short;
    float z_p_max;
    float z_p_rand;
};

class forrest_filter : public dust::filter<pose, observation>
{
public:
    forrest_filter(const std::array<float, 4>& alpha,
                   const std::array<range_settings, 6>& ir_theta,
                   unsigned int num_particles, map& maze, const pose& init)
        : alpha(alpha), ir_theta(ir_theta), maze(maze),
          dist_x(maze.get_min_x(), maze.get_max_x()),
          dist_y(maze.get_min_y(), maze.get_max_y()),
          dist_theta(-M_PI, M_PI), filter(num_particles, init)
    {
    }
    forrest_filter(const std::array<float, 4>& alpha,
                   const std::array<range_settings, 6>& ir_theta,
                   unsigned int num_particles, map& maze)
        : alpha(alpha), ir_theta(ir_theta), maze(maze),
          dist_x(maze.get_min_x(), maze.get_max_x()),
          dist_y(maze.get_min_y(), maze.get_max_y()),
          dist_theta(-M_PI, M_PI), filter(num_particles)
    {
    }

private:
public:
    // sample from normal distribution with zero mean and b^2 variance
    float sample(float b2) const;

    // moves a particle forward based on an observation, returns { probability, new_state }
    std::pair<float, pose> motion(const pose& state, const observation& obs) const override;

    // helper functions for motion and probability
    std::pair<float, pose> odometry(const pose& state, const observation& obs) const;
    float rangefinder(const line<2>& r, const range_settings& theta) const;

    // creates a random particle
    pose uniform() const override;

    map& maze;

    // odometry parameters
    std::array<float, 4> alpha;
    // IR parameters
    std::array<range_settings, 6> ir_theta;

    // uniform map distributions
    mutable std::uniform_real_distribution<float> dist_x;
    mutable std::uniform_real_distribution<float> dist_y;
    mutable std::uniform_real_distribution<float> dist_theta;
};
