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
    observation(float v, float w, const std::array<line<2>, 6>& ir, float dt)
        : v(v), w(w), ir(ir), dt(dt) { };
    observation() { };

    float v;
    float w;
    std::array<line<2>, 6> ir;
    float dt;
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
    forrest_filter(const std::array<float, 6>& alpha,
                   const std::array<range_settings, 6>& ir_theta,
                   unsigned int num_particles, map& maze, const pose& init)
        : alpha(alpha), ir_theta(ir_theta), maze(maze),
          dist_x(maze.get_min_x(), maze.get_max_x()),
          dist_y(maze.get_min_y(), maze.get_max_y()),
          dist_theta(0, 2 * M_PI), dist_sample(-1, 1),
          filter(num_particles, init)
    {
    }

protected:
    // moves a particle forward based on an observation, returns { probability, new_state }
    std::pair<float, pose> motion(const pose& state, const observation& obs) const override;

    // creates a random particle
    pose uniform() const override;

private:
    // sample from normal distribution with zero mean and b^2 variance
    float sample(float b2) const;

    // helper functions for motion and probability
    pose motion_model(const pose& state, const observation& obs) const;
    float motion_probability(const pose& state, const pose& next,
                             const observation& obs) const;
    float rangefinder(const line<2>& r, const range_settings& theta) const;
    float map_probability(const pose& state, const pose& next) const;

    map& maze;

    // odometry parameters
    std::array<float, 6> alpha;
    // IR parameters
    std::array<range_settings, 6> ir_theta;

    // uniform map distributions
    mutable std::uniform_real_distribution<float> dist_x;
    mutable std::uniform_real_distribution<float> dist_y;
    mutable std::uniform_real_distribution<float> dist_theta;
    mutable std::uniform_real_distribution<float> dist_sample;
};
