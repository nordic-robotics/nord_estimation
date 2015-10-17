#pragma once

#include "filter.hpp"

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

class point
{
public:
    point(float x, float y, float z)
        : x(x), y(y), z(z) { };
    point() { };

    void normalize()
    {
        auto d = std::sqrt(x * x + y * y + z * z);
        x /= d;
        y /= d;
        z /= d;
    }

    float x = 0;
    float y = 0;
    float z = 0;
};

class ray
{
public:
    ray(const point& origin, const point& target)
        : origin(origin), target(target) { };
    ray() { };

    point origin;
    point target;
};

class observation
{
public:
    observation(const pose& odometry, const pose& odometry_prev,
                const std::array<ray, 6>& ranges)
        : odometry(odometry), odometry_prev(odometry_prev),
          ranges(ranges) { };
    observation() { };

    pose odometry;
    pose odometry_prev;
    std::array<ray, 6> ranges;
};

class forrest_filter : public dust::filter<pose, observation>
{
public:
    forrest_filter(float alpha_1, float alpha_2, float alpha_3, float alpha_4,
                   float min_x, float min_y, float max_x, float max_y,
                   unsigned int num_particles, const pose& init)
        : alpha_1(alpha_1), alpha_2(alpha_2), alpha_3(alpha_3), alpha_4(alpha_4),
          min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y),
          filter(num_particles, init)
    {
    }
    forrest_filter(float alpha_1, float alpha_2, float alpha_3, float alpha_4,
               float min_x, float min_y, float max_x, float max_y,
               unsigned int num_particles)
        : alpha_1(alpha_1), alpha_2(alpha_2), alpha_3(alpha_3), alpha_4(alpha_4),
          min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y),
          filter(num_particles)
    {
    }

private:
    static constexpr float square(float n);

    // wrap an angle to [-pi, pi]
    // assumes the angle is not more than 2pi away from that range
    static float wrap(float angle);

    // sample from normal distribution with zero mean and b^2 variance
    float sample(float b2) const;

    // probability of the value 'a' given a zero mean and b^2 variance
    float prob(float a, float b2) const;

    // moves a particle forward based on an observation, returns { probability, new_state }
    std::pair<float, pose> motion(const pose& state, const observation& obs) const override;

    // helper functions for motion and probability
    std::pair<float, pose> odometry(const pose& state, const observation& obs) const;
    float rangefinder(const pose& state, const ray& r) const;

    // creates a random particle
    pose uniform() const override;

    // odometry parameters
    float alpha_1;
    float alpha_2;
    float alpha_3;
    float alpha_4;
    // map parameters
    float min_x;
    float min_y;
    float max_x;
    float max_y;
};
