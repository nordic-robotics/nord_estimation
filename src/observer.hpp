#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ras_arduino_msgs/Encoders.h"
#include "forrest_filter.hpp"
#include "aggregator.hpp"
#include <valarray>

class observer_settings
{
public:
    observer_settings(const point<2>& ir_front, const point<2>& ir_back,
                      const point<2>& ir_left_front, const point<2>& ir_left_back,
                      const point<2>& ir_right_front, const point<2>& ir_right_back,
                      float wheel_r, float wheel_b)
        : ir_front(ir_front), ir_back(ir_back),
          ir_left_front(ir_left_front), ir_left_back(ir_left_back),
          ir_right_front(ir_right_front), ir_right_back(ir_right_back),
          wheel_r(wheel_r), wheel_b(wheel_b) { };

    point<2> ir_front;
    point<2> ir_back;
    point<2> ir_left_front;
    point<2> ir_left_back;
    point<2> ir_right_front;
    point<2> ir_right_back;
    float wheel_r;
    float wheel_b;
};

class observer
{
    using Encoders = ras_arduino_msgs::Encoders;
    using Pose2D = geometry_msgs::Pose2D;
public:
    observer(ros::NodeHandle& n, const observer_settings& settings)
        : pub(n.advertise<geometry_msgs::Pose2D>("/nord/estimation/particle_filter", 10)),
          encoders(n, "/arduino/encoders",
                   [](const Encoders::ConstPtr& e) {
                       Pose2D res;
                       // TODO: check timestamp unit
                       float delta_time = e->timestamp / 1000.0f;
                       auto estimated_w1 = float((e->delta_encoder1 * 2 * M_PI / delta_time)
                                                / 360.0f);
                       auto estimated_w2 = float((e->delta_encoder2 * 2 * M_PI / delta_time)
                                                / 360.0f);
                       const auto b = 0.2015f;
                       const auto r = 0.09935f;
                       auto v = r * (estimated_w1 + estimated_w2) / 2.0f;
                       auto omega = r * (estimated_w1 - estimated_w2) / b;
                       return std::valarray<float>({v, omega, delta_time});
                   },
                   [](const std::vector<std::valarray<float>>& readings) {
                       Pose2D pose;
                       for (auto& entry : readings)
                       {
                           float v = entry[0];
                           float omega = entry[1];
                           float delta_time = entry[2];
                           pose.x += -std::sin(pose.theta) * v * delta_time;
                           pose.y += std::cos(pose.theta) * v * delta_time;
                           pose.theta += omega * delta_time;
                       }
                       return pose;
                   })
    { };

    ros::Publisher pub;

    aggregate::custom<std::valarray<float>, Encoders, Pose2D> encoders;
};
