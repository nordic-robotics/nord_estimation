#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ras_arduino_msgs/Encoders.h"
#include "forrest_filter.hpp"
#include "aggregator.hpp"
#include <valarray>

class observer
{
    using Encoders = ras_arduino_msgs::Encoders;
    using Pose2D = geometry_msgs::Pose2D;
public:
    observer(ros::NodeHandle& n)
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
