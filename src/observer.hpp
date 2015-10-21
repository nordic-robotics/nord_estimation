#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ras_arduino_msgs/Encoders.h"
#include "nord_messages/IRSensors.h"
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
    using IRSensors = nord_messages::IRSensors;
    using Pose2D = geometry_msgs::Pose2D;
public:
    observer(ros::NodeHandle& n, const observer_settings& settings)
        : pub(n.advertise<geometry_msgs::Pose2D>("/nord/estimation/particle_filter", 10)),
          encoders( n, "/arduino/encoders",
                    [settings](const Encoders::ConstPtr& e) {
                        Pose2D res;
                        float delta_time = e->timestamp / 1000.0f;
                        auto estimated_w1 = float((e->delta_encoder1 * 2 * M_PI / delta_time)
                                                 / 360.0f);
                        auto estimated_w2 = float((e->delta_encoder2 * 2 * M_PI / delta_time)
                                                 / 360.0f);
                        auto v = settings.wheel_r * (estimated_w1 + estimated_w2) / 2.0f;
                        auto omega = settings.wheel_r * (estimated_w1 - estimated_w2)
                                   / settings.wheel_b;
                        return std::valarray<float>({v, omega, delta_time});
                    },
                    [](const std::vector<std::valarray<float>>& readings) {
                        pose p;
                        for (auto& entry : readings)
                        {
                            float v = entry[0];
                            float omega = entry[1];
                            float delta_time = entry[2];
                            p.x += -std::sin(p.theta) * v * delta_time;
                            p.y += std::cos(p.theta) * v * delta_time;
                            p.theta += omega * delta_time;
                        }
                        return p;
                    }),
          ir_sensors(   n, "/nord/sensors/ir",
                        [settings](const IRSensors::ConstPtr& ir) {
                            return std::array<line<2>, 6>({
                                line<2>(settings.ir_front,
                                        settings.ir_front + point<2>(0, ir->front)),
                                line<2>(settings.ir_back,
                                        settings.ir_back + point<2>(0, -ir->back)),
                                line<2>(settings.ir_left_front,
                                        settings.ir_left_front + point<2>(ir->left_front, 0)),
                                line<2>(settings.ir_left_back,
                                        settings.ir_left_back + point<2>(ir->left_back, 0)),
                                line<2>(settings.ir_right_front,
                                        settings.ir_right_front + point<2>(-ir->right_front, 0)),
                                line<2>(settings.ir_right_back,
                                        settings.ir_right_back + point<2>(-ir->right_back, 0)),
                            });
                        })
    { };

    bool all_new()
    {
        return encoders.has_new()
            && ir_sensors.has_new();
    }

    ros::Publisher pub;

    aggregate::custom<std::valarray<float>, Encoders, pose> encoders;
    aggregate::latest<std::array<line<2>, 6>, IRSensors> ir_sensors;
};