#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ras_arduino_msgs/Encoders.h"
#include "nord_messages/IRSensors.h"
#include "sensor_msgs/Imu.h"
#include "forrest_filter.hpp"
#include "aggregator.hpp"
#include <valarray>

class observer_settings
{
public:
    observer_settings(const point<2>& ir_front, const point<2>& ir_back,
                      const point<2>& ir_left_front, const point<2>& ir_left_back,
                      const point<2>& ir_right_front, const point<2>& ir_right_back,
                      double wheel_r, double wheel_b)
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
    double wheel_r;
    double wheel_b;
};

class observer
{
    using Encoders = ras_arduino_msgs::Encoders;
    using IRSensors = nord_messages::IRSensors;
    using Pose2D = geometry_msgs::Pose2D;
    using Imu = sensor_msgs::Imu;
    using Vector2Array = nord_messages::Vector2Array;
public:
    observer(ros::NodeHandle& n, const observer_settings& settings)
        : pub(n.advertise<geometry_msgs::Pose2D>("/nord/estimation/particle_filter", 10)),
          encoders( n, "/arduino/encoders",
                    [](const Encoders::ConstPtr& e) {
                        return *e;
                    },
                    [settings, this](const std::vector<Encoders>& es)
                    {
                        Pose2D res;
                        if (last_enc.timestamp == 0)
                        {
                            std::cout << "timestamp == 0" << std::endl;
                            last_enc = es.back();
                            return std::valarray<double>({0, 0, 0, 0});
                        }
                        int16_t delta_e1 = int16_t(es.back().encoder1)
                                         - int16_t(last_enc.encoder1);
                        int16_t delta_e2 = int16_t(es.back().encoder2)
                                         - int16_t(last_enc.encoder2);
                        double delta_time = 0;
                        for (auto& e : es)
                        {
                            delta_time += e.timestamp;
                        }
                        delta_time /= 1000.0f;
                        if (delta_time >= 1.0)
                        {
                            std::cout << "delta >= 1 " << std::endl;
                            last_enc = es.back();
                            return std::valarray<double>({0, 0, 0, 0});
                        }
                        double estimated_w1 = (-double(delta_e1) * 2 * M_PI / delta_time)
                                            / 360.0f;
                        double estimated_w2 = (-double(delta_e2) * 2 * M_PI / delta_time)
                                            / 360.0f;
                        double v = settings.wheel_r * (estimated_w1 + estimated_w2) / 2.0f;
                        double w = settings.wheel_r * (estimated_w2 - estimated_w1)
                                 / settings.wheel_b;
                        last_enc = es.back();
                        return std::valarray<double>({v, w, estimated_w1, estimated_w2});
                    }),
          ir_sensors(   n, "/nord/sensors/ir",
                        [settings](const IRSensors::ConstPtr& ir) {
                            // reconstruct IR rays
                            return std::array<line<2>, 6>({
                                line<2>(settings.ir_front,
                                        settings.ir_front + point<2>(ir->front, 0)),
                                line<2>(settings.ir_back,
                                        settings.ir_back + point<2>(-ir->back, 0)),
                                line<2>(settings.ir_left_front,
                                        settings.ir_left_front + point<2>(0, ir->left_front)),
                                line<2>(settings.ir_left_back,
                                        settings.ir_left_back + point<2>(0, ir->left_back)),
                                line<2>(settings.ir_right_front,
                                        settings.ir_right_front + point<2>(0, -ir->right_front)),
                                line<2>(settings.ir_right_back,
                                        settings.ir_right_back + point<2>(0, -ir->right_back)),
                            });
                        }),
          imu(  n, "/imu/data",
                [settings](const Imu::ConstPtr& imu) {
                    return -imu->angular_velocity.z;
                }, 0),
          primesense(   n, "/nord/pointcloud/wallsvector",
                        [settings](const Vector2Array::ConstPtr& rays) {
                            return *rays;
                        })
    { last_enc.timestamp = 0; };

    bool all_new()
    {
        return encoders.has_new()
            && ir_sensors.has_new()
            && imu.has_new();
    }

    ros::Publisher pub;
    Encoders last_enc;

    aggregate::custom<Encoders, Encoders, std::valarray<double>> encoders;
    aggregate::average<double, Imu> imu;
    aggregate::latest<std::array<line<2>, 6>, IRSensors> ir_sensors;
    aggregate::latest<Vector2Array, Vector2Array> primesense;
};
