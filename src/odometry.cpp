#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose2D.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/Vector2Array.h"
#include "line.hpp"
#include <vector>
#include "map.hpp"
#include "forrest_filter.hpp"
#include "observer.hpp"
#include "aggregator.hpp"
#include "rviz.hpp"
#include "estimate.hpp"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <string>

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

pose motion_model_cool(const pose& state, double w1, double w2, double dt)
{
    auto l = 0.2015;
    auto r = 0.049675;
    auto Vr = w2 * r;
    auto Vl = w1 * r;
    auto w = (Vr - Vl) / l;
    auto R = (l / 2.0f) * ((Vl + Vr) / (Vr - Vl));
    if (Vr == Vl)
        R = 0; // danger zone
    auto ICCx = state.x - R * std::sin(state.theta);
    auto ICCy = state.y + R * std::cos(state.theta);

    auto A11 = std::cos(w * dt);
    auto A21 = std::sin(w * dt);
    auto A12 = -std::sin(w * dt);
    auto A22 = std::cos(w * dt);
    auto B1 = state.x - ICCx;
    auto B2 = state.y - ICCy;

    pose next = state;
    next.x = B1 * A11 + B2 * A12 + ICCx;
    next.y = B1 * A21 + B2 * A22 + ICCy;
    next.theta = wrap(state.theta + w * dt);

    return next;
}

int main(int argc, char** argv)
{
    using nord_messages::PoseEstimate;
    using nord_messages::Vector2Array;
    using ras_arduino_msgs::Encoders;

    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    //auto start_pose = pose(1.01, 2.11, M_PI);
    pose odom(0, 0, 0);

    ros::Publisher odom_pub = n.advertise<PoseEstimate>("/nord/estimation/odometry",
                                                        10);

    bool got_particle_filter = false;
    ros::Subscriber particle_sub(n.subscribe<nord_messages::PoseEstimate>(
        "/nord/estimation/pose_estimation", 1,
        [&](const nord_messages::PoseEstimate::ConstPtr& msg)
        {
            got_particle_filter = true;
            odom.x = msg->x.mean;
            odom.y = msg->y.mean;
            odom.theta = msg->theta.mean;
        }
    ));

    PoseEstimate odom_est;
    ros::Subscriber enc_sub(n.subscribe<Encoders>("/arduino/encoders", 10,
        [&](const Encoders::ConstPtr& msg) {
            if (!got_particle_filter)
                return;

            double delta_time = msg->timestamp / 1000.0f;
            double estimated_w1 = (-msg->delta_encoder1 * 2 * M_PI / delta_time) / 360.0f;
            double estimated_w2 = (-msg->delta_encoder2 * 2 * M_PI / delta_time) / 360.0f;
            odom = motion_model_cool(odom, estimated_w1, estimated_w2, delta_time);

            odom_est.x.mean = odom.x;
            odom_est.y.mean = odom.y;
            odom_est.theta.mean = odom.theta;
            odom_est.stamp = ros::Time::now();
            odom_pub.publish(odom_est);
    }));

    auto reset_timer = n.createTimer(ros::Duration(2), [&](const ros::TimerEvent& e) {
        got_particle_filter = false;
    });

    ros::spin();

    return 0;
}
