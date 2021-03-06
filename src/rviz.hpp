#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose2D.h"
#include "nord_messages/PoseEstimate.h"
#include "map.hpp"
#include "line.hpp"
#include "forrest_filter.hpp"

namespace rviz
{
    using namespace visualization_msgs;
    using namespace nord_messages;

    // draws the map walls
    inline Marker create_map_message(const map& maze)
    {
        Marker line_list;
        line_list.id = 2;
        line_list.type = Marker::LINE_LIST;
        line_list.color.a = line_list.color.r = line_list.color.g = line_list.color.b = 1.0;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "pf_map";
        line_list.action = Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration();
        line_list.scale.x = 0.01;

        for (auto& wall : maze.get_walls())
        {
            geometry_msgs::Point p0, p1;
            p0.x = wall.start.x();
            p0.y = wall.start.y();
            p1.x = wall.end.x();
            p1.y = wall.end.y();
            p0.z = p1.z = 0;
            line_list.points.push_back(p0);
            line_list.points.push_back(p1);
        }

        return line_list;
    }

    // draws the map walls
    inline Marker create_rays_message(const std::vector<line<2>>& rays)
    {
        Marker line_list;
        line_list.id = 20;
        line_list.type = Marker::LINE_LIST;
        line_list.color.a = line_list.color.r = 1.0;
        line_list.color.g = line_list.color.b = 0.4;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "pf_rays";
        line_list.action = Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration();
        line_list.scale.x = 0.01;

        for (auto& wall : rays)
        {
            geometry_msgs::Point p0, p1;
            p0.x = wall.start.x();
            p0.y = wall.start.y();
            p1.x = wall.end.x();
            p1.y = wall.end.y();
            p0.z = p1.z = 0;
            line_list.points.push_back(p0);
            line_list.points.push_back(p1);
        }

        return line_list;
    }

    // draws particles with weight, blue are more likely than yellow
    inline Marker create_points_message(std::vector<std::pair<double, pose>> pairs)
    {
        double max = 0;
        double min = 10000000000;
        for (auto& p : pairs)
        {
            max = std::max(p.first, max);
            min = std::min(p.first, min);
        }
        for (auto& p : pairs)
        {
            p.first = (p.first - min) / (min - max);
        }

        Marker point_list;
        point_list.id = 1;
        point_list.type = Marker::POINTS;
        point_list.color.a = 1.0f;
        point_list.header.frame_id = "/map";
        point_list.header.stamp = ros::Time::now();
        point_list.ns = "pf_particles";
        point_list.action = Marker::ADD;
        point_list.pose.orientation.w = 1.0;
        point_list.lifetime = ros::Duration();
        point_list.scale.x = point_list.scale.y = 0.02;

        for (auto& p : pairs)
        {
            geometry_msgs::Point p0;
            std_msgs::ColorRGBA c0;
            p0.x = p.second.x;
            p0.y = p.second.y;
            p0.z = 0;
            point_list.points.push_back(p0);
            c0.r = c0.g = 0.7;
            c0.b = p.first;
            c0.a = 1.0;
            point_list.colors.push_back(c0);
        }

        return point_list;
    }

    inline Marker create_pose_message(const PoseEstimate& p)
    {
        Marker arrow;
        arrow.id = 3;
        arrow.type = Marker::ARROW;
        arrow.color.a = arrow.color.r = 1.0;
        arrow.color.g = arrow.color.b = 0.0;
        arrow.header.frame_id = "/map";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "pf_largest_weight";
        arrow.action = Marker::ADD;
        arrow.lifetime = ros::Duration();
        arrow.scale.x = arrow.scale.y = arrow.scale.z = 0.01;

        geometry_msgs::Point p0;
        p0.x = p.x.mean;
        p0.y = p.y.mean;

        auto p1 = p0;
        p1.x += std::cos(p.theta.mean) * 0.1;
        p1.y += std::sin(p.theta.mean) * 0.1;
        arrow.points.push_back(p0);
        arrow.points.push_back(p1);
        return arrow;
    }

    // draws the robot outline
    inline Marker create_robot_message(const PoseEstimate& p,
                                       const observer_settings& settings,
                                       bool blue = false)
    {
        Marker line_list;
        line_list.id = 4;
        line_list.type = Marker::LINE_LIST;
        line_list.color.a = line_list.color.g = 1.0;
        line_list.color.r = line_list.color.b = 0.7;
        line_list.ns = "pf_robot";
        if (blue)
        {
            line_list.ns = "pf_robot_2";
            line_list.id = 5;
            line_list.color.a = line_list.color.b = 1.0;
            line_list.color.r = line_list.color.g = 0.5;
        }
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.action = Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration();
        line_list.scale.x = 0.01;

        auto x = p.x.mean;
        auto y = p.y.mean;
        auto theta = p.theta.mean;

        auto to_msg = [&](const point<2> p0) {
            geometry_msgs::Point p1;
            p1.x = x + p0.x();
            p1.y = y + p0.y();
            p1.z = 0;
            return p1;
        };
        auto enlist = [&](const std::vector<point<2>>& ps) {
            for (size_t i = 0; i < ps.size() - 1; i++)
            {
                line_list.points.push_back(to_msg(ps[i]));
                line_list.points.push_back(to_msg(ps[i + 1]));
            }
            line_list.points.push_back(to_msg(ps.front()));
            line_list.points.push_back(to_msg(ps.back()));
        };

        { // robot outline
            double sides = 0.07;
            double front = 0.075;
            double back = 0.125;

            auto p0 = point<2>(-back, -sides).rotated(theta);
            auto p1 = point<2>(-back, +sides).rotated(theta);
            auto p2 = point<2>(front, +sides).rotated(theta);
            auto p3 = point<2>(front, -sides).rotated(theta);

            enlist({p0, p1, p2, p3});
        }
        { // wheels
            auto side = settings.wheel_b / 2;
            auto radius = settings.wheel_r;
            auto width = 0.02;

            { // right wheel
                auto p0 = point<2>(+radius, -side - width).rotated(theta);
                auto p1 = point<2>(-radius, -side - width).rotated(theta);
                auto p2 = point<2>(-radius, -side + width).rotated(theta);
                auto p3 = point<2>(+radius, -side + width).rotated(theta);

                enlist({p0, p1, p2, p3});
            }
            { // left wheel
                auto p0 = point<2>(+radius, +side - width).rotated(theta);
                auto p1 = point<2>(-radius, +side - width).rotated(theta);
                auto p2 = point<2>(-radius, +side + width).rotated(theta);
                auto p3 = point<2>(+radius, +side + width).rotated(theta);

                enlist({p0, p1, p2, p3});
            }
        }
        { // IR sensors
            auto d = 0.02;
            line_list.points.push_back(to_msg(settings.ir_front.rotated(theta)));
            line_list.points.push_back(to_msg((settings.ir_front + point<2>(d, 0)).rotated(theta)));
            line_list.points.push_back(to_msg(settings.ir_back.rotated(theta)));
            line_list.points.push_back(to_msg((settings.ir_back - point<2>(d, 0)).rotated(theta)));
            line_list.points.push_back(to_msg(settings.ir_left_front.rotated(theta)));
            line_list.points.push_back(to_msg((settings.ir_left_front + point<2>(0, d)).rotated(theta)));
            line_list.points.push_back(to_msg(settings.ir_left_back.rotated(theta)));
            line_list.points.push_back(to_msg((settings.ir_left_back + point<2>(0, d)).rotated(theta)));
            line_list.points.push_back(to_msg(settings.ir_right_front.rotated(theta)));
            line_list.points.push_back(to_msg((settings.ir_right_front - point<2>(0, d)).rotated(theta)));
            line_list.points.push_back(to_msg(settings.ir_right_back.rotated(theta)));
            line_list.points.push_back(to_msg((settings.ir_right_back - point<2>(0, d)).rotated(theta)));
        }

        return line_list;
    }
}