#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose2D.h"
#include "map.hpp"
#include "forrest_filter.hpp"

namespace rviz
{
    using namespace visualization_msgs;

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

    // draws particles with weight, blue are more likely than yellow
    inline Marker create_points_message(std::vector<std::pair<float, pose>> pairs)
    {
        float max = 0;
        float min = 10000000000;
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

    inline Marker create_pose_message(const pose& p)
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
        p0.x = p.x;
        p0.y = p.y;

        auto p1 = p0;
        p1.x += std::cos(p.theta) * 0.1;
        p1.y += std::sin(p.theta) * 0.1;
        arrow.points.push_back(p0);
        arrow.points.push_back(p1);
        return arrow;
    }

    // draws the robot outline
    inline Marker create_robot_message(const pose& p, const observer_settings& settings)
    {
        Marker line_list;
        line_list.id = 4;
        line_list.type = Marker::LINE_LIST;
        line_list.color.a = line_list.color.g = 1.0;
        line_list.color.r = line_list.color.b = 0.7;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "pf_robot";
        line_list.action = Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration();
        line_list.scale.x = 0.01;

        auto to_msg = [&](const point<2> p0) {
            geometry_msgs::Point p1;
            p1.x = p.x + p0.x();
            p1.y = p.y + p0.y();
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
            float sides = 0.07;
            float front = 0.075;
            float back = 0.125;

            auto p0 = point<2>(-back, -sides).rotated(p.theta);
            auto p1 = point<2>(-back, +sides).rotated(p.theta);
            auto p2 = point<2>(front, +sides).rotated(p.theta);
            auto p3 = point<2>(front, -sides).rotated(p.theta);

            enlist({p0, p1, p2, p3});
        }
        { // wheels
            auto side = settings.wheel_b / 2;
            auto radius = settings.wheel_r;
            auto width = 0.02;

            { // right wheel
                auto p0 = point<2>(+radius, -side - width).rotated(p.theta);
                auto p1 = point<2>(-radius, -side - width).rotated(p.theta);
                auto p2 = point<2>(-radius, -side + width).rotated(p.theta);
                auto p3 = point<2>(+radius, -side + width).rotated(p.theta);

                enlist({p0, p1, p2, p3});
            }
            { // left wheel
                auto p0 = point<2>(+radius, +side - width).rotated(p.theta);
                auto p1 = point<2>(-radius, +side - width).rotated(p.theta);
                auto p2 = point<2>(-radius, +side + width).rotated(p.theta);
                auto p3 = point<2>(+radius, +side + width).rotated(p.theta);

                enlist({p0, p1, p2, p3});
            }
        }
        { // IR sensors
            auto d = 0.02;
            line_list.points.push_back(to_msg(settings.ir_front.rotated(p.theta)));
            line_list.points.push_back(to_msg((settings.ir_front + point<2>(d, 0)).rotated(p.theta)));
            line_list.points.push_back(to_msg(settings.ir_back.rotated(p.theta)));
            line_list.points.push_back(to_msg((settings.ir_back - point<2>(d, 0)).rotated(p.theta)));
            line_list.points.push_back(to_msg(settings.ir_left_front.rotated(p.theta)));
            line_list.points.push_back(to_msg((settings.ir_left_front + point<2>(0, d)).rotated(p.theta)));
            line_list.points.push_back(to_msg(settings.ir_left_back.rotated(p.theta)));
            line_list.points.push_back(to_msg((settings.ir_left_back + point<2>(0, d)).rotated(p.theta)));
            line_list.points.push_back(to_msg(settings.ir_right_front.rotated(p.theta)));
            line_list.points.push_back(to_msg((settings.ir_right_front - point<2>(0, d)).rotated(p.theta)));
            line_list.points.push_back(to_msg(settings.ir_right_back.rotated(p.theta)));
            line_list.points.push_back(to_msg((settings.ir_right_back - point<2>(0, d)).rotated(p.theta)));
        }

        return line_list;
    }
}