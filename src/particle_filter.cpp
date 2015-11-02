#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose2D.h"
#include "map.hpp"
#include "forrest_filter.hpp"
#include "observer.hpp"
#include "aggregator.hpp"
#include <fstream>
#include <sstream>
#include <string>

map read_map(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<line<2>> walls;
    float min_x, min_y, max_x, max_y;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        if (l[0] == '#')
            continue;

        float x0, y0, x1, y1;
        iss >> x0 >> y0 >> x1 >> y1;
        min_x = std::min({min_x, x0, x1});
        min_y = std::min({min_y, y0, y1});
        max_x = std::max({max_x, x0, x1});
        max_y = std::max({max_y, y0, y1});
        walls.push_back(line<2>(point<2>(x0, y0), point<2>(x1, y1)));
    }

    return map(walls, min_x, min_y, max_x, max_y);
}

visualization_msgs::Marker create_map_message(const map& maze)
{
    visualization_msgs::Marker line_list;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.a = line_list.color.r = line_list.color.g = line_list.color.b = 1.0;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "pf_map";
    line_list.action = visualization_msgs::Marker::ADD;
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

visualization_msgs::Marker create_point_message(const std::vector<point<2>>& points,
                                                float r, float g, float b)
{
    visualization_msgs::Marker point_list;
    point_list.id = 1;
    point_list.type = visualization_msgs::Marker::POINTS;
    point_list.color.a = 1.0f;
    point_list.color.r = r;
    point_list.color.g = g;
    point_list.color.b = b;
    point_list.header.frame_id = "/map";
    point_list.header.stamp = ros::Time::now();
    point_list.ns = "pf_particles";
    point_list.action = visualization_msgs::Marker::ADD;
    point_list.pose.orientation.w = 1.0;
    point_list.lifetime = ros::Duration();
    point_list.scale.x = point_list.scale.y = 0.02;

    for (auto& p : points)
    {
        geometry_msgs::Point p0;
        p0.x = p.x();
        p0.y = p.y();
        p0.z = 0;
        point_list.points.push_back(p0);
    }

    return point_list;
}

visualization_msgs::Marker create_pose_message(const pose& p)
{
    visualization_msgs::Marker arrow;
    arrow.id = 3;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = arrow.color.r = 1.0;
    arrow.color.g = arrow.color.b = 0.0;
    arrow.header.frame_id = "/map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "pf_largest_weight";
    arrow.action = visualization_msgs::Marker::ADD;
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

visualization_msgs::Marker create_robot_message(const pose& p, const observer_settings& settings)
{
    visualization_msgs::Marker line_list;
    line_list.id = 4;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.a = line_list.color.g = 1.0;
    line_list.color.r = line_list.color.b = 0.7;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "pf_robot";
    line_list.action = visualization_msgs::Marker::ADD;
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
        line_list.points.push_back(to_msg(settings.ir_front.rotated(p.theta)));
        line_list.points.push_back(to_msg((settings.ir_front + point<2>(0.02, 0)).rotated(p.theta)));
        line_list.points.push_back(to_msg(settings.ir_back.rotated(p.theta)));
        line_list.points.push_back(to_msg((settings.ir_back + point<2>(-0.02, 0)).rotated(p.theta)));
    }

    return line_list;
}

int main(int argc, char** argv)
{
    using geometry_msgs::Pose2D;

    std::array<float, 6> alpha;
    for (unsigned i = 0; i < 6; i++)
    {
        alpha[i] = std::stod(argv[1 + i]);
    }

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    auto start_pose = pose(0.7, 0.2, 0);
    std::array<range_settings, 6> settings_range;
    settings_range[0] = settings_range[1] = range_settings(0.8, 0.05, 0.2, 0.6, 0.1, 0.2, 0.1);
    map maze = read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");
    forrest_filter filter(alpha, settings_range, 10000, maze, start_pose);
    //filter.reset();

    // front is 4 left 9 forward from center
    observer_settings settings(point<2>(0.09, 0.04), point<2>(-0.13, 0.03),
                               point<2>(), point<2>(),
                               point<2>(), point<2>(),
                               0.049675f, 0.2015f);
    observer o(n, settings);

    ros::Publisher guess_pub = n.advertise<Pose2D>("/nord/estimation/largest_weight", 10);

    auto map_msg = create_map_message(maze);
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
    auto map_timer = n.createTimer(ros::Duration(0.25), [&](const ros::TimerEvent& e) {
        map_pub.publish(map_msg);
    });

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        if (o.all_new())
        {
            std::valarray<float> encoders = o.encoders.aggregate();
            std::array<line<2>, 6> ir_sensors = o.ir_sensors.aggregate();
            observation obs(encoders[0], encoders[1], ir_sensors, encoders[2]);
            filter.update(obs);
        }

        auto guess = filter.estimate_largest_weight();
        Pose2D p;
        p.x = guess.second.x;
        p.y = guess.second.y;
        p.theta = guess.second.theta;
        guess_pub.publish(p);

        std::vector<point<2>> particles;
        std::transform(filter.get_particles().begin(),
                       filter.get_particles().end(),
                       std::back_inserter(particles),
                       [&](const pose& p){
                           return point<2>(p.x, p.y);
                       });
        map_pub.publish(create_point_message(particles, 0.7, 0.7, 1.0));
        map_pub.publish(create_pose_message(guess.second));
        map_pub.publish(create_robot_message(guess.second, settings));

        r.sleep();
    }

    return 0;
}
