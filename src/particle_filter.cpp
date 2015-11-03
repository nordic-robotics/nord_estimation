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
    float min_x = 0, min_y = 0, max_x = 0, max_y = 0;
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

visualization_msgs::Marker create_point_message(std::vector<std::pair<float, pose>> pairs)
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

    visualization_msgs::Marker point_list;
    point_list.id = 1;
    point_list.type = visualization_msgs::Marker::POINTS;
    point_list.color.a = 1.0f;
    point_list.header.frame_id = "/map";
    point_list.header.stamp = ros::Time::now();
    point_list.ns = "pf_particles";
    point_list.action = visualization_msgs::Marker::ADD;
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
    float long_sigma_hit = std::stod(argv[7]);
    float long_lambda_short = std::stod(argv[8]);
    float long_p_hit = std::stod(argv[9]);
    float long_p_short = std::stod(argv[10]);
    float long_p_max = std::stod(argv[11]);
    float long_p_rand = std::stod(argv[12]);
    float short_sigma_hit = std::stod(argv[13]);
    float short_lambda_short = std::stod(argv[14]);
    float short_p_hit = std::stod(argv[15]);
    float short_p_short = std::stod(argv[16]);
    float short_p_max = std::stod(argv[17]);
    float short_p_rand = std::stod(argv[18]);
    int num_particles = std::stoi(argv[19]);
    bool reset = std::string(argv[20]) == "reset";

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    auto start_pose = pose(0.7, 0.2, 0);
    std::array<range_settings, 6> settings_range;
    settings_range[0] = settings_range[1]
                      = range_settings(0.7, long_sigma_hit, long_lambda_short,
                                       long_p_hit, long_p_short,
                                       long_p_max, long_p_rand);
    settings_range[2] = settings_range[3]
                      = settings_range[4]
                      = settings_range[5]
                      = range_settings(0.3, short_sigma_hit, short_lambda_short,
                                       short_p_hit, short_p_short,
                                       short_p_max, short_p_rand);

    map maze = read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");
    forrest_filter filter(alpha, settings_range, num_particles, maze, start_pose);

    if (reset)
        filter.reset();

    observer_settings settings(point<2>(0.09, 0.04), point<2>(-0.13, 0.03),
                               point<2>(0.07, 0.09), point<2>(-0.06, 0.09),
                               point<2>(0.07, -0.09), point<2>(-0.06, -0.09),
                               0.049675f, 0.2015f);
    observer o(n, settings);

    ros::Publisher guess_pub = n.advertise<Pose2D>("/nord/estimation/largest_weight", 10);

    auto map_msg = create_map_message(maze);
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
    auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
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

        map_pub.publish(create_point_message(filter.get_sampled_particles()));
        map_pub.publish(create_pose_message(guess.second));
        map_pub.publish(create_robot_message(guess.second, settings));

        r.sleep();
    }

    return 0;
}
