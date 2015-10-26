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
    line_list.ns = "particle_filter";
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

int main(int argc, char** argv)
{
    using geometry_msgs::Pose2D;

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    map maze = read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");
    forrest_filter filter(std::array<float, 4>{0, 0, 0, 0},
                          std::array<range_settings, 6>(),
                          1000, &maze, pose(0, 0, 0));

    observer o(n, observer_settings(point<2>(), point<2>(),
                                    point<2>(), point<2>(),
                                    point<2>(), point<2>(),
                                    0.049675f, 0.2015f));

    ros::Publisher guess_pub = n.advertise<Pose2D>("/nord/estimation/largest_weight", 10);

    auto map_msg = create_map_message(maze);
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
    auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
        map_pub.publish(map_msg);
    });

    pose odometry_prev;
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        pose odometry = o.encoders.aggregate();
        std::array<line<2>, 6> ir_sensors = o.ir_sensors.aggregate();
        observation obs(odometry, odometry_prev, ir_sensors);
        filter.update(obs);

        auto guess = filter.estimate_largest_weight();
        Pose2D p;
        p.x = guess.second.x;
        p.y = guess.second.y;
        p.theta = guess.second.theta;
        guess_pub.publish(p);

        r.sleep();
    }

    return 0;
}
