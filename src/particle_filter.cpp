#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
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

int main(int argc, char** argv)
{
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

        r.sleep();
    }

    return 0;
}
