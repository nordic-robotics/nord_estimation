#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose2D.h"
#include "nord_messages/PoseEstimate.h"
#include "map.hpp"
#include "forrest_filter.hpp"
#include "observer.hpp"
#include "aggregator.hpp"
#include "rviz.hpp"
#include "estimate.hpp"
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

// (mean, variance) for each dimension
nord_messages::PoseEstimate estimate_pose(const std::vector<pose>& particles) 
{
    std::vector<float> x;
    x.reserve(particles.size());
    std::vector<float> y;
    y.reserve(particles.size());
    std::vector<float> theta;
    theta.reserve(particles.size());

    for (auto& p : particles)
    {
        x.push_back(p.x);
        y.push_back(p.y);
        theta.push_back(p.theta);
    }

    nord_messages::PoseEstimate result;
    std::tie(result.x.mean, result.x.stddev)
        = maffs::estimate_normal_distribution(x.begin(), x.end());
    std::tie(result.y.mean, result.y.stddev)
        = maffs::estimate_normal_distribution(y.begin(), y.end());
    std::tie(result.theta.mean, result.theta.stddev)
        = maffs::estimate_wrapped_normal_distribution(theta.begin(), theta.end(), -M_PI, M_PI);

    result.stamp = ros::Time::now();
    return result;
}

int main(int argc, char** argv)
{
    using geometry_msgs::Pose2D;
    using nord_messages::PoseEstimate;

    std::array<float, 6> alpha;
    for (unsigned i = 0; i < 6; i++)
    {
        alpha[i] = std::stod(argv[1 + i]);
    }
    // nord_estimation particle_filter 0.2 0.2 0.2 0.2 0.2 0.2 0.2 5 0.95 0.01 0.01 0.03 0.1 9 0.4 0.1 0.25 0.25 10000 reset
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
    // long range IR sensors
    settings_range[0] = settings_range[1]
                      = range_settings(0.7, long_sigma_hit, long_lambda_short,
                                       long_p_hit, long_p_short,
                                       long_p_max, long_p_rand);
    // short range IR sensors
    settings_range[2] = settings_range[3]
                      = settings_range[4]
                      = settings_range[5]
                      = range_settings(0.3, short_sigma_hit, short_lambda_short,
                                       short_p_hit, short_p_short,
                                       short_p_max, short_p_rand);

    map maze = read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");
    forrest_filter filter(alpha, settings_range, num_particles, maze, start_pose);

    // kidnapped?
    if (reset)
        filter.reset();

    // positions of IR sensors
    observer_settings settings(point<2>(0.09, 0.04), point<2>(-0.13, 0.03),
                               point<2>(0.07, 0.09), point<2>(-0.06, 0.09),
                               point<2>(0.07, -0.09), point<2>(-0.06, -0.09),
                               0.049675f, 0.2015f);
    observer o(n, settings);

    ros::Publisher guess_pub = n.advertise<PoseEstimate>("/nord/estimation/gaussian", 10);

    auto map_msg = rviz::create_map_message(maze);
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
    auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
        map_pub.publish(map_msg);
    });

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        // only update when all sensors have something to contribute
        if (o.all_new())
        {
            std::valarray<float> encoders = o.encoders.aggregate();
            std::array<line<2>, 6> ir_sensors = o.ir_sensors.aggregate();
            observation obs(encoders[0], encoders[1], ir_sensors, encoders[2]);
            filter.update(obs);
        }

        auto guess = estimate_pose(filter.get_particles());
        guess_pub.publish(guess);

        map_pub.publish(rviz::create_points_message(filter.get_sampled_particles()));
        map_pub.publish(rviz::create_pose_message(guess));
        map_pub.publish(rviz::create_robot_message(guess, settings));

        r.sleep();
    }

    return 0;
}
