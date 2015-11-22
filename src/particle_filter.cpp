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
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        if (l[0] == '#')
            continue;

        double x0, y0, x1, y1;
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
nord_messages::PoseEstimate estimate_pose(const std::vector<std::pair<double, pose>>& particles) 
{
    std::vector<std::pair<double, double>> x;
    x.reserve(particles.size());
    std::vector<std::pair<double, double>> y;
    y.reserve(particles.size());
    std::vector<std::pair<double, double>> theta;
    theta.reserve(particles.size());

    for (auto& p : particles)
    {
        x.emplace_back(p.first, p.second.x);
        y.emplace_back(p.first, p.second.y);
        theta.emplace_back(p.first, p.second.theta);
    }

    nord_messages::PoseEstimate result;
    std::tie(result.x.mean, result.x.stddev)
        = maffs::estimate_weighted_normal_distribution(x.begin(), x.end());
    std::tie(result.y.mean, result.y.stddev)
        = maffs::estimate_weighted_normal_distribution(y.begin(), y.end());
    std::tie(result.theta.mean, result.theta.stddev)
        = maffs::estimate_weighted_wrapped_normal_distribution(theta.begin(), theta.end(),
                                                               -M_PI, M_PI);

    result.stamp = ros::Time::now();
    return result;
}

int main(int argc, char** argv)
{
    using geometry_msgs::Pose2D;
    using nord_messages::PoseEstimate;

    std::array<double, 2> alpha;
    double long_sigma_hit = std::stod(argv[7]);
    double long_lambda_short = std::stod(argv[8]);
    double long_p_hit = std::stod(argv[9]);
    double long_p_short = std::stod(argv[10]);
    double long_p_max = std::stod(argv[11]);
    double long_p_rand = std::stod(argv[12]);
    double short_sigma_hit = std::stod(argv[13]);
    double short_lambda_short = std::stod(argv[14]);
    double short_p_hit = std::stod(argv[15]);
    double short_p_short = std::stod(argv[16]);
    double short_p_max = std::stod(argv[17]);
    double short_p_rand = std::stod(argv[18]);
    double imu_variance = std::stod(argv[19]);
    int num_particles = std::stoi(argv[20]);
    bool reset = std::string(argv[21]) == "reset";

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    auto start_pose = pose(0.795, 0.23, 0.0);
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
    forrest_filter filter(alpha, settings_range, imu_variance, num_particles, maze, start_pose);

    // kidnapped?
    if (reset)
        filter.reset();

    // positions of IR sensors
    observer_settings settings(point<2>(0.09, 0.04), point<2>(-0.13, 0.03),
                               point<2>(0.07, 0.09), point<2>(-0.06, 0.09),
                               point<2>(0.07, -0.09), point<2>(-0.06, -0.09),
                               0.049675f, 0.2015f);
    observer o(n, settings);

    ros::Publisher guess_pub = n.advertise<PoseEstimate>("/nord/estimation/pose_estimation", 1);

    auto map_msg = rviz::create_map_message(maze);
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
    auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
        map_pub.publish(map_msg);
    });

    ros::Rate r(10);
    ros::Time last = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();
        // only update when all sensors have something to contribute
        if (o.all_new())
        {
            auto current = ros::Time::now();
            std::valarray<double> encoders = o.encoders.aggregate();
            std::array<line<2>, 6> ir_sensors = o.ir_sensors.aggregate();
            double imu = o.imu.aggregate();
            observation obs(encoders[0], encoders[1], ir_sensors, imu,
                            (current - last).toSec());
            filter.update(obs);
            last = current;
        }

        auto guess = estimate_pose(filter.get_particles());
        guess_pub.publish(guess);

        map_pub.publish(rviz::create_points_message(filter.get_particles()));
        map_pub.publish(rviz::create_pose_message(guess));
        map_pub.publish(rviz::create_robot_message(guess, settings));

        r.sleep();
    }

    return 0;
}
