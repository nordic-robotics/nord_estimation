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
#include "std_msgs/Int64.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
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
nord_messages::PoseEstimate
estimate_pose(const std::vector<std::pair<double, pose>>& particles)
{
    std::vector<std::pair<double, double>> x;
    x.reserve(particles.size());
    std::vector<std::pair<double, double>> y;
    y.reserve(particles.size());
    std::vector<double> theta;
    theta.reserve(particles.size());

    for (auto& p : particles)
    {
        x.emplace_back(p.first, p.second.x);
        y.emplace_back(p.first, p.second.y);
        theta.emplace_back(p.second.theta);
    }

    nord_messages::PoseEstimate result;
    std::tie(result.x.mean, result.x.stddev)
        = maffs::estimate_weighted_normal_distribution(x.begin(), x.end());
    std::tie(result.y.mean, result.y.stddev)
        = maffs::estimate_weighted_normal_distribution(y.begin(), y.end());
    std::tie(result.theta.mean, result.theta.stddev)
        = maffs::estimate_wrapped_normal_distribution(theta.begin(), theta.end(),
                                                      -M_PI, M_PI);

    result.stamp = ros::Time::now();
    return result;
}

auto load_params(std::string filename)
{
    std::unordered_map<std::string, double> output;

    std::ifstream file(filename);
    std::string l;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        std::string key;
        std::string value;
        std::getline(iss, key, '=');
        std::getline(iss, value, '=');

        output[key] = std::stod(value);
        std::cout << key << " = " << output[key] << std::endl;
    }

    return output;
}

int main(int argc, char** argv)
{
    using geometry_msgs::Pose2D;
    using nord_messages::PoseEstimate;
    using nord_messages::Vector2Array;
    using ras_arduino_msgs::Encoders;

    auto params = load_params(ros::package::getPath("nord_estimation") + "/data/settings.txt");

    std::array<double, 2> alpha({params["left_encoder"], params["right_encoder"]});
    unsigned int num_particles = uint(params["num_particles"]);
    bool reset = params["reset"] > 0.1;
    unsigned int resample_period = uint(params["resample_period"]);
    double bump_xy_multiplier = params["bump_xy_multiplier"];
    double bump_theta_multiplier = params["bump_theta_multiplier"];

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    //auto start_pose = pose(1.01, 2.11, M_PI);
    auto start_pose = pose(0.15, 0.23, 0);
    std::array<range_settings, 7> settings_range;
    // long range IR sensors
    settings_range[0] = settings_range[1]
                      = range_settings(0.8, params["long_sigma_hit"],
                                       params["long_lambda_short"],
                                       params["long_p_hit"], params["long_p_short"],
                                       params["long_p_max"], params["long_p_rand"]);
    // left short range IR sensors
    settings_range[2] = settings_range[3]
                      = range_settings(0.4, params["left_sigma_hit"],
                                       params["left_lambda_short"],
                                       params["left_p_hit"], params["left_p_short"],
                                       params["left_p_max"], params["left_p_rand"]);
    // right short range IR sensors
    settings_range[4] = settings_range[5]
                      = range_settings(0.4, params["right_sigma_hit"],
                                       params["right_lambda_short"],
                                       params["right_p_hit"], params["right_p_short"],
                                       params["right_p_max"], params["right_p_rand"]);

    // primesense
    settings_range[6] = range_settings(6.0, params["prime_sigma_hit"],
                                       params["prime_lambda_short"],
                                       params["prime_p_hit"], params["prime_p_short"],
                                       params["prime_p_max"], params["prime_p_rand"]);

    map maze = read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");
    forrest_filter filter(alpha, settings_range, params["imu_variance"],
                          num_particles, maze, params["uniform_fraction"],
                          params["num_primesense_rays"], start_pose);

    // kidnapped?
    if (reset)
    {
        std::cout << "help im lost" << std::endl;
        filter.reset();
    }

    // positions of IR sensors
    observer_settings settings(point<2>(-0.065, 0.0), point<2>(-0.125, 0.05),
                               point<2>(0.07, 0.02), point<2>(-0.125, 0.015),
                               point<2>(0.07, -0.013), point<2>(-0.125, -0.03),
                               0.049675f, 0.2015f);
    observer o(n, settings);

    ros::Publisher guess_pub = n.advertise<PoseEstimate>("/nord/estimation/pose_estimation",
                                                         10);

    // from forrest_filter.hpp
    GLOBAL_INITIALIZATION_DONE = true;

    auto map_msg = rviz::create_map_message(maze);
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 10);
    auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
        map_pub.publish(map_msg);
    });

    PoseEstimate guess;
    guess.x.mean = start_pose.x;
    guess.y.mean = start_pose.y;
    guess.theta.mean = start_pose.theta;
    guess.x.stddev = guess.y.stddev = guess.theta.stddev = 0.000001;
    guess.stamp = ros::Time::now();

    ros::Subscriber bump_sub(n.subscribe<nord_messages::Vector2>("/imu/bump", 1,
        [&](const nord_messages::Vector2::ConstPtr& msg) {
            filter.bump(guess, bump_xy_multiplier, bump_theta_multiplier);
            std::cout << "BUMP!" << std::endl;
    }));

    bool paused = false;
    ros::Subscriber pause_sub(n.subscribe<std_msgs::Bool>("/nord/estimation/pause", 1,
        [&](const std_msgs::Bool::ConstPtr& msg) {
            paused = msg->data;
    }));

    ros::Subscriber num_particles_sub(n.subscribe<std_msgs::Int64>(
        "/nord/estimation/set_num_particles", 1,
        [&](const std_msgs::Int64::ConstPtr& msg) {
            filter.set_num_particles(msg->data);
    }));

    ros::Rate r(20);
    ros::Time last = ros::Time::now();
    unsigned int resample_counter = 0;
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
            Vector2Array primesense;
            if (o.primesense.has_new())
                primesense = o.primesense.aggregate();

            auto dt = (current - last).toSec();
            observation obs(encoders[0], encoders[1], encoders[2], encoders[3],
                            ir_sensors, imu, primesense, dt);

            if (dt > 0.12)
            {
                std::cout << "lagging behind! " << dt << std::endl;
            }
            if (!paused)
            {
                filter.update(obs);
                last = current;
                resample_counter++;
                if (/*primesense.data.size() > 0)*/resample_counter == resample_period)
                {
                    resample_counter = 0;
                    filter.resample();
                }
            }
            guess = estimate_pose(filter.get_particles());

            if (std::isnan(guess.x.mean) || std::isnan(guess.y.mean)
             || std::isnan(guess.theta.mean))
            {
                std::cout << guess.x.mean << ' ' << guess.y.mean << '\n';
                exit(1);
            }
            guess.stamp = ros::Time::now();
            guess_pub.publish(guess);
            map_pub.publish(rviz::create_rays_message(filter.rays_to_draw));
            map_pub.publish(rviz::create_points_message(filter.get_particles()));
            map_pub.publish(rviz::create_pose_message(guess));
            map_pub.publish(rviz::create_robot_message(guess, settings));
        }

        r.sleep();
    }

    return 0;
}
