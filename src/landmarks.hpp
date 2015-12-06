#pragma once

#include <experimental/optional>
#include <string>
#include <valarray>
#include "point.hpp"
#include "map.hpp"
#include "argmax.hpp"
#include "nord_messages/Features.h"
#include "sensor_msgs/Image.h"
#include <fstream>



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
// ACHTUNG!!is this where this should be done?
map maze =  read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");


namespace
{
    point<2> relative_to_world(const point<2>& relative_location,
                               const std::valarray<double>& pose)
    {
        auto c = pose[6];
        auto s = pose[7];

        return point<2>(pose[0], pose[1])
             + point<2>(relative_location.x() * c - relative_location.y() * s,
                        relative_location.x() * s + relative_location.y() * c);
    }
}

class landmark
{
public:
    landmark(const point<2>& relative_location, const std::valarray<double>& robot_pose,
             size_t id, const nord_messages::Features& features,
                    const int xi, const int yi, const sensor_msgs::Image& shot)
        : id(id)
    {
        update(relative_to_world(relative_location, robot_pose), robot_pose, features, xi, yi, shot);
    };

    double distance_to(const point<2>& world_location) const
    {
        return (mean - world_location).length();
    }



    point<2> pushThroughWalls(const point<2>& world_location, const std::valarray<double>& robot_pose) {
        // 
        point<2> robot_point = point<2>(robot_pose[0],robot_pose[1]);
        // Ray from the robot to the object
        line<2> ray = line<2>(robot_point, world_location);
        auto p = maze.raycast(ray);
        point<2> final_location;
        // If the ray collided we push the coordinate towards the robot
        if (p) {
            final_location = p.value() - (ray.end - ray.start).normalized() * 0.02;
        } else {
            final_location = world_location;
        }
        return final_location;
    }
    

    point<2> update(const point<2>& world_location, const std::valarray<double>& robot_pose,
                    const nord_messages::Features& features,
                    const int xi, const int yi, const sensor_msgs::Image& shot)
    {
        point<2> new_world_location = pushThroughWalls(world_location, robot_pose);
        data.emplace_back(new_world_location, robot_pose);
        aggregated_features.push_back(features);

        float x = 0;
        float y = 0;
        for (auto& d : data)
        {
            x += d.first.x();
            y += d.first.y();
        }

        mean = point<2>(x / data.size(), y / data.size());

        // Update the moneyshot and imagecoordinates of the object
        if ( moneyshot.data.size() == 0 ) {
            moneyshot = shot;
        } else {
            // calculate the distance from the center
            int h = shot.height/2;
            int w = shot.width/2;
            int r_new = (h-yi)*(h-yi) + (w-xi)*(w-xi);
            int r_old = (h-yp)*(h-yp) + (w-xp)*(w-xp);
            // update if the new image is more centered
            if (r_new < r_old) {
                moneyshot = shot;
                xp = xi;
                yp = yi;
            }
        }

        return mean;
    }

    void add_features(const nord_messages::Features& features)
    {
        aggregated_features.push_back(features);
    }

    std::pair<size_t, size_t> get_num_features() const
    {
        std::pair<size_t, size_t> num(0, 0);
        for (auto& f : aggregated_features)
        {
            num.first++;
            if (f.vfh.size() != 0)
            {
                num.second++;
            }
        }
        return num;
    }

    size_t get_id() const { return id; }
    const point<2>& get_mean() const { return mean; }
    const std::vector<nord_messages::Features>& get_aggregated_features() const
    {
        return aggregated_features;
    }

    int get_xp() const { return xp;}
    int get_yp() const { return yp;}
    const sensor_msgs::Image & get_moneyshot() const { return moneyshot; }


private:
    point<2> mean;
    std::vector<std::pair<point<2>, std::valarray<double>>> data;
    size_t id;
    std::vector<nord_messages::Features> aggregated_features;
    int xp, yp;
    sensor_msgs::Image moneyshot;
};

class landmarks
{
public:
    landmarks(double max_distance)
        : max_distance(max_distance) { };

    landmark& add(const point<2>& relative_location,
                  const std::valarray<double>& robot_pose,
                  const nord_messages::Features& features,
                  const int xp, const int yp,
                  const sensor_msgs::Image& moneyshot)
    {
        auto world_location = relative_to_world(relative_location, robot_pose);

        if (objects.size() == 0)
        {
            objects.emplace_back(relative_location, robot_pose, objects.size(), features, xp, yp, moneyshot);
            return objects.back();
        }

        auto res = studd::argmax(objects.begin(), objects.end(),
            [&](const landmark& object) {
                auto dist = object.distance_to(world_location);
                if (dist > max_distance)
                    return -1000.0;

                return -dist;
            });

        if (res.second == -1000.0)
        {
            objects.emplace_back(relative_location, robot_pose, objects.size(), features, xp, yp,  moneyshot);
            return objects.back();
        }
        else
        {
            res.first->update(world_location, robot_pose, features, xp, yp, moneyshot);
            return *res.first;
        }
    }

    const std::vector<landmark>& get_objects() const { return objects; };

private:
    std::vector<landmark> objects;
    double max_distance;
};
