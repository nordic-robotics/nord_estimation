#pragma once

#include <experimental/optional>
#include <string>
#include <valarray>
#include "point.hpp"
#include "argmax.hpp"
#include "nord_messages/Features.h"
#include "sensor_msgs/Image.h"

namespace
{
    point<2> relative_to_world(const point<2>& relative_location,
                               const std::valarray<double>& pose)
    {
        auto c = std::cos(pose[2]);
        auto s = std::sin(pose[2]);

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

    point<2> update(const point<2>& world_location, const std::valarray<double>& robot_pose,
                    const nord_messages::Features& features,
                    const int xi, const int yi, const sensor_msgs::Image& shot)
    {
        data.emplace_back(world_location, robot_pose);
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
        if ( moneyshot == NULL ) {
            *moneyshot = shot;
        } else {
            // calculate the distance from the center
            int h = shot.height/2;
            int w = shot.width/2;
            int r_new = (h-yi)*(h-yi) + (w-xi)*(w-xi);
            int r_old = (h-yp)*(h-yp) + (w-xp)*(w-xp);
            // update if the new image is more centered
            if (r_new < r_old) {
                *moneyshot = shot;
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
    sensor_msgs::Image * get_moneyshot() const { return moneyshot; }


private:
    point<2> mean;
    std::vector<std::pair<point<2>, std::valarray<double>>> data;
    size_t id;
    std::vector<nord_messages::Features> aggregated_features;
    int xp, yp;
    sensor_msgs::Image * moneyshot;
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
