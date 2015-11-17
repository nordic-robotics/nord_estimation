#pragma once

#include <experimental/optional>
#include <string>
#include <valarray>
#include "point.hpp"
#include "argmax.hpp"

namespace
{
    point<2> relative_to_world(const point<2>& relative_location,
                               const std::valarray<float>& pose)
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
    landmark(const point<2>& relative_location, const std::valarray<float>& robot_pose,
             const std::string& type = "")
        : type(type)
    {
        update(relative_to_world(relative_location, robot_pose), robot_pose);
    };

    float distance_to(const point<2>& world_location) const
    {
        return (mean - world_location).length();
    }

    point<2> update(const point<2>& world_location, const std::valarray<float>& robot_pose)
    {
        data.emplace_back(world_location, robot_pose);

        float sum_confidence_x = 0;
        float sum_confidence_y = 0;
        for (auto& d : data)
        {
            sum_confidence_x += 1.0f / d.second[3];
            sum_confidence_y += 1.0f / d.second[4];
        }

        float x = 0;
        float y = 0;
        for (auto& d : data)
        {
            x += (d.second[0] / d.second[3]) / sum_confidence_x;
            y += (d.second[1] / d.second[4]) / sum_confidence_y;
        }

        mean = point<2>(x, y);
        return mean;
    }

    const point<2>& get_mean() const { return mean; }
    const std::string& get_type() const { return type; }

private:
    point<2> mean;
    std::vector<std::pair<point<2>, std::valarray<float>>> data;
    std::string type;
};

class landmarks
{
public:
    landmarks(float max_distance)
        : max_distance(max_distance) { };

    std::pair<bool, landmark> add(const point<2>& relative_location,
                                  const std::valarray<float>& robot_pose,
                                  const std::string& type = "")
    {
        auto world_location = relative_to_world(relative_location, robot_pose);

        if (objects.size() == 0)
        {
            objects.emplace_back(relative_location, robot_pose, type);
            return std::make_pair(true, objects.back());
        }

        auto res = studd::argmax(objects.begin(), objects.end(),
            [&](const landmark& object) {
                auto dist = object.distance_to(world_location);
                if (dist > max_distance
                    || (type != "" && type != object.get_type()))
                    return -1000.0f;

                return -dist;
            });

        if (res.second == -1000.0f)
        {
            objects.emplace_back(relative_location, robot_pose, type);
            return std::make_pair(true, objects.back());
        }
        else
        {
            res.first->update(world_location, robot_pose);
            return std::make_pair(false, *res.first);
        }
    }

private:
    std::vector<landmark> objects;
    float max_distance;
};