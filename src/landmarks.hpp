#pragma once

#include <experimental/optional>
#include <string>
#include <valarray>
#include "point.hpp"
#include "argmax.hpp"
#include "nord_messages/Features.h"

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
             size_t id, const nord_messages::Features& features)
        : id(id)
    {
        update(relative_to_world(relative_location, robot_pose), robot_pose, features);
    };

    float distance_to(const point<2>& world_location) const
    {
        return (mean - world_location).length();
    }

    point<2> update(const point<2>& world_location, const std::valarray<float>& robot_pose,
                    const nord_messages::Features& features)
    {
        data.emplace_back(world_location, robot_pose);
        aggregated_features.push_back(features);

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

    void add_features(const nord_messages::Features& features)
    {
        aggregated_features.push_back(features);
    }

    size_t get_id() const { return id; }
    const point<2>& get_mean() const { return mean; }
    const std::vector<nord_messages::Features>& get_aggregated_features() const
    {
        return aggregated_features;
    }

private:
    point<2> mean;
    std::vector<std::pair<point<2>, std::valarray<float>>> data;
    size_t id;
    std::vector<nord_messages::Features> aggregated_features;
};

class landmarks
{
public:
    landmarks(float max_distance)
        : max_distance(max_distance) { };

    landmark& add(const point<2>& relative_location,
                  const std::valarray<float>& robot_pose,
                  const nord_messages::Features& features)
    {
        auto world_location = relative_to_world(relative_location, robot_pose);

        if (objects.size() == 0)
        {
            objects.emplace_back(relative_location, robot_pose, objects.size(), features);
            return objects.back();
        }

        auto res = studd::argmax(objects.begin(), objects.end(),
            [&](const landmark& object) {
                auto dist = object.distance_to(world_location);
                if (dist > max_distance)
                    return -1000.0f;

                return -dist;
            });

        if (res.second == -1000.0f)
        {
            objects.emplace_back(relative_location, robot_pose, objects.size(), features);
            return objects.back();
        }
        else
        {
            res.first->update(world_location, robot_pose, features);
            return *res.first;
        }
    }

    const std::vector<landmark>& get_objects() const { return objects; }

private:
    std::vector<landmark> objects;
    float max_distance;
};