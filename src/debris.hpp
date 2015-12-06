#pragma once

#include <experimental/optional>
#include <string>
#include <valarray>
#include "point.hpp"
#include "argmax.hpp"

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

    float ccw(const point<2>& p1, const point<2>& p2, const point<2>& p3) const {
        return (p2.x() - p1.x())*(p3.y() - p1.y()) - (p2.y() - p1.y())*(p3.x() - p1.x());
    }

    float polarangle(const point<2>& diff) const {
        return std::atan2(diff.y(),diff.x());
    }

    std::vector<nord_messages::Vector2>grayham(std::vector<nord_messages::Vector2> input){
        //find min in y
        std::cout << input.size() << std::endl;

        auto res = studd::argmax(input.begin(), input.end(),
        [&](const nord_messages::Vector2& elem) {
                return elem.y;
            });
        if (res.first == input.end())
        {
            std::cout << "argmax failed" << std::endl;
            exit(1);
        }
        //swap to input[1]
        std::swap(*res.first, input[1]);

        std::vector<point<2>> points;
        std::transform(input.begin(), input.end(), std::back_inserter(points),
            [&](const nord_messages::Vector2& p) {
                return point<2>(p.x, p.y);
            });


        //sort by polar angle with input[1]
        std::sort(points.begin(), points.end(), [&](const point<2>& a, const point<2>& b) {
            return polarangle(a-points[1]) < polarangle(b-points[1]);
        });


        points.insert(points.begin(),points.back());



        int m=1;
        for (uint i=2;i<points.size();i++){
            while (ccw(points[m-1],points[m],points[i]) <= 0){
                if (m>1){
                    m -=1;
                }
                else if (i==points.size()-1)
                {
                    break;
                }
                else{
                    i+=1;
                }
            }
            m+=1;
            std::swap(points[m], points[i]);    
        }

        //works

    std::vector<nord_messages::Vector2> output;
    std::transform(points.begin(), points.begin()+(m - 1), std::back_inserter(output),
        [&](const point<2>& p) {
            nord_messages::Vector2 o;
            o.x=p.x();
            o.y=p.y();
            return o;
        });
    return output;
    }
}


class debris
{
public:
    debris(const point<2>& relative_location, const std::valarray<double>& robot_pose,
             size_t id, const std::vector<nord_messages::Vector2>& hull)
        : id(id)
    {
        update(relative_to_world(relative_location, robot_pose), robot_pose, hull);
    };

    double distance_to(const point<2>& world_location) const
    {
        return (mean - world_location).length();
    }

    point<2> update(const point<2>& world_location, const std::valarray<double>& robot_pose,
                 std::vector<nord_messages::Vector2> hull)
    {
        data.emplace_back(world_location, robot_pose);
        std::move(hull.begin(), hull.end(), std::back_inserter(aggregated_hull));
        hull_size++;
        aggregated_hull=grayham(aggregated_hull);
        float x = 0;
        float y = 0;
        for (auto& d : data)
        {
            x += d.first.x();
            y += d.first.y();
        }

        mean = point<2>(x / data.size(), y / data.size());

        return mean;
    }

    uint get_num_features() const
    {
        return hull_size;
    }

    std::vector<nord_messages::Vector2> get_hull() const
    {
        return aggregated_hull;
    }

    size_t get_id() const { return id; }

    const point<2>& get_mean() const { return mean; }

    const std::vector<nord_messages::Vector2>& get_aggregated_hull() const
    {
        return aggregated_hull;
    }

private:
    point<2> mean;
    std::vector<std::pair<point<2>, std::valarray<double>>> data;
    size_t id;
    std::vector<nord_messages::Vector2> aggregated_hull;
    uint hull_size=0;

};

class debris2
{
public:
    debris2(double max_distance)
        : max_distance(max_distance) { };

    debris& add(const point<2>& relative_location,
                  const std::valarray<double>& robot_pose,
                  const std::vector<nord_messages::Vector2> hulls)
    {
        auto world_location = relative_to_world(relative_location, robot_pose);

        if (objects.size() == 0)
        {
            objects.emplace_back(relative_location, robot_pose, objects.size(), hulls);
            return objects.back();
        }

        auto res = studd::argmax(objects.begin(), objects.end(),
            [&](const debris& object) {
                auto dist = object.distance_to(world_location);
                if (dist > max_distance)
                    return -1000.0;

                return -dist;
            });

        if (res.second == -1000.0)
        {
            objects.emplace_back(relative_location, robot_pose, objects.size(), hulls);
            return objects.back();
        }
        else
        {
            res.first->update(world_location, robot_pose, hulls);
            return *res.first;
        }
    }

    const std::vector<debris>& get_objects() const { return objects; };

private:
    std::vector<debris> objects;
    double max_distance;
};
