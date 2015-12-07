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
        auto c = pose[6];
        auto s = pose[7];

        return point<2>(pose[0], pose[1])
             + point<2>(relative_location.x() * c - relative_location.y() * s,
                        relative_location.x() * s + relative_location.y() * c);
    }

    float ccw(const point<2>& p1, const point<2>& p2, const point<2>& p3) {
        return (p2.x() - p1.x())*(p3.y() - p1.y()) - (p2.y() - p1.y())*(p3.x() - p1.x());
    }

    float polarangle(const point<2>& diff) {
        return std::atan2(diff.y(),diff.x());
    }

    std::vector<nord_messages::Vector2>
    grayham(const std::vector<nord_messages::Vector2>& input){
        //change format to points
        std::vector<point<2>> points;
        std::transform(input.begin(), input.end(), std::back_inserter(points),
            [&](const nord_messages::Vector2& p) {
                return point<2>(p.x, p.y);
            });

        int n=points.size();
        //will always be convex 
        // if (n<5){
        //     return input;
        // }
        //find min in y
        auto res = studd::argmax(points.begin(), points.end(),
        [&](const point<2>& elem) {
                return -elem.y();
            });
        if (res.first == points.end())
        {
            std::cout << "argmax failed" << std::endl;
            exit(1);
        }

        //swap to points[1]
        std::swap(*res.first, points[1]);



        //sort by polar angle with points[1]
        std::sort(points.begin(), points.end(), [&](const point<2>& a, const point<2>& b) {
            return polarangle(a-points[1]) < polarangle(b-points[1]);
        });

        //sentinel
        points.insert(points.begin(),points.back());


        int m=1;
        for (uint i=2;i<=n;i++){
            while (ccw(points[m-1],points[m],points[i]) <= 0){
                if (m>1){
                    m -=1;
                }
                else if (i==n)
                {
                    break;
                }
                else{
                    i+=1;
                }
            }
            m+=1;
            if (m >= points.size())
            {
                std::cout << "shits fucked yo" << std::endl;
                exit(1);
            }
            std::swap(points[m], points[i]);    
        }

        //works




        std::vector<nord_messages::Vector2> output;
        for (int i = 0; i < m; i++)
        {
            nord_messages::Vector2 o;
            o.x=points[i].x();
            o.y=points[i].y();
            output.push_back(o);
        }

        return output;
    }

    int orientation(const point<2>& p, const point<2>& q, const point<2>& r)
    {
        int val = (q.y() - p.y()) * (r.x() - q.x())
                - (q.x() - p.x()) * (r.y() - q.y());

        if (val == 0)
            return 0;
        return val > 0 ? 1 : 2;
    }

    nord_messages::Vector2 to_vec(const point<2> p)
    {
        nord_messages::Vector2 v;
        v.x = p.x();
        v.y = p.y();
        return v;
    }

    std::vector<nord_messages::Vector2>
    jarviss(const std::vector<nord_messages::Vector2>& input){
        //will always be convex 
        if (input.size()<3)
            return input;

        //change format to points
        std::vector<point<2>> points;
        std::transform(input.begin(), input.end(), std::back_inserter(points),
            [&](const nord_messages::Vector2& p) {
                return point<2>(p.x, p.y);
            });

        //initalize result
        std::vector<nord_messages::Vector2> hull;

        size_t l = 0;
        for (size_t i = 1; i < points.size(); i++)
        {
            if (points[i].x() < points[l].x())
                l = i;
        }

        size_t p = l;
        size_t q = 0;

        do
        {
            hull.push_back(to_vec(points[q]));
            q = (p + 1) % points.size();
            for (size_t i = 0; i < points.size(); i++)
            {
                if (orientation(points[p], points[i], points[q]) == 2)
                q = i;
            }
                p = q;

        } while (p != l);
        return hull;
    }

    std::vector<nord_messages::Vector2>
    suckysquare(const std::vector<nord_messages::Vector2>& input){
        //will always be convex 
        // if (input.size()<5)
        //     return input;
        float xmax=0.0f;
        float xmin=10000.0f;
        float ymax=0.0f;
        float ymin=10000.0f;
        for (uint k=0; k<input.size(); k++){
            if(input[k].x>xmax)
                xmax=input[k].x;
            if(input[k].x<xmin)
                xmin=input[k].x;
            if(input[k].y>ymax)
                ymax=input[k].y;
            if(input[k].y<ymin)
                ymin=input[k].y;
        }
        //initalize result
        std::vector<nord_messages::Vector2> hull;
        nord_messages::Vector2 punkt1;
        punkt1.x=xmax;
        punkt1.y=ymax;
        hull.push_back(punkt1);
        nord_messages::Vector2 punkt2;
        punkt2.x=xmax;
        punkt2.y=ymin;
        hull.push_back(punkt2);
        nord_messages::Vector2 punkt3;
        punkt3.x=xmin;
        punkt3.y=ymin;
        hull.push_back(punkt3);
        nord_messages::Vector2 punkt4;
        punkt4.x=xmin;
        punkt4.y=ymax;
        hull.push_back(punkt4);
        return hull;
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
                 const std::vector<nord_messages::Vector2>& hull)
    {
        data.emplace_back(world_location, robot_pose);
        std::vector<nord_messages::Vector2> v;
        std::transform(hull.begin(), hull.end(), std::back_inserter(v),
            [&](nord_messages::Vector2 v) {
                auto p = relative_to_world(point<2>(v.x, v.y), robot_pose);
                v.x = p.x();
                v.y = p.y();
                return v;
            });
        hull_size++;
        //auto temp = suckysquare(aggregated_hull);
        //aggregated_hull = temp;
        aggregated_hull=suckysquare(v);
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

    std::vector<nord_messages::Vector2>& get_hull()
    {
        return aggregated_hull;
    }


    size_t get_id() const { return id; }

    const point<2>& get_mean() const { return mean; }
    void set_mean(point<2> meanin) {  mean=meanin; }

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
    std::vector<debris>& get_objects() { return objects; };


private:
    std::vector<debris> objects;
    double max_distance;
};
