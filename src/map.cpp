#include "map.hpp"

opt_point<2> map::raycast(line<2> ray) const
{
    auto min_distance = std::numeric_limits<float>::infinity();
    opt_point<2> closest;
    for (auto& wall : walls)
    {
        float denominator = ((ray.end.x() - ray.start.x())
                             * (wall.end.y() - wall.start.y()))
                          - ((ray.end.y() - ray.start.y())
                             * (wall.end.x() - wall.start.x()));
        float numerator1 = ((ray.start.y() - wall.start.y())
                             * (wall.end.x() - wall.start.x()))
                          - ((ray.start.x() - wall.start.x())
                             * (wall.end.y() - wall.start.y()));
        float numerator2 = ((ray.start.y() - wall.start.y())
                             * (ray.end.x() - ray.start.x()))
                          - ((ray.start.x() - wall.start.x())
                             * (ray.end.y() - ray.start.y()));

        if (denominator == 0 && numerator1 == 0 && numerator2 == 0)
            continue;

        float r = numerator1 / denominator;
        float s = numerator2 / denominator;

        if (!((r >= 0 && r <= 1) && (s >= 0 && s <= 1)))
            continue;

        auto p = (ray.end - ray.start) * r;
        auto distance = p.length();
        if (distance < min_distance)
        {
            min_distance = distance;
            closest = ray.start + p;
        }
    }

    return closest;
}

bool map::contains(point<2> p) const
{
    return p.x() >= min_x
        && p.x() <= max_x
        && p.y() >= min_y
        && p.y() <= max_y;
}

const std::vector<line<2>>& map::get_walls() const
{
    return walls;
}

float map::get_area() const
{
    return (max_x - min_x) * (max_y - min_y);
}
