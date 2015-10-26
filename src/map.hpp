#pragma once

#include <vector>
#include <experimental/optional>
#include <limits>
#include <iostream>
#include "point.hpp"
#include "line.hpp"

template<unsigned int d>
using opt_point = std::experimental::optional<point<d>>;

class map
{
public:
    map(const std::vector<line<2>> walls,
        float min_x, float min_y, float max_x, float max_y)
        : walls(walls), min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) { };

    opt_point<2> raycast(line<2> ray) const;
    bool contains(point<2> p) const;
    const std::vector<line<2>>& get_walls() const;
    const float get_min_x() const { return min_x; };
    const float get_max_x() const { return max_x; };
    const float get_min_y() const { return min_y; };
    const float get_max_y() const { return max_y; };

private:
    std::vector<line<2>> walls;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
};