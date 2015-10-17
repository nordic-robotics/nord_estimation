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
    map(const std::vector<line<2>> walls)
        : walls(walls) { };

    opt_point<2> raycast(line<2> ray) const;

private:
    std::vector<line<2>> walls;
};