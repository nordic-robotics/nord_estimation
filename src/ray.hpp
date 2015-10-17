#pragma once

#include "point.hpp"

class ray
{
public:
    ray(const point& origin, const point& target)
        : origin(origin), target(target) { };
    ray() { };

    point origin;
    point target;
};