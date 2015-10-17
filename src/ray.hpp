#pragma once

#include "point.hpp"

template<unsigned int d>
class ray
{
public:
    ray(const point<d>& origin, const point<d>& target)
        : origin(origin), target(target) { };
    ray() { };

    point<d> origin;
    point<d> target;
};