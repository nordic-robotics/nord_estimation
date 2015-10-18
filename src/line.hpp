#pragma once

#include "point.hpp"

template<unsigned int d>
class line
{
public:
    line(const point<d>& start, const point<d>& end)
        : start(start), end(end) { };
    line() { };

    float length() const
    {
        return (end - start).length();
    }

    point<d> start;
    point<d> end;
};