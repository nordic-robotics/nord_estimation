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

    line rotated(float theta) const
    {
        point<d> p = end - start;
        auto st = std::sin(theta);
        auto ct = std::cos(theta);
        return line(point<d>(start.x() * ct - start.y() * st,
                             start.x() * st + start.y() * ct),
                    point<d>(p.x() * ct - p.y() * st,
                             p.x() * st + p.y() * ct));
    }

    point<d> start;
    point<d> end;
};