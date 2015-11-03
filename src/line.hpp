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
        static_assert(d == 2, "can only rotate if 2d");
        point<d> p = end - start;
        auto st = std::sin(theta);
        auto ct = std::cos(theta);
        auto p0 = point<d>(start.x() * ct - start.y() * st,
                           start.x() * st + start.y() * ct);
        auto p1 = p0 + point<d>(p.x() * ct - p.y() * st,
                                p.x() * st + p.y() * ct);
        return line(p0, p1);
    }

    friend line<d> operator+(const line& lhs, const point<d>& rhs)
    {
        return line<d>(lhs.start + rhs, lhs.end + rhs);
    }

    point<d> start;
    point<d> end;
};