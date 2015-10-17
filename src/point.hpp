#pragma once

#include <cmath>

class point
{
public:
    point(float x, float y, float z)
        : x(x), y(y), z(z) { };
    point() { };

    void normalize()
    {
        auto d = std::sqrt(x * x + y * y + z * z);
        x /= d;
        y /= d;
        z /= d;
    }

    float x = 0;
    float y = 0;
    float z = 0;
};