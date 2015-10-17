#pragma once

#include <cmath>
#include <array>
#include <ostream>

template<unsigned int d>
struct point
{
public:
    point()
        : data{0} { };
    point(float x, float y, float z)
        : data{x, y, z} { static_assert(d == 3, "xyz constructor only exists in 3d"); };
    point(float x, float y)
        : data{x, y} { static_assert(d == 2, "xyz constructor only exists in 3d"); };

    const float operator[](uint i) const { return data[i]; }
    float& operator[](uint i) { return data[i]; }

    friend point<d> operator*(const point& p, float other)
    {
        point<d> output;
        for (uint i = 0; i < d; i++)
            output[i] = p[i] * other;
        return output;
    }

    friend point<d> operator+(const point& p, float other)
    {
        point<d> output;
        for (uint i = 0; i < d; i++)
            output[i] += p[i] + other;
        return output;
    }
    friend point operator+(const point& lhs, const point& rhs)
    {
        point output = lhs;
        for (uint i = 0; i < d; i++)
            output[i] += rhs[i];
        return output;
    }

    friend point<d> operator-(const point& p, float other)
    {
        point<d> output = p;
        for (uint i = 0; i < d; i++)
            output[i] -= other;
        return output;
    }

    friend bool operator==(const point& lhs, const point& rhs)
    {
        for (uint i = 0; i < d; i++)
            if (lhs[i] != rhs[i])
                return false;
        return true;
    }
    friend bool operator!=(const point& lhs, const point& rhs)
    {
        return !(lhs == rhs);
    }

    friend std::ostream& operator<< (std::ostream& stream, const point& p)
    {
        stream << "(";
        for (uint i = 0; i < d; i++)
        {
            stream << +p.data[i];
            if (i != d - 1)
                stream << ", ";
        }
        stream << ")";
        return stream;
    }

private:
    std::array<float, d> data;
};

namespace std
{
    template<unsigned int d>
    struct hash<point<d>>
    {
        size_t operator()(const point<d>& p) const
        {
            size_t output = hash<float>()(p[0]);
            for (uint i = 1; i < d; i++)
            {
                output << 2;
                output ^= hash<float>()(p[i]);
            }
            return output;
        }
    };

}
