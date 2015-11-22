#pragma once

#include <vector>
#include <utility>
#include <cassert>

template<class T>
class lerp_vector
{
public:
    void push_back(float time, const T& value)
    {
        assert(values.size() == 0 || time > values.back().first);

        values.emplace_back(time, value);
    }

    T operator[](float time)
    {
        for (size_t i = 0; i < values.size() - 1; i++)
        {
            if (values[i].first <= time && time <= values[i + 1].first)
            {
                return lerp(values[i].second, values[i + 1].second,
                            (time - values[i].first)
                            / (values[i + 1].first - values[i].first));
            }
        }
        return values.back().second;
    }

private:
    T lerp(T a, T b, float t)
    {
        return (1.0f - t) * a + t * b;
    }

    std::vector<std::pair<float, T>> values;
};