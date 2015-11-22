#pragma once

#include <cmath>
#include <cassert>

namespace studd
{
    // returns an iterator to the element which maximizes the function
    template<class InputIt, class UnaryFunction,
             class T = typename std::iterator_traits<InputIt>::value_type>
    auto argmax(InputIt first, InputIt last, UnaryFunction f)
        -> std::pair<InputIt, decltype(f(*first))>
    {
        assert(std::distance(first, last) > 0);

        decltype(f(*first)) max{};
        InputIt max_it;

        for (auto it = first; it != last; ++it)
        {
            auto res = f(*it);
            if (it == first || res > max)
            {
                max = res;
                max_it = it;
            }
        }

        return std::make_pair(max_it, max);
    }
}
