#pragma once

#include <complex>
#include <cmath>
#include <cassert>

namespace maffs
{
    // returns (mean, stddev)
    template<class InputIt, class T = typename std::iterator_traits<InputIt>::value_type>
    std::pair<float, float>
    estimate_wrapped_normal_distribution(InputIt first, InputIt last, T min, T max)
    {
        assert(std::distance(first, last) > 0);

        std::complex<T> z;
        T r2_cos{};
        T r2_sin{};
        for (auto it = first; it != last; ++it)
        {
            z += std::exp(std::complex<T>(0, *it));
            r2_cos += std::cos(*it);
            r2_sin += std::sin(*it);
        }
        z /= std::distance(first, last);
        auto r2 = std::pow(r2_cos / std::distance(first, last), 2)
                + std::pow(r2_sin / std::distance(first, last), 2);
        auto variance = std::log(1.0f / r2);

        auto mu = std::log(z).imag();
        if (std::isnan(mu))
        {
            std::cout << "mu is fucked yo" << std::endl;
            exit(1);
        }

        return {mu, std::sqrt(variance)};
    }

    // returns (mean, stddev)
    template<class InputIt, class T = typename std::iterator_traits<InputIt>::value_type>
    std::pair<float, float>
    estimate_normal_distribution(InputIt first, InputIt last)
    {
        assert(std::distance(first, last) > 0);

        T mean{};
        for (auto it = first; it != last; ++it)
        {
            mean += *it;
        }
        mean /= std::distance(first, last);

        T variance{};
        for (auto it = first; it != last; ++it)
        {
            variance += std::pow(*it - mean, 2);
        }
        variance /= std::distance(first, last);

        if (std::isnan(mean))
        {
            std::cout << "mean is fucked yo" << std::endl;
            exit(1);
        }

        return {mean, std::sqrt(variance)};
    }
}
