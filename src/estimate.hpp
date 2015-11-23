#pragma once

#include <complex>
#include <cmath>
#include <cassert>

namespace maffs
{
    // returns (mean, stddev)
    template<class InputIt, class T = typename std::iterator_traits<InputIt>::value_type>
    std::pair<double, double>
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
        auto stddev = std::log(1.0f / r2);

        auto mu = std::log(z).imag();
        if (std::isnan(mu))
        {
            std::cout << "mu is fucked yo" << std::endl;
            exit(1);
        }

        return {mu, stddev};
    }

    // returns (mean, stddev)
    template<class InputIt, class T = typename std::iterator_traits<InputIt>::value_type>
    std::pair<double, double>
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

    // returns (mean, stddev)
    template<class InputIt, class PairT = typename std::iterator_traits<InputIt>::value_type,
             class T1 = typename PairT::first_type, class T2 = typename PairT::second_type>
    std::pair<double, double>
    estimate_weighted_wrapped_normal_distribution(InputIt first, InputIt last, T2 min, T2 max)
    {
        assert(std::distance(first, last) > 0);

        std::complex<T2> z;
        T2 r2_cos{};
        T2 r2_sin{};
        T1 weight_sum{};
        for (auto it = first; it != last; ++it)
        {
            z += std::exp(std::complex<T2>(0, it->first * it->second));
            weight_sum += it->first;
            r2_cos += std::cos(it->second);
            r2_sin += std::sin(it->second);
        }
        //z *= std::distance(first, last);//std::complex<T2>(0, weight_sum);
        auto r2 = std::pow(r2_cos / std::distance(first, last), 2)
                + std::pow(r2_sin / std::distance(first, last), 2);
        auto stddev = std::log(1.0f / r2);

        auto mu = (std::log(z).imag() / weight_sum) * std::distance(first, last);
        if (std::isnan(mu))
        {
            std::cout << "mu is fucked yo" << std::endl;
            exit(1);
        }

        return {mu, stddev};
    }

    // returns (mean, stddev)
    template<class InputIt, class PairT = typename std::iterator_traits<InputIt>::value_type,
             class T1 = typename PairT::first_type, class T2 = typename PairT::second_type>
    std::pair<double, double>
    estimate_weighted_normal_distribution(InputIt first, InputIt last)
    {
        assert(std::distance(first, last) > 0);

        T2 mean{};
        T1 weight_sum{};
        for (auto it = first; it != last; ++it)
        {
            mean += it->first * it->second;
            weight_sum += it->first;
        }
        mean /= weight_sum;

        T2 variance{};
        for (auto it = first; it != last; ++it)
        {
            variance += std::pow(it->second - mean, 2);
        }
        variance /= std::distance(first, last);

        if (!std::isnormal(weight_sum))
        {
            std::cout << "weight_sum is fucked yo" << std::endl;
            std::cout << weight_sum << std::endl;
            exit(1);
        }
        if (!std::isnormal(mean) && mean != 0)
        {
            std::cout << "mean is fucked yo" << std::endl;
            std::cout << mean << std::endl;
            exit(1);
        }

        return {mean, std::sqrt(variance)};
    }
}
