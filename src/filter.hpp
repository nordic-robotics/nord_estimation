#pragma once
 
#include <random>
#include <functional>
#include <algorithm>
#include <iostream>

namespace dust
{
    template<class State, class Observation>
    class filter
    {
    public:
        filter(unsigned int num_particles, double uniform_fraction, const State& init)
            : num_particles(num_particles),
              uniform_fraction(uniform_fraction),
              resample_dist(0, 1.0 / num_particles),
              particles(num_particles, std::make_pair(1.0 / num_particles, init))
        {
        }

        // uniform reset
        void reset()
        {
            particles.clear();
            particles.reserve(num_particles);
            std::generate_n(std::back_inserter(particles), num_particles,
                            [&]{ return std::make_pair(1.0 / num_particles, uniform()); });
        }
        // reset to specific State
        void reset(const State& s)
        {
            std::fill(particles.begin(), particles.end(), s);
        }
        // reset with generating function (with type State(void))
        template<class DistributionFunc>
        void reset(unsigned int new_num_particles, DistributionFunc f)
        {
            num_particles = new_num_particles;
            resample_dist = std::uniform_real_distribution<double>(0, 1.0 / num_particles);
            particles.clear();
            particles.reserve(num_particles);
            std::generate_n(std::back_inserter(particles), num_particles, [&]() {
                return std::make_pair(1.0 / num_particles, f());
            });
        }

        // move particles according to motion model
        void update(const Observation& z)
        {
            std::transform(particles.begin(), particles.end(), particles.begin(),
                           [&](const std::pair<double, State>& before) {
                               auto after = motion(before.second, z);
                               after.first = (after.first + before.first)/2;
                               return after;
                           });
            latest_obs = z;
        }

        void resample()
        {
            double sum = 0;
            for (auto p : particles)
            {
                sum += p.first;
            }

            std::sort(particles.begin(), particles.end(),
                [](const std::pair<double, State>& a, const std::pair<double, State>& b) {
                    return a.first > b.first;
                });

            // division by sum is to normalize sum to 1
            resample_buffer.clear();
            resample_buffer.reserve(num_particles);
            auto r = resample_dist(gen);
            auto c = particles.front().first / sum;
            unsigned int i = 0;
            for (unsigned int m = 0; m < num_particles; m++)
            {
                auto u = r + double(m) / num_particles;
                while (u > c)
                {
                    i++;
                    c += particles[i].first / sum;
                }
                resample_buffer.emplace_back(1.0 / num_particles, particles[i].second);

                if (resample_buffer.size() > (1.0 - uniform_fraction) * num_particles)
                {
                    break;
                }
            }

            particles = std::move(resample_buffer);
            std::generate_n(std::back_inserter(particles), num_particles - particles.size(),
                            [&]() {
                                auto next = motion(uniform(), latest_obs);
                                next.first /= 2.0;
                                return next;
                            });
            assert(num_particles == particles.size());
        }

        const std::vector<std::pair<double, State>>& get_particles() const
        {
            return particles;
        }

        unsigned int get_num_particles() const
        {
            return num_particles;
        }

        void set_num_particles(unsigned int new_num_particles)
        {
            num_particles = new_num_particles;
        }

    protected:
        virtual std::pair<double, State> motion(const State& state,
                                               const Observation& obs) const = 0;
        virtual State uniform() const = 0;
        mutable std::ranlux24_base gen;

    private:
        unsigned int num_particles;
        double uniform_fraction;
        std::vector<std::pair<double, State>> particles;
        std::vector<std::pair<double, State>> resample_buffer;
        std::uniform_real_distribution<double> resample_dist;
        Observation latest_obs;
    };
}
