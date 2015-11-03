#pragma once
 
#include <random>
#include <functional>
#include <algorithm>

namespace dust
{
    template<class State, class Observation>
    class filter
    {
    public:
        filter(unsigned int num_particles, const State& init)
            : num_particles(num_particles),
              resample_dist(0, 1.0f / num_particles),
              particles(num_particles, init),
              sampled_particles(num_particles)
        {
        }

        void reset()
        {
            particles.clear();
            particles.reserve(num_particles);
            std::generate_n(std::back_inserter(particles), num_particles,
                            [&]{ return uniform(); });
        }
        void reset(const State& s)
        {
            std::fill(particles.begin(), particles.end(), s);
        }

        void update(const Observation& z)
        {
            sampled_particles.clear();
            sampled_particles.reserve(num_particles);
            float sum = 0;
            std::transform(particles.begin(), particles.end(),
                           std::back_inserter(sampled_particles),
                           [&](const State& particle) {
                               auto next = motion(particle, z);
                               sum += next.first;
                               return next;
                           });

            particles.clear();
            particles.reserve(num_particles);
            auto r = resample_dist(gen);
            auto c = sampled_particles.front().first / sum;
            unsigned int i = 0;
            for (unsigned int m = 0; m < num_particles; m++)
            {
                auto u = r + float(m) / num_particles;
                while (u > c)
                {
                    i++;
                    c += sampled_particles[i].first / sum;
                }
                particles.push_back(sampled_particles[i].second);
            }
        }

        std::pair<float, State> estimate_largest_weight()
        {
            std::pair<float, State> largest = { 0, State() };
            for (auto& p : sampled_particles)
            {
                if (p.first >= largest.first)
                {
                    largest = p;
                }
            }

            return largest;
        }

        const std::vector<State>& get_particles() const
        {
            return particles;
        }

        const unsigned int get_num_particles() const
        {
            return num_particles;
        }

        const std::vector<std::pair<float, State>>& get_sampled_particles() const
        {
            return sampled_particles;
        }

    protected:
        virtual std::pair<float, State> motion(const State& state,
                                               const Observation& obs) const = 0;
        virtual State uniform() const = 0;
        mutable std::ranlux24_base gen;

    private:
        unsigned int num_particles;
        std::vector<State> particles;
        std::vector<std::pair<float, State>> sampled_particles;
        std::uniform_real_distribution<float> resample_dist;
    };
}
