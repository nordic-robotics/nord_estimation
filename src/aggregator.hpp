#pragma once

#include <vector>
#include <functional>

namespace aggregate
{
    template<class T, class MSG, class T2 = T>
    class aggregator : public std::vector<T>
    {
        using MSGPtr = typename MSG::ConstPtr;
    public:
        aggregator(ros::NodeHandle& n, const std::string& topic,
                   std::function<T(const MSGPtr&)> convert, T identity = T(),
                   unsigned int queue = 10)
            : sub(n.subscribe<MSG>(topic, queue,
                                   [&, convert](const MSGPtr& m) {
                                       readings.push_back(convert(m));
                                   }))
        { };

        T2 aggregate()
        {
            if (has_new())
            {
                latest_aggregate = aggregate_impl();
            }
            return latest_aggregate;
        }

        virtual T2 aggregate_impl() = 0;

        bool has_new()
        {
            return readings.size() > 0;
        }

    protected:
        std::vector<T> readings;
        T2 latest_aggregate = T2();
        T identity;

    private:
        ros::Subscriber sub;
    };

    template<class T, class MSG>
    class sum : public aggregator<T, MSG>
    {
    public:
        using aggregator<T, MSG>::aggregator;
        using aggregator<T, MSG>::readings;
        using aggregator<T, MSG>::identity;

        T aggregate_impl() override
        {
            auto result = std::accumulate(readings.begin(), readings.end(), identity);
            readings.clear();
            return result;
        }
    };

    template<class T, class MSG>
    class product : public aggregator<T, MSG>
    {
    public:
        using aggregator<T, MSG>::aggregator;
        using aggregator<T, MSG>::readings;
        using aggregator<T, MSG>::identity;

        T aggregate_impl() override
        {
            auto result = std::accumulate(readings.begin(), readings.end(), identity,
                                          std::multiplies<T>());
            readings.clear();
            return result;
        }
    };

    template<class T, class MSG>
    class average : public aggregator<T, MSG>
    {
    public:
        using aggregator<T, MSG>::aggregator;
        using aggregator<T, MSG>::readings;
        using aggregator<T, MSG>::identity;

        T aggregate_impl() override
        {
            auto result = std::accumulate(readings.begin(), readings.end(), identity);
            result /= readings.size();
            readings.clear();
            return result;
        }
    };

    template<class T, class MSG>
    class latest : public aggregator<T, MSG>
    {
    public:
        using aggregator<T, MSG>::aggregator;
        using aggregator<T, MSG>::readings;
        using aggregator<T, MSG>::identity;

        T aggregate_impl() override
        {
            auto result = readings.back();
            readings.clear();
            return result;
        }
    };
}