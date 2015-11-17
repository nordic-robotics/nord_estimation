#include "ros/ros.h"
#include "ros/package.h"
#include "nord_messages/CoordinateArray.h"
#include "nord_messages/PoseEstimate.h"
#include "ros/package.h"
#include "landmarks.hpp"
#include <string>
#include <valarray>

#include "lerp_vector.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    lerp_vector<std::valarray<float>> poses;
    landmarks objects(0.1);

    ros::Subscriber pose_sub = n.subscribe<nord_messages::PoseEstimate>(
        "/nord/estimation/gaussian", 10,
        [&](const nord_messages::PoseEstimate::ConstPtr& p) {
            poses.push_back(p->stamp.toSec(),
                            std::valarray<float>({p->x.mean, p->y.mean, p->theta.mean,
                                                  p->x.stddev, p->y.stddev, p->theta.stddev}));
        });

    ros::Subscriber ugo_sub = n.subscribe<nord_messages::CoordinateArray>(
        "/nord/nord_vision/ugo", 10,
        [&](const nord_messages::CoordinateArray::ConstPtr& centroids) {
            std::valarray<float> pose = poses[centroids->header.stamp.toSec()];

            for (auto& c : centroids->data)
            {
                auto object = objects.add(point<2>(c.x, c.y), pose);

                if (object.first)
                    std::cout << "new at " << object.second.get_mean() << std::endl;
                else
                    std::cout << "recognized at " << object.second.get_mean() << std::endl;
            }
        });

    ros::spin();

    return 0;
}
