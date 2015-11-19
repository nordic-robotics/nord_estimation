#include "ros/ros.h"
#include "ros/package.h"
#include "nord_messages/CoordinateArray.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/ObjectArray.h"
#include "nord_messages/Object.h"
#include "nord_messages/LandmarksSrv.h"
#include "nord_messages/Features.h"
#include "ros/package.h"
#include "landmarks.hpp"
#include <string>
#include <valarray>

#include "lerp_vector.hpp"

const float max_distance_threshold = 0.08;

landmarks* lm_ptr;

bool landmarks_service(nord_messages::LandmarksSrv::Request& req,
            nord_messages::LandmarksSrv::Response& res)
{
    res.data = lm_ptr->get_objects()[req.id].get_aggregated_features();
    return true;
}

int main(int argc, char** argv)
{
    using nord_messages::ObjectArray;

    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    lerp_vector<std::valarray<float>> poses;
    landmarks lm(max_distance_threshold);
    lm_ptr = &lm;

    ros::ServiceServer srv = n.advertiseService("/nord/estimation/landmarks_service", landmarks_service);

    ros::Publisher obj_pub = n.advertise<ObjectArray>("/nord/estimation/objects", 10);

    ros::Subscriber pose_sub = n.subscribe<nord_messages::PoseEstimate>(
        "/nord/estimation/pose_estimation", 10,
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
                lm.add(point<2>(c.x, c.y), pose, c.features);
            }

            ObjectArray msg_array;
            for (auto& o : lm.get_objects())
            {
                nord_messages::Object msg;
                msg.id = o.get_id();
                msg.x = o.get_mean().x();
                msg.y = o.get_mean().y();
                msg_array.data.push_back(msg);
            }
            obj_pub.publish(msg_array);
        });
    ros::spin();

    return 0;
}
