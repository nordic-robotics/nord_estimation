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
#include "visualization_msgs/Marker.h"

#include "lerp_vector.hpp"

const double max_distance_threshold = 0.20;

landmarks* lm_ptr;

bool landmarks_service(nord_messages::LandmarksSrv::Request& req,
            nord_messages::LandmarksSrv::Response& res)
{
    std::cout << "entered service" << std::endl;
    res.data = lm_ptr->get_objects()[req.id].get_aggregated_features();
    return true;
}


// draws particles with weight, blue are more likely than yellow
visualization_msgs::Marker
create_points_message(std::vector<landmark> objs)
{
    visualization_msgs::Marker point_list;
    point_list.id = 11;
    point_list.type = visualization_msgs::Marker::POINTS;
    point_list.color.a = point_list.color.r = 1.0f;
    point_list.color.g = point_list.color.b = 0.0f;
    point_list.header.frame_id = "/map";
    point_list.header.stamp = ros::Time::now();
    point_list.ns = "pf_particles";
    point_list.action = visualization_msgs::Marker::ADD;
    point_list.pose.orientation.w = 1.0;
    point_list.lifetime = ros::Duration();
    point_list.scale.x = point_list.scale.y = 0.05;

    for (auto& o : objs)
    {
        geometry_msgs::Point p0;
        p0.x = o.get_mean().x();
        p0.y = o.get_mean().y();
        p0.z = 0;
        point_list.points.push_back(p0);
    }

    return point_list;
}

int main(int argc, char** argv)
{
    using nord_messages::ObjectArray;

    ros::init(argc, argv, "landmark_tracker");
    ros::NodeHandle n;

    lerp_vector<std::valarray<double>> poses;
    landmarks lm(max_distance_threshold);
    lm_ptr = &lm;

    ros::ServiceServer srv = n.advertiseService("/nord/estimation/landmarks_service", landmarks_service);

    ros::Publisher obj_pub = n.advertise<ObjectArray>("/nord/estimation/objects", 10);

    ros::Subscriber pose_sub = n.subscribe<nord_messages::PoseEstimate>(
        "/nord/estimation/pose_estimation", 10,
        [&](const nord_messages::PoseEstimate::ConstPtr& p) {
            poses.push_back(p->stamp.toSec(),
                            std::valarray<double>({p->x.mean, p->y.mean, p->theta.mean,
                                                  p->x.stddev, p->y.stddev, p->theta.stddev}));
        });

    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 10);
    auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
        auto msg = create_points_message(lm.get_objects());
        map_pub.publish(msg);
    });

    ros::Subscriber ugo_sub = n.subscribe<nord_messages::CoordinateArray>(
        "/nord/vision/ugo", 10,
        [&](const nord_messages::CoordinateArray::ConstPtr& centroids) {
            if (poses.size() == 0)
                return;
            std::valarray<double> pose = poses[centroids->header.stamp.toSec()];
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
