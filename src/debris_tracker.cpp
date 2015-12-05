#include "ros/ros.h"
#include "ros/package.h"
#include "nord_messages/CoordinateArray.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/DebrisArray.h"
#include "nord_messages/Debris.h"
#include "debris.hpp"
#include <string>
#include <valarray>
#include "visualization_msgs/Marker.h"

#include "lerp_vector.hpp"

const double max_distance_threshold = 0.08;
const size_t num_debris_required = 5;

debris2* lm_ptr;

// draws particles with weight, blue are more likely than yellow
visualization_msgs::Marker
create_points_message(std::vector<debris> objs)
{
    visualization_msgs::Marker point_list;
    point_list.id = 19;
    point_list.type = visualization_msgs::Marker::POINTS;
    point_list.color.g = point_list.color.r = 0.0f;
    point_list.color.a = point_list.color.b = 1.0f;
    point_list.header.frame_id = "/map";
    point_list.header.stamp = ros::Time::now();
    point_list.ns = "debris";
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
    using nord_messages::DebrisArray;

    ros::init(argc, argv, "debris_tracker");
    ros::NodeHandle n;

    lerp_vector<std::valarray<double>> poses;
    debris2 lm(max_distance_threshold);
    lm_ptr = &lm;

    //subscribers
    ros::Subscriber pose_sub = n.subscribe<nord_messages::PoseEstimate>(
        "/nord/estimation/pose_estimation", 10,
        [&](const nord_messages::PoseEstimate::ConstPtr& p) {
            poses.push_back(p->stamp.toSec(),
                            std::valarray<double>({p->x.mean, p->y.mean, p->theta.mean,
                                                  p->x.stddev, p->y.stddev, p->theta.stddev}));
                                                          });
    //publishers
    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 10);
    auto debris_pub = n.advertise< nord_messages::DebrisArray>("/nord/estimation/debris", 10);
    ros::Subscriber ugo_sub = n.subscribe<nord_messages::CoordinateArray>(
        "/nord/pointcloud/centroids", 10,
        [&](const nord_messages::CoordinateArray::ConstPtr& centroids) {
            if (poses.size() == 0)
                return;
            std::valarray<double> pose = poses[centroids->header.stamp.toSec()];
            for (auto& c : centroids->data)
            {
                lm.add(point<2>(c.x, c.y), pose, c.hull);
            }

            DebrisArray msg_array;
            std::vector<debris> temp;
            for (auto& o : lm.get_objects())
            {
                auto hull_num = o.get_num_features();
                if (hull_num > num_debris_required)
                {
                    nord_messages::Debris msg;
                    msg.id = o.get_id();
                    msg.x = o.get_mean().x();
                    msg.y = o.get_mean().y();
                    msg.hull=o.get_hull();
                    msg_array.data.push_back(msg);
                    temp.push_back(o);
                }
            }
            debris_pub.publish(msg_array);
            auto msg = create_points_message(temp);
            map_pub.publish(msg);
        });
    ros::spin();

    return 0;
}
