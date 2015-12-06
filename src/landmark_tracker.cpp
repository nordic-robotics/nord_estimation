#include "ros/ros.h"
#include "ros/package.h"
#include "nord_messages/CoordinateArray.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/ObjectArray.h"
#include "nord_messages/Object.h"
#include "nord_messages/LandmarksSrv.h"
#include "nord_messages/MoneyshotSrv.h"
#include "nord_messages/Features.h"
#include "landmarks.hpp"
#include <string>
#include <valarray>
#include "visualization_msgs/Marker.h"

#include "lerp_vector.hpp"

const double max_distance_threshold = 0.04;
const size_t num_shape_required = 2;
const size_t num_color_required = 8;

landmarks* lm_ptr;



bool landmarks_service(nord_messages::LandmarksSrv::Request& req,
            nord_messages::LandmarksSrv::Response& res)
{
    std::cout << "entered service" << std::endl;
    res.data = lm_ptr->get_objects()[req.id].get_aggregated_features();
    return true;
}

bool moneyshot_service(nord_messages::MoneyshotSrv::Request& req,
            nord_messages::MoneyshotSrv::Response& res)
{
    std::cout << "entered money shot service" << std::endl;
    // res.data = lm_ptr->get_objects()[req.id].get_aggregated_features();
    int best_xp, best_yp, xp, yp, best_r, h, w, r, objId;
    sensor_msgs::Image  const * moneyshot = nullptr;
    for (uint i=0; i<req.ids.size();i++) {
        objId = req.ids[i];
        if ( i == 0 ) {
            moneyshot = &lm_ptr->get_objects()[objId].get_moneyshot();
            h = moneyshot->height / 2;
            w = moneyshot->width / 2;
            best_xp = lm_ptr->get_objects()[objId].get_xp();
            best_yp = lm_ptr->get_objects()[objId].get_yp();
            best_r = (h-best_yp)*(h-best_yp) + (w-best_xp)*(w-best_yp);
        } else {
            // calculate the distance from the center
            xp = lm_ptr->get_objects()[objId].get_xp();
            yp = lm_ptr->get_objects()[objId].get_yp();
            r = (h-yp)*(h-yp) + (w-xp)*(w-xp);
            // update if the new image is more centered
            if ( r < best_r ) {
                moneyshot = &lm_ptr->get_objects()[objId].get_moneyshot();
                xp = xp;
                yp = yp;
            }
        }
    }

    res.moneyshot = *moneyshot;

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
    ros::ServiceServer moneyshotSrv = n.advertiseService("/nord/estimation/moneyshot_service", moneyshot_service);

    ros::Publisher obj_pub = n.advertise<ObjectArray>("/nord/estimation/objects", 10);

    ros::Subscriber pose_sub = n.subscribe<nord_messages::PoseEstimate>(
        "/nord/estimation/pose_estimation", 10,
        [&](const nord_messages::PoseEstimate::ConstPtr& p) {
            poses.push_back(p->stamp.toSec(),
                            std::valarray<double>({p->x.mean, p->y.mean, p->theta.mean,
                                                   p->x.stddev, p->y.stddev, p->theta.stddev,
                                                   std::cos(p->theta.mean),
                                                   std::sin(p->theta.mean)}));
        });

    auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 10);

    ros::Subscriber ugo_sub = n.subscribe<nord_messages::CoordinateArray>(
        "/nord/vision/ugo", 10,
        [&](const nord_messages::CoordinateArray::ConstPtr& centroids) {
	  std::cout<<"entered ugo callback"<<std::endl;
            if (poses.size() == 0)
                return;
            std::valarray<double> pose = poses[centroids->header.stamp.toSec()];
            for (auto& c : centroids->data)
            {
                lm.add(point<2>(c.x, c.y), pose, c.features, c.xp, c.yp, centroids->moneyshot);
            }

            ObjectArray msg_array;
            std::vector<landmark> temp;
            for (auto& o : lm.get_objects())
            {
                auto features = o.get_num_features();
                if (features.first > num_color_required
                 && features.second > num_shape_required)
                {
                    nord_messages::Object msg;
                    msg.id = o.get_id();
                    msg.x = o.get_mean().x();
                    msg.y = o.get_mean().y();
                    msg.xp = o.get_xp();
                    msg.yp = o.get_yp();
                    msg.nrObs = features.first + features.second;
                    msg.moneyshot = o.get_moneyshot();
                    msg_array.data.push_back(msg);
		    std::cout<<"before posting"<<std::endl;
		    std::cout<<o.get_moneyshot().data.size()<<std::endl;
		    std::cout<<msg.moneyshot.data.size()<<std::endl;
                    temp.push_back(o);
                }
            }
            obj_pub.publish(msg_array);
            auto msg = create_points_message(temp);
            map_pub.publish(msg);
        });
    ros::spin();

    return 0;
}
