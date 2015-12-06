#include "ros/ros.h"
#include "ros/package.h"
#include "nord_messages/CoordinateArray.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/DebrisArray.h"
#include "nord_messages/Debris.h"
#include "debris.hpp"
#include "map.hpp"
#include "line.hpp"
#include <string>
#include <valarray>
#include "visualization_msgs/Marker.h"
#include <fstream>
#include <sstream>

#include "lerp_vector.hpp"

const double max_distance_threshold = 0.08;
const size_t num_debris_required = 5;

debris2* lm_ptr;

map read_map(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<line<2>> walls;
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        if (l[0] == '#')
            continue;

        double x0, y0, x1, y1;
        iss >> x0 >> y0 >> x1 >> y1;
        min_x = std::min({min_x, x0, x1});
        min_y = std::min({min_y, y0, y1});
        max_x = std::max({max_x, x0, x1});
        max_y = std::max({max_y, y0, y1});
        walls.push_back(line<2>(point<2>(x0, y0), point<2>(x1, y1)));
    }

    return map(walls, min_x, min_y, max_x, max_y);
}

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

    map maze = read_map(ros::package::getPath("nord_estimation") + "/data/small_maze.txt");

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
            point<2> max;
            for (auto& o : lm.get_objects())
            {   
                auto temp_hull=o.get_hull();
                point<2> max;
                //find max offset through wall
                for (uint f=0; f<temp_hull.size();f++){
                    auto point1=point<2>(pose[0],pose[1]);
                    auto point2=point<2>(temp_hull[f].x,temp_hull[f].y);
                    line<2> ray = line<2>(point1, point2);
                    auto p = maze.raycast(ray);
                    if (p){
                        auto point3=p.value() - point<2>(pose[0],pose[1]);
                        if (point3.length()>max.length()){
                            max=point3;
                        }
                    }

                }
                //move all of object to this side of the wall                
                for (uint l=0; l<temp_hull.size();l++){
                    temp_hull[l].x=temp_hull[l].x-max.length();
                    temp_hull[l].y=temp_hull[l].y-max.length();
                }
                //if we have seen object enough times. post it
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
