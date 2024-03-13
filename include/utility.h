#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_core/global_planner.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <memory>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
// 기타 필요한 PCL 관련 헤더

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;
using namespace rclcpp;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;

class ParamServer : public rclcpp::Node
{
public:
    std::string robot_id;
    string _pointCloudTopic;

    // Occupancy Map Params
    float _mapResolution;
    float _occuMapInflation;
    float _occuMapField;

    // Filter Threshold Params
    float _sensorRangeLimitMin;
    float _sensorRangeLimitMax;
    float _sensorHeightLimitUpper;
    float _sensorHeightLimitDown;
    float _sensorCloudTimeout;

    int   _local_map_grid_num;
    float _local_map_length;

    // Alternative Path Params
    float _samplingTipMargin;
    float _samplingOutMargin;
    float _pathResolution;
    float _maxPathDistance;
    float _rollOutDensity;
    int   _rollOutNumber;
    int   _rollOutCenter;

    ParamServer() : Node("param_server")
    {
        this->declare_parameter<std::string>("robot_id", "roboat");
        this->get_parameter("robot_id", robot_id);

        this->declare_parameter<std::string>("_pointCloudTopic", "points_raw");
        this->get_parameter("_pointCloudTopic", _pointCloudTopic);

        // Repeat the above pattern for each parameter
        // Example:
        this->declare_parameter<float>("_mapResolution", 0.1);
        this->get_parameter("_mapResolution", _mapResolution);
        this->declare_parameter<float>("roboat_planning.occuMapInflation", 0.5);
        this->get_parameter("roboat_planning.occuMapInflation", _occuMapInflation);
        this->declare_parameter<float>("roboat_planning.occuMapField", 0.5);
        this->get_parameter("roboat_planning.occuMapField", _occuMapField);
        this->declare_parameter<float>("roboat_planning.sensorRangeLimitMin", 1.0);
        this->get_parameter("roboat_planning.sensorRangeLimitMin", _sensorRangeLimitMin);
        this->declare_parameter<float>("roboat_planning.sensorRangeLimitMax", 30.0);
        this->get_parameter("roboat_planning.sensorRangeLimitMax", _sensorRangeLimitMax);
        this->declare_parameter<float>("roboat_planning.sensorHeightLimitUpper", 0.5);
        this->get_parameter("roboat_planning.sensorHeightLimitUpper", _sensorHeightLimitUpper);
        this->declare_parameter<float>("roboat_planning.sensorHeightLimitDown", -0.2);
        this->get_parameter("roboat_planning.sensorHeightLimitDown", _sensorHeightLimitDown);
        this->declare_parameter<float>("roboat_planning.sensorCloudTimeout", 25.0);
        this->get_parameter("roboat_planning.sensorCloudTimeout", _sensorCloudTimeout);
        this->declare_parameter<float>("roboat_planning.samplingTipMargin", 1.0);
        this->get_parameter("roboat_planning.samplingTipMargin", _samplingTipMargin);
        this->declare_parameter<float>("roboat_planning.samplingOutMargin", 1.0);
        this->get_parameter("roboat_planning.samplingOutMargin", _samplingOutMargin);
        this->declare_parameter<float>("roboat_planning.pathResolution", 0.1);
        this->get_parameter("roboat_planning.pathResolution", _pathResolution);
        this->declare_parameter<float>("roboat_planning.maxPathDistance", 10.0);
        this->get_parameter("roboat_planning.maxPathDistance", _maxPathDistance);
        this->declare_parameter<float>("roboat_planning.rollOutDensity", 0.1);
        this->get_parameter("roboat_planning.rollOutDensity", _rollOutDensity);
        this->declare_parameter<int>("roboat_planning.rollOutNumber", 20);
        this->get_parameter("roboat_planning.rollOutNumber", _rollOutNumber);
     
        // Calculate derived parameters
        _rollOutCenter = _rollOutNumber / 2;
        if (_maxPathDistance < _sensorRangeLimitMax * 2.0)
            RCLCPP_WARN(this->get_logger(), "Assigned length for generating alternative paths might not be long enough!");

        _local_map_grid_num = round(_sensorRangeLimitMax * 4.0 / _mapResolution);
        _local_map_length = _sensorRangeLimitMax * 4.0;
    }

    nav_msgs::Path processPath(nav_msgs::Path pathIn)
    {
        pathIn = calculatePathYaw(pathIn);
        pathIn = fixPathDensity(pathIn);
        pathIn = smoothPath(pathIn);
        return pathIn;
    }

    nav_msgs::msg::Path calculatePathYaw(nav_msgs::msg::Path pathIn)
    {
        int length = pathIn.poses.size();
        if (length <= 1)
        {
            if (length == 1)
            {
                tf2::Quaternion q;
                q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
                pathIn.poses[0].pose.orientation = tf2::toMsg(q);
            }
            return pathIn;
        }
    
        for (int i = 0; i < length - 1; ++i)
        {
            double dx = pathIn.poses[i+1].pose.position.x - pathIn.poses[i].pose.position.x;
            double dy = pathIn.poses[i+1].pose.position.y - pathIn.poses[i].pose.position.y;
            double theta = atan2(dy, dx);
            
            tf2::Quaternion q;
            q.setRPY(0, 0, theta); // Roll, Pitch, Yaw
            pathIn.poses[i].pose.orientation = tf2::toMsg(q);
        }
    
        pathIn.poses.back().pose.orientation = pathIn.poses[length-2].pose.orientation;
    
        return pathIn;
    }

    nav_msgs::Path smoothPath(nav_msgs::Path path)
    {
        if (path.poses.size() <= 2)
            return path;

        double weight_data = 0.45;
        double weight_smooth = 0.4;
        double tolerance = 0.05;

        nav_msgs::Path smoothPath_out = path;

        double change = tolerance;
        double xtemp, ytemp;
        int nIterations = 0;

        int size = path.poses.size();

        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < size - 1; i++) {
                xtemp = smoothPath_out.poses[i].pose.position.x;
                ytemp = smoothPath_out.poses[i].pose.position.y;

                smoothPath_out.poses[i].pose.position.x += weight_data * (path.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
                smoothPath_out.poses[i].pose.position.y += weight_data * (path.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

                smoothPath_out.poses[i].pose.position.x += weight_smooth * (smoothPath_out.poses[i-1].pose.position.x + smoothPath_out.poses[i+1].pose.position.x - (2.0 * smoothPath_out.poses[i].pose.position.x));
                smoothPath_out.poses[i].pose.position.y += weight_smooth * (smoothPath_out.poses[i-1].pose.position.y + smoothPath_out.poses[i+1].pose.position.y - (2.0 * smoothPath_out.poses[i].pose.position.y));

                change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
                change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
            }
            nIterations++;
        }

        return smoothPath_out;
    }

    nav_msgs::Path fixPathDensity(nav_msgs::Path path)
    {
        if (path.poses.size() == 0 || _pathResolution == 0)
            return path;

        double dis = 0, ang = 0;
        double margin = _pathResolution * 0.01;
        double remaining = 0;
        int nPoints = 0;

        nav_msgs::Path fixedPath = path;
        fixedPath.poses.clear();
        fixedPath.poses.push_back(path.poses[0]);

        size_t start = 0, next = 1;
        while (next < path.poses.size())
        {
            dis += hypot(path.poses[next].pose.position.x - path.poses[next-1].pose.position.x, path.poses[next].pose.position.y - path.poses[next-1].pose.position.y) + remaining;
            ang = atan2(path.poses[next].pose.position.y - path.poses[start].pose.position.y, path.poses[next].pose.position.x - path.poses[start].pose.position.x);

            if (dis < _pathResolution - margin)
            {
                next++;
                remaining = 0;
            } else if (dis > (_pathResolution + margin))
            {
                geometry_msgs::PoseStamped point_start = path.poses[start];
                nPoints = dis / _pathResolution;
                for (int j = 0; j < nPoints; j++)
                {
                    point_start.pose.position.x = point_start.pose.position.x + _pathResolution * cos(ang);
                    point_start.pose.position.y = point_start.pose.position.y + _pathResolution * sin(ang);
                    point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
                    fixedPath.poses.push_back(point_start);
                }
                remaining = dis - nPoints * _pathResolution;
                start++;
                path.poses[start].pose.position = point_start.pose.position;
                dis = 0;
                next++;
            } else {
                dis = 0;
                remaining = 0;
                fixedPath.poses.push_back(path.poses[next]);
                next++;
                start = next - 1;
            }
        }

        return fixedPath;
    }
};

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

float pointDistance(PointType p)
{
    return sqrt((p.x*p.x)+(p.y*p.y)+(p.z*p.z));
}


struct state_t;
struct neighbor_t;

struct state_t
{
    float x = 0;
    float y = 0;
    float z = 0;
    float theta = 0;
    float costsToRoot[NUM_COSTS] = {FLT_MAX};

    int idx = -1;
    int idy = -1;
    int stateId = -1;

    bool validFlag = false;

    state_t* parentState = NULL;
    vector<neighbor_t> neighborList;
};

struct neighbor_t{
    state_t* neighbor;
    float edgeCosts[NUM_COSTS]; // the cost from this state to neighbor
    neighbor_t(){
        neighbor = NULL;
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = FLT_MAX;
    }
};

void publishCloud(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    if (thisPub->get_subscription_count() == 0)
        return;
    
    auto tempCloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*thisCloud, *tempCloud);
    tempCloud->header.stamp = thisStamp;
    tempCloud->header.frame_id = thisFrame;
    thisPub->publish(*tempCloud);
}

// coordinate system transform

// lidar = camera
// x = z
// y = x
// z = y
// roll = yaw
// pitch = roll
// yaw = pitch

// camera = lidar
// x = y
// y = z
// z = x
// roll = pitch
// pitch = yaw
// yaw = roll

#endif
