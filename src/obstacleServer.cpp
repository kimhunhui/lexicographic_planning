#include "utility.h"
#include <memory>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pcl_ros/transforms.hpp"

using namespace std::chrono_literals;

class ObstacleServer : public rclcpp::Node, public ParamServer
{
public:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurroundMapCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurroundMapCloudDS;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMap;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMap2;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    pcl::PointCloud<PointType>::Ptr surroundMapCloud;
    pcl::PointCloud<PointType>::Ptr surroundMapCloudDS;
    pcl::VoxelGrid<PointType> downSizeFilter;

    std::deque<pcl::PointCloud<PointType>> cloudQueue;
    std::deque<rclcpp::Time> timeQueue;

    nav_msgs::msg::OccupancyGrid occupancyMap2D;

    ObstacleServer() : Node("obstacle_server")
    {
        this->declare_parameter<std::string>("point_cloud_topic", "planning/registered_cloud");

        std::string point_cloud_topic;
        this->get_parameter("point_cloud_topic", point_cloud_topic);
        
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, rclcpp::QoS(1), std::bind(&ObstacleServer::cloudHandler, this, _1));

        pubSurroundMapCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("planning/obstacle/surround_cloud_map", rclcpp::QoS(1));
        pubSurroundMapCloudDS = this->create_publisher<sensor_msgs::msg::PointCloud2>("planning/obstacle/surround_cloud_map_downsample", rclcpp::QoS(1));
        pubOccupancyMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("planning/obstacle/map", rclcpp::QoS(1));
        pubOccupancyMap2 = this->create_publisher<nav_msgs::msg::OccupancyGrid>("planning/obstacle/map_inflated", rclcpp::QoS(1));

        surroundMapCloud.reset(new pcl::PointCloud<PointType>());
        surroundMapCloudDS.reset(new pcl::PointCloud<PointType>());

        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        downSizeFilter.setLeafSize(_mapResolution, _mapResolution, _mapResolution);

        initializeOccupancyMap();
    }

    void initializeOccupancyMap()
    {
        occupancyMap2D.header.frame_id = "map";
        occupancyMap2D.info.width = _local_map_grid_num;
        occupancyMap2D.info.height = _local_map_grid_num;
        occupancyMap2D.info.resolution = _mapResolution;
        occupancyMap2D.info.origintation.x = 0.0;
        occupancyMap2D.info.origin.orientation.y = 0.0;
        occupancyMap2D.info.origin.orientation.z = 0.0;
        occupancyMap2D.info.origin.orientation.w = 1.0;
        occupancyMap2D.data.resize(occupancyMap2D.info.width * occupancyMap2D.info.height);
    } 


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform);} 
        catch (tf::TransformException ex){ return; }

        double timeScanCur = laserCloudMsg->header.stamp.toSec();

        PointType robotPoint;
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        // convert cloud
        pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        // save cloud and pop old cloud
        cloudQueue.push_back(*laserCloudIn);
        timeQueue.push_back(timeScanCur);

        while (!timeQueue.empty())
        {
            if (timeScanCur - timeQueue.front() > _sensorCloudTimeout)
            {
                cloudQueue.pop_front();
                timeQueue.pop_front();
            } else {
                break;
            }
        }
        
        // gather surrounding map
        surroundMapCloud->clear();
        surroundMapCloudDS->clear();
        for (int i = 0; i < cloudQueue.size(); ++i)
            *surroundMapCloud += cloudQueue[i];

        downSizeFilter.setInputCloud(surroundMapCloud);
        downSizeFilter.filter(*surroundMapCloudDS);

        // visualize surrounding map
        publishCloud(&pubSurroundMapCloud, surroundMapCloud, laserCloudMsg->header.stamp, "map");
        publishCloud(&pubSurroundMapCloudDS, surroundMapCloudDS, laserCloudMsg->header.stamp, "map");

        // update occupancy map
        if (pubOccupancyMap2.getNumSubscribers() != 0)
        {
            occupancyMap2D.header.stamp = laserCloudMsg->header.stamp;
            occupancyMap2D.info.origin.position.x = robotPoint.x - _local_map_length / 2.0;
            occupancyMap2D.info.origin.position.y = robotPoint.y - _local_map_length / 2.0;
            occupancyMap2D.info.origin.position.z = -0.1;
            std::fill(occupancyMap2D.data.begin(), occupancyMap2D.data.end(), 0);

            int search_num_ob = round(_occuMapInflation / _mapResolution);
            for (int i = 0; i < surroundMapCloudDS->size(); ++i)
            {
                int index_x = (surroundMapCloudDS->points[i].x - occupancyMap2D.info.origin.position.x) / _mapResolution;
                int index_y = (surroundMapCloudDS->points[i].y - occupancyMap2D.info.origin.position.y) / _mapResolution;

                for (int m = -search_num_ob; m <= search_num_ob; ++m)
                {
                    for (int n = -search_num_ob; n <= search_num_ob; ++n)
                    {
                        if (sqrt(float(m*m + n*n)) * _mapResolution > _occuMapInflation)
                            continue;

                        int x_id = index_x + m;
                        int y_id = index_y + n;

                        if (x_id < 0 || y_id < 0 || x_id >= _local_map_grid_num || y_id >= _local_map_grid_num)
                            continue;

                        int index = y_id * occupancyMap2D.info.width + x_id;
                        occupancyMap2D.data[index] = 100;
                    }
                }
            }

            pubOccupancyMap.publish(occupancyMap2D);

            // inflate occupancy map
            int search_num_field = round(_occuMapField / _mapResolution);
            float scale = 100.0 / max((float)log(1.0 / _mapResolution), (float)1e-6);

            for (int i = 0; i < occupancyMap2D.info.width; ++i)
            {
                for (int j = 0; j < occupancyMap2D.info.height; ++j)
                {
                    if (occupancyMap2D.data[j * occupancyMap2D.info.width + i] == 100)
                    {
                        for (int m = -search_num_field; m <= search_num_field; ++m)
                        {
                            for (int n = -search_num_field; n <= search_num_field; ++n)
                            {
                                float dist = sqrt(float(m*m + n*n)) * _mapResolution;
                                if (dist > _occuMapField)
                                    continue;

                                int newIdX = i + m;
                                int newIdY = j + n;
                                if (newIdX < 0 || newIdX >= occupancyMap2D.info.width || newIdY < 0 || newIdY >= occupancyMap2D.info.height)
                                    continue;

                                int index = newIdX + newIdY * occupancyMap2D.info.width;
                                if (occupancyMap2D.data[index] != 100)
                                {
                                    float inverse_dist = 1.0 / max(dist, _mapResolution);
                                    float log_inverse_dist = max((float)log(inverse_dist), (float)1e-6);
                                    int8_t grid_value = min(int(log_inverse_dist * scale), 99);
                                    occupancyMap2D.data[index] = max(occupancyMap2D.data[index], grid_value);
                                }
                            }
                        }
                    }
                }
            }

            pubOccupancyMap2.publish(occupancyMap2D);
        }
    }
};


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurroundMapCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurroundMapCloudDS;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMap;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubOccupancyMap2;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    
    // More members similar to ROS1 version
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
