#include "utility.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "pcl_ros/transforms.hpp"

using std::placeholders::_1;

class CloudRegister : public rclcpp::Node, public ParamServer
{
public:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    bool newCloudFlag;
    std_msgs::msg::Header cloudHeader;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::VoxelGrid<PointType> downSizeFilter;

    CloudRegister() : Node("cloud_register")
    {
        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("planning/registered_cloud", 10);
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            _pointCloudTopic, 10, std::bind(&CloudRegister::cloudHandler, this, _1));

        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        newCloudFlag = false;
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        downSizeFilter.setLeafSize(_mapResolution, _mapResolution, _mapResolution);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        if (pubLaserCloud->get_subscription_count() == 0)
            return;

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", "base_link", "map", ex.what());
            return;
        }

        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        newCloudFlag = true;
    }

    void run()
    {
        if (!newCloudFlag)
        {
            return;
        }

        // Transform cloud to global frame
        pcl::PointCloud<PointType>::Ptr laserCloudInGlobal(new pcl::PointCloud<PointType>());
        pcl_ros::transformPointCloud("map", *laserCloudIn, *laserCloudInGlobal, *tfBuffer);

        // Downsample cloud
        pcl::PointCloud<PointType>::Ptr laserCloudInDS(new pcl::PointCloud<PointType>());
        downSizeFilter.setInputCloud(laserCloudInGlobal);
        downSizeFilter.filter(*laserCloudInDS);

        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*laserCloudInDS, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "map";

        pubLaserCloud->publish(output);
        newCloudFlag = false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudRegister>();
    rclcpp::Rate rate(3);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->run();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
