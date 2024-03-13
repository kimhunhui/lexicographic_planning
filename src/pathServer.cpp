#include "utility.h"

class PathServer : public rclcpp::Node
{
public:
    PathServer() : Node("path_server"), tfBuffer(this->get_clock()), tfListener(tfBuffer)
    {
        // ROS2에서는 this->create_publisher<MsgType>("topic", queue_size)를 사용하여 퍼블리셔를 생성합니다.
        pubPathRaw = this->create_publisher<nav_msgs::msg::Path>("planning/server/path_blueprint_raw", 10);
        pubPathSmooth = this->create_publisher<nav_msgs::msg::Path>("planning/server/path_blueprint_smooth", 10);

        // ROS2에서는 타이머를 생성할 때 this->create_wall_timer(duration, callback)를 사용합니다.
        pathUpdateTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathServer::updatePath, this));
    }

    nav_msgs::msg::Path pathRaw;
    nav_msgs::msg::Path pathSmooth;

    geometry_msgs::msg::Point robotPoint; // ROS2에서는 geometry_msgs::msg namespace를 사용합니다.
    
    double radius = 2.0;
    double length = 2.0;
    double width = radius * 2.0;
    
    // 아래의 함수들은 ROS2에서 사용할 수 있도록 수정해야 합니다.
    // 예를 들어, createPoseStamped 함수는 ROS2 메시지 타입을 사용해야 합니다.
    geometry_msgs::msg::PoseStamped createPoseStamped(float x, float y, float z)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now(); // ROS2에서 현재 시간을 얻는 방법
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        // pose.pose.orientation은 기본적으로 (0, 0, 0, 1)로 초기화됩니다.
        // 필요에 따라 회전을 적용할 수 있습니다.
        return pose;
    }

    PathServer() : Node("PathServer"), tfBuffer(this->get_clock()), tfListener(tfBuffer)
    {
        pubPathRaw = this->create_publisher<nav_msgs::msg::Path>("planning/server/path_blueprint_raw", 10);
        pubPathSmooth = this->create_publisher<nav_msgs::msg::Path>("planning/server/path_blueprint_smooth", 10);

        pathUpdateTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathServer::updatePath, this));

        createPath2();
    }

bool getRobotPosition() {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
        return false;
    }

    robotPoint.x = transformStamped.transform.translation.x;
    robotPoint.y = transformStamped.transform.translation.y;
    robotPoint.z = 0; // Z 값이 필요한 경우 transformStamped.transform.translation.z 사용

    return true;
}

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathRaw;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathSmooth;
    rclcpp::TimerBase::SharedPtr pathUpdateTimer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    nav_msgs::msg::Path pathRaw;
    nav_msgs::msg::Path pathSmooth;
    double width = 4.0; // Example value


    void createPath1()
    {
        pathRaw = nav_msgs::Path();
        // create raw path
        pathRaw.poses.push_back(createPoseStamped(0, 0, 0));
        pathRaw.poses.push_back(createPoseStamped(length, 0, 0));

        for (double angle = 0; angle <= M_PI; angle += M_PI / 18)
        {
            float x = length + radius * sin(angle);
            float y = width/2 - radius * cos(angle);
            pathRaw.poses.push_back(createPoseStamped(x, y, 0));
        }

        pathRaw.poses.push_back(createPoseStamped(length, width, 0));
        pathRaw.poses.push_back(createPoseStamped(0, width, 0));

        // smooth path
        pathSmooth = processPath(pathRaw);
    }

    void createPath2()
    {
        pathRaw = nav_msgs::Path();
        // create raw path
        pathRaw.poses.push_back(createPoseStamped(0, width, 0));
        pathRaw.poses.push_back(createPoseStamped(-length, width, 0));

        for (double angle = M_PI; angle <= 2 * M_PI; angle += M_PI / 18)
        {
            float x = -length + radius * sin(angle);
            float y =  width/2 - radius * cos(angle);
            pathRaw.poses.push_back(createPoseStamped(x, y, 0));
        }

        pathRaw.poses.push_back(createPoseStamped(-length, 0, 0));
        pathRaw.poses.push_back(createPoseStamped(0, 0, 0));

        // smooth path
        pathSmooth = processPath(pathRaw);
    }


    geometry_msgs::msg::PoseStamped createPoseStamped(float x, float y, float z)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        // pose.pose.orientation set to default quaternion, adjust as needed
        return pose;
    }

    void publishGlobalPath()
    {
        if (pubPathRaw.getNumSubscribers() != 0)
        {
            pathRaw.header.frame_id = "map";
            pathRaw.header.stamp = ros::Time::now();
            pubPathRaw.publish(pathRaw);
        }
        
        if (pubPathSmooth.getNumSubscribers() != 0)
        {
            pathSmooth.header.frame_id = "map";
            pathSmooth.header.stamp = ros::Time::now();
            pubPathSmooth.publish(pathSmooth);
        }
    }

    void updatePath(const ros::TimerEvent& event)
    {
        if (getRobotPosition() == false) return;

        PointType p1;
        p1.x = 0;
        p1.y = 0;
        p1.z = 0;

        PointType p2;
        p2.x = 0;
        p2.y = width;
        p2.z = 0;

        if (pointDistance(robotPoint, p1) < 1.0)
            createPath1();
        else if (pointDistance(robotPoint, p2) < 1.0)
            createPath2();
        else
            return;

        publishGlobalPath();
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
