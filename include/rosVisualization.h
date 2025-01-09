#ifndef _ROS_VISUALIZATION_H_
#define _ROS_VISUALIZATION_H_

#include "visualizationInterface.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "laserDataInterface.h"
#include "rclcpp/rclcpp.hpp"

class RosVisualization: public VisualizationInterface, rclcpp::Node{

public:
RosVisualization(void):Node("Ros2Visualization")
{
    mapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("probmap", 10);
    positionPublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robotpos",10);
    PointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("laser_pointcloud", 10);
}

void showHighMap(ProbablisticGridMap *pMap);
void showLowMap(ProbablisticGridMap *pMap);
void showPosition(Eigen::Vector3f P);
void showLaserScanWithPos(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T);


private: 
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   positionPublisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr   PointCloudPublisher_;

};


#endif