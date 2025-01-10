#include "rosVisualization.h"
#include <Eigen/Dense>
#include <Eigen/Core>


//TODO
//the frame definition in ROS2 rviz2 seems not the same as the grid frame definition
//need to be figured out later

void RosVisualization::showHighMap(ProbablisticGridMap *pMap)
{
    auto t = rclcpp::Clock().now();
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = "laser_frame";
    map_msg.header.stamp = t;
    map_msg.info.height = HighResMapIndex;
    map_msg.info.width = HighResMapIndex;
    map_msg.info.resolution = HighResolutionGridSize;
    map_msg.info.origin.position.x = 150;
    map_msg.info.origin.position.y = 150;
    map_msg.info.origin.orientation.w = 0;

    map_msg.data.resize(map_msg.info.width * map_msg.info.height);

    for(int i = 0; i < HighResMapIndex; i++)
    {
        for(int j = 0; j < HighResMapIndex; j++)
        {
            float value = pMap->getHighMap()[j * HighResMapIndex + i];
            if(value < 0.0) map_msg.data[j * HighResMapIndex + i] = 0;
            else if(value > 0.0) map_msg.data[j * HighResMapIndex + i] = 100;
            else map_msg.data[j * HighResMapIndex + i] = -1;
        } 
    }

    mapPublisher_->publish(map_msg);
}

void RosVisualization::showLowMap(ProbablisticGridMap *pMap)
{
    auto t = rclcpp::Clock().now();
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = "laser_frame";
    map_msg.header.stamp = t;
    map_msg.info.height = LowResMapIndex;
    map_msg.info.width = LowResMapIndex;
    map_msg.info.resolution = LowResolutionGridSize;
    map_msg.info.origin.position.x = 150;
    map_msg.info.origin.position.y = 150;
    map_msg.info.origin.orientation.w = 0;

    map_msg.data.resize(map_msg.info.width * map_msg.info.height);

    for(int i = 0; i < LowResMapIndex; i++)
    {
        for(int j = 0; j < LowResMapIndex; j++)
        {
            float value = pMap->getLowMap()[i * LowResMapIndex + j];
            if(value < 0.0) map_msg.data[i * LowResMapIndex + j] = 0;
            else if(value > 0.0) map_msg.data[i * LowResMapIndex + j] = 100;
            else map_msg.data[i * LowResMapIndex + j] = -1;
        }
    }

    mapPublisher_->publish(map_msg);
}


void RosVisualization::showPosition(Eigen::Vector3f P)
{
    auto t = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped pose_stamped_msg;

    pose_stamped_msg.header.frame_id = "laser_frame";
    pose_stamped_msg.header.stamp = t;
    pose_stamped_msg.pose.position.x = -P(0); //why?
    pose_stamped_msg.pose.position.y = P(1); //translate to grid frame
    pose_stamped_msg.pose.position.z = 0;

    Eigen::Vector3d eulerAngle(P(2),0.,0.);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ())); 
    Eigen::Quaterniond quaternion=yawAngle*pitchAngle*rollAngle;


    pose_stamped_msg.pose.orientation.w = quaternion.w();
    pose_stamped_msg.pose.orientation.x = quaternion.x();
    pose_stamped_msg.pose.orientation.y = quaternion.y();
    pose_stamped_msg.pose.orientation.z = quaternion.z();


    positionPublisher_->publish(pose_stamped_msg);
}

void RosVisualization::showLaserScanWithPos(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T)
{
    sensor_msgs::msg::PointCloud scan;
    auto t = rclcpp::Clock().now();
    scan.header.frame_id = "laser_frame";
    scan.header.stamp = t;
    // end //////////////////////

    //data
    int len = laserScan->size();
    scan.points.resize(len);
    scan.channels.resize(1);
    scan.channels[0].values.resize(len);
    scan.channels[0].name = "intensity";

    Eigen::Matrix<float, 3, 1> pointInWorld;

    Eigen::Rotation2D<float> rotation(0.);
    Eigen::Matrix2f rot = rotation.toRotationMatrix();
    Eigen::Matrix<float, 2, 1> translation(0., 0.);
    Eigen::Matrix3f Trot;
    Trot << rot, translation, 0., 0., 1.;

    //std::cout << "Trot = "<<Trot<<std::endl;
    
    for(size_t i = 0; i < len; ++i)
    {
        Eigen::Matrix<float, 3, 1> point(laserScan->at(i).x, laserScan->at(i).y, 1.);
        pointInWorld = Trot* T * point;

        scan.points.at(i).x = -pointInWorld(0); //why?
        scan.points.at(i).y = pointInWorld(1);
        scan.points.at(i).z = 0.;
        scan.channels[0].values.at(i) = laserScan->at(i).intensity;
    }

    PointCloudPublisher_->publish(scan); 

}