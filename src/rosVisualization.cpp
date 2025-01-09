#include "rosVisualization.h"
#include <Eigen/Dense>
#include <Eigen/Core>



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


//这个函数在显示的时候有问题，角度显示似乎不正确，后续需要调试查找bug
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
    
    
    //because I've already deleted some points which are too large or too low, 
    //so I need to insert the areas with some useless points,
    //or the points's angle after them will be decoded incorrectly,
    //Not done yet, do it later 20241218
    for(size_t i = 0; i < len; ++i)
    {
        Eigen::Matrix<float, 3, 1> point(laserScan->at(i).x, laserScan->at(i).y, 1.);
        pointInWorld = Trot* T * point;

        scan.points.at(i).x = -pointInWorld(0);
        scan.points.at(i).y = pointInWorld(1);
        scan.points.at(i).z = 0.;
        scan.channels[0].values.at(i) = laserScan->at(i).intensity;

        // if(i == 0)
        // {
        //     //adjust start angle
        //     float startangle = atan(pointInWorld(1)/pointInWorld(0));
        //     scan.angle_min = startangle;
        //     //scan.angle_max = startangle + 4.7;
        // }

        // scan.ranges[i] = sqrt(pointInWorld(0)*pointInWorld(0) + pointInWorld(1)*pointInWorld(1));
        // scan.intensities[i] = laserScan->at(i).intensity;
    }

    PointCloudPublisher_->publish(scan); 

}