#include <memory>

#include "offlineLaserData.h"
#include "laserDataInterface.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "fastCorrelatedMatching.h"
#include "frontEnd.h"
#include "rosVisualization.h"

using std::placeholders::_1;
 
class OfflineDataSubscriber : public rclcpp::Node
{
  public:
    OfflineDataSubscriber(OfflineLaserData  *ptr)
    : Node("OfflineDataSubscriber"), offlineLaserDataPtr(ptr)
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::MultiEchoLaserScan>(
      "/horizontal_laser_2d", 10, std::bind(&OfflineDataSubscriber::topic_callback, this, _1));
      // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      // "/FLASER", 10, std::bind(&OfflineDataSubscriber::topic_callback, this, _1));

      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg)
    // void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      offlineLaserDataPtr->decodeLaserData(msg);
      
      // //显示激光数据
      // sensor_msgs::msg::LaserScan scan;
      // scan.header.frame_id = "laser_frame";
      // scan.header.stamp = msg->header.stamp;
      // scan.angle_min = msg->angle_min;
      // scan.angle_max = msg->angle_max;
      // scan.angle_increment = msg->angle_increment;
      // scan.range_max = msg->range_max;
      // scan.range_min = msg->range_min;
      // scan.scan_time = msg->scan_time;
      // scan.time_increment = msg->time_increment;

      // int len = msg->ranges.size();
      // scan.ranges.resize(len);
      // scan.intensities.resize(len);

      // for(int i = 0; i < len; i++)
      // {
      //   // scan.ranges[i] = msg->ranges[i].echoes[0];
      //   // scan.intensities[i] = msg->intensities[i].echoes[0];
      //   scan.ranges[i] = msg->ranges[i];
      //   scan.intensities[i] = msg->intensities[i];
        
      // }
      // publisher_->publish(scan);
      
    }

    rclcpp::Subscription<sensor_msgs::msg::MultiEchoLaserScan>::SharedPtr subscription_;
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    OfflineLaserData *offlineLaserDataPtr;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  OfflineLaserData *ros2OfflineData = new OfflineLaserData();
  RosVisualization *ros2Visualization = new RosVisualization();

  PeriodicTaskManager taskManager;
  FrontEnd frontEnd(ros2OfflineData, ros2Visualization, &taskManager, 0.2, "FrontEnd");

  frontEnd.start();

  rclcpp::spin(std::make_shared<OfflineDataSubscriber>(ros2OfflineData));


  delete ros2OfflineData;
  rclcpp::shutdown();
  return 0;  
}
