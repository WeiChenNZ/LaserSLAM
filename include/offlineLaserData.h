
#ifndef _OFFLINE_LASER_DATA_H_
#define _OFFLINE_LASER_DATA_H_


#include "laserDataInterface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <queue>
#include <mutex>

//ROS2 offline laser data decode
class OfflineLaserData: public LaserDataInterface<float>{

    public:
    void decodeLaserData(const sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg);
    // void decodeLaserData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<LaserPointXY<float>>* getLaserScanXY(void);

    bool ifDataUpdated(void);
    bool isDataQueueEmpty(void);
    void popFrontData(void);
 
    ~OfflineLaserData(){};  

    private:
    std::mutex laserDataRWMutex;
    bool dataUpdated = false;

    //std::vector<LaserPoint<float>> laserScan;
    std::vector<LaserPointXY<float>> laserScanXY;
    std::queue<std::vector<LaserPointXY<float>>> laserScanQueue;

};

#endif