#include "offlineLaserData.h"
#include <mutex>

#include<iostream>



void OfflineLaserData::decodeLaserData(const sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg)
// void OfflineLaserData::decodeLaserData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
   int len = msg->ranges.size();
   LaserPoint<float> tempLaserPoint;
   LaserPointXY<float> tempLaserPointXY;
   laserDataRWMutex.lock();

   laserScanXY.clear();

   dataUpdated = false;

   for(int i = 0; i < len; i++)
   {
      tempLaserPoint.distance = msg->ranges[i].echoes[0];//use the first echo
      tempLaserPoint.intensity = msg->intensities[i].echoes[0];
      // tempLaserPoint.distance = msg->ranges[i];
      // tempLaserPoint.intensity = 0.;//msg->intensities[i];
      tempLaserPoint.angle = msg->angle_min + i * msg->angle_increment;

      if(tempLaserPoint.distance > 30.||//msg->range_max || 
         tempLaserPoint.distance < msg->range_min ||
         std::isnan(tempLaserPoint.distance)) 
         continue;

      tempLaserPointXY.x = tempLaserPoint.distance * cos(tempLaserPoint.angle);
      tempLaserPointXY.y = tempLaserPoint.distance * sin(tempLaserPoint.angle);
      tempLaserPointXY.intensity = tempLaserPoint.intensity;

      //laserScan.push_back(tempLaserPoint);
      laserScanXY.push_back(tempLaserPointXY);
   }

   //std::cout<<"decoded laserscan lenght = "<<len<<std::endl;

   laserScanQueue.push(laserScanXY);//insert laser scan into a queue

   //std::cout<<"Laser Scan Length = "<<laserScanXY.size()<<std::endl;

   dataUpdated = true;
   laserDataRWMutex.unlock();
}

std::vector<LaserPointXY<float>>* OfflineLaserData::getLaserScanXY(void)
{
   dataUpdated = false;

   return &laserScanQueue.front();
}

bool OfflineLaserData::ifDataUpdated(void)
{
   return dataUpdated;
}

bool OfflineLaserData::isDataQueueEmpty(void)
{
   return laserScanQueue.empty();
}

void OfflineLaserData::popFrontData(void)
{
   if(!laserScanQueue.empty()) laserScanQueue.pop();
}