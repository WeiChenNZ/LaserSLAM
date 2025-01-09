#ifndef _VISUALIZATION_INTERFACE_H_
#define _VISUALIZATION_INTERFACE_H_

#include "probablisticGridMap.h"
#include "Eigen/Core"

//显示用的虚基类，提供了显示接口
class VisualizationInterface{

    public:
    virtual void showHighMap(ProbablisticGridMap *) = 0;
    virtual void showLowMap(ProbablisticGridMap *) = 0;
    virtual void showPosition(Eigen::Vector3f) = 0;
    virtual void showLaserScanWithPos(std::vector<LaserPointXY<float>>*, Eigen::Matrix3f) = 0;
};
 
#endif