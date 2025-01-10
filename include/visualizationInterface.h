#ifndef _VISUALIZATION_INTERFACE_H_
#define _VISUALIZATION_INTERFACE_H_

#include "probablisticGridMap.h"
#include "Eigen/Core"

//visualization public interface
class VisualizationInterface{

    public:
    virtual void showHighMap(ProbablisticGridMap *) = 0;
    virtual void showLowMap(ProbablisticGridMap *) = 0;
    virtual void showPosition(Eigen::Vector3f) = 0;
    virtual void showLaserScanWithPos(std::vector<LaserPointXY<float>>*, Eigen::Matrix3f) = 0;
};
 
#endif