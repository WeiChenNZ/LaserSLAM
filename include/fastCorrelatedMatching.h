#ifndef _FAST_CORRELATED_MATCHING_H_
#define _FAST_CORRELATED_MATCHING_H_

#include "laserDataInterface.h"
#include "probablisticGridMap.h"
#include <Eigen/Core>
#include <mutex>


//存储地图匹配结果的得分和index
struct MatchingResult{
    float value;
    int i;
    int j;
    Eigen::Matrix3f T;
};

struct MatchingResultWithAngle{
    float value;
    float angle;
    Eigen::Matrix3f T;
};

//世界坐标系定义为w，激光坐标系定义为c
//Twc表示世界坐标系转换到激光坐标系
//Tcw表示激光坐标系转换到世界坐标系

class FastCorrelatedMatching{

    public:
    FastCorrelatedMatching(ProbablisticGridMap *mmap, LaserDataInterface<float> *scan):
    map(mmap), laserScan(scan)
    {
        Twc = Eigen::Matrix3f::Identity();
        Tcw = Eigen::Matrix3f::Identity();
        fastMatchingResult = {0., 0., 0.};
    };

    void matching(void);
    Eigen::Matrix3f getScanPos(void);
    Eigen::Vector3f getScanPosVect(void);
    void setScanPosVect(Eigen::Vector3f );

    float getAngle(void);
    void getLaserScan(void);
    MatchingResult findHighResMatchingResult(float angle, int lowI, int lowJ);

    private:
    ProbablisticGridMap *map;
    LaserDataInterface<float> *laserScan;
    
    Eigen::Matrix3f Tcw, Twc;
    Eigen::Vector3f fastMatchingResult;

    std::vector<LaserPointXY<float>> laserScanXY;
    std::mutex rwLocker;
};


#endif