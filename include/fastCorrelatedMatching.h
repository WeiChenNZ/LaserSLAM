#ifndef _FAST_CORRELATED_MATCHING_H_
#define _FAST_CORRELATED_MATCHING_H_

#include "laserDataInterface.h"
#include "probablisticGridMap.h"
#include <Eigen/Core>
#include <mutex>


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
    
    //world frame is w, laser frame is c
    //Twc is transformation from laser to world and vice versa
    Eigen::Matrix3f Tcw, Twc; //T matrix is used to calculate
    Eigen::Vector3f fastMatchingResult; //vector3f is used to store result

    std::vector<LaserPointXY<float>> laserScanXY;
    std::mutex rwLocker;
};


#endif