#ifndef _PROBABLISTIC_GRID_MAP_H_
#define _PROBABLISTIC_GRID_MAP_H_

#include <vector>
#include "laserDataInterface.h"
#include "Eigen/Core" 


//two maps, one is high resolution grid map with 0.03m*0.03m, and the other is low resolution grid map with 0.3m*0.3m
//the maximum distance of the map is 300m*300m, so the maximum grid index in the two maps should be 10000 and 1000
//use float type (4 bytes) to store the maps, need 400MB and 4MB RAM respectively
#define HighResolutionGridSize 0.03f
#define LowResolutionGridSize  0.3f
#define MaxMapSize 300  //米
#define HighResMapIndex 10000
#define LowResMapIndex  1000




struct GridIndex{
    int x;
    int y;

    bool operator==(const GridIndex& rhs) const {
        return (x == rhs.x && y == rhs.y);
    }
};

struct GridIndexHash {
    size_t operator()(const GridIndex& p) const {
        return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
    }
};

enum MapResolution{
    LowRes = 0,
    HighRes
};

//cartographer
//not used now
#define Phit  0.55
#define Pmiss  0.49

//log P(hit)/[1 - P(hit)]
//not used now
#define logPhit    0.05017
#define logPmiss  -0.05436

//this is log(odd) definition, not probalility
// log(Odd(probablility))
// Odd = probablility / (1 - probablility)
#define HitProbability 0.087   //log 0.55/(1-0.55)
#define MissProbability -0.017    // log 0.49/(1-0.49)

//in cartographer, the maximum probability is 0.9
//in this project, the maximum probability is about 0.91, so that the log(odd) value is 1
#define LogOddMax 1.0f
#define LogOddMin -1.0f


class ProbablisticGridMap{

    public:
    void setLaserGridProbability(int startIndexX, int startIndexY, int endIndexX, int endIndexY);
    void setLaserGridProbability_backup(int startIndexX, int startIndexY, int endIndexX, int endIndexY);
    void updateHighMap(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T);
    void updateLowMap(void);

    float getScanProbability(int resolution, std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T);
    float getScanProbabilityInHighMap(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T);
    float getScanProbabilityInLowMap(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T);

    const float * getHighMap (void) const;
    const float * getLowMap(void) const;
    void clearHighMap(void);
    void clearLowMap(void);

    ProbablisticGridMap(void)
    {
        clearHighMap();
        clearLowMap();
    }

    private:
    float HighResMap[HighResMapIndex][HighResMapIndex] = {{0}};
    float LowResMap[LowResMapIndex][LowResMapIndex] = {{0}};
};



#endif