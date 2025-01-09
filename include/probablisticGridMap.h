#ifndef _PROBABLISTIC_GRID_MAP_H_
#define _PROBABLISTIC_GRID_MAP_H_

#include <vector>
#include "laserDataInterface.h"
#include "Eigen/Core" 

//两个地图，一个高精度地图0.03m*0.03m，一个低精度地图0.3m*0.3m
//地图最大长宽为300m*300m，对应的栅格为10000*10000和1000*1000
//存储方式使用float数组（4字节），对应的空间大小为400MB和4MB
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
#define Phit  0.55
#define Pmiss  0.49

//log P(hit)/[1 - P(hit)]
#define logPhit    0.05017
#define logPmiss  -0.05436

//hit则对应栅格加上hit概率，miss则对应栅格加上miss概率
#define HitProbability 0.087   //log 0.55/(1-0.55)
#define MissProbability -0.017    // log 0.49/(1-0.49)
// #define HitProbability 0.05017//0.9f //log P(hit)/[1 - P(hit)]
// #define MissProbability -0.05436//-0.2f
// #define HitProbability 0.9f //log P(hit)/[1 - P(hit)]
// #define MissProbability -0.2f


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