#ifndef _LASER_DATA_INTERFACE_H_
#define _LASER_DATA_INTERFACE_H_

#include <vector>

//模板化激光点数据类型，方便后续根据不同精度调整
template<class T>
class LaserPoint{
    public:
    T distance;
    T intensity;
    T angle;
};

template<class T>
class LaserPointXY{
    public:
    T x;
    T y;
    T intensity;
};

template<class T>
class LaserDataInterface{

    public:

    virtual std::vector<LaserPointXY<T>>* getLaserScanXY(void) = 0;

    //通过数据是否更新的接口同步多线程，此方法不一定是最好的方法
    //后期可以考虑使用条件变量等方法来同步线程
    virtual bool ifDataUpdated(void) = 0;

    virtual bool isDataQueueEmpty(void) = 0;

    virtual void popFrontData(void) = 0;

    virtual ~LaserDataInterface(void){};

    // std::vector<LaserPoint<T>> laserScan;
    // std::vector<LaserPointXY<T>> laserScanXY;

    //std::vector<std::vector<LaserPointXY<T>>> laserScanQueue;
};




#endif