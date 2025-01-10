#ifndef _LASER_DATA_INTERFACE_H_
#define _LASER_DATA_INTERFACE_H_

#include <vector>

//raw laser data, in r theta mode
template<class T>
class LaserPoint{
    public:
    T distance;
    T intensity;
    T angle;
};

//decoded laser data, in x y mode
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

    //this is not the best way to synchronize data
    virtual bool ifDataUpdated(void) = 0;

    virtual bool isDataQueueEmpty(void) = 0;

    virtual void popFrontData(void) = 0;

    virtual ~LaserDataInterface(void){};

};




#endif