#ifndef _FRONT_END_H_
#define _FRONT_END_H_

#include "laserDataInterface.h"
#include "visualizationInterface.h"
#include "probablisticGridMap.h"
#include "fastCorrelatedMatching.h"
#include "PeriodicTask.h"
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "occupiedSpaceCostFunction.h"
#include "ceresScanMatching.h"

class FrontEnd : public PeriodicTask{

    public:
    FrontEnd(LaserDataInterface<float> *mlaserScan,VisualizationInterface *visualPtr,
             PeriodicTaskManager *taskManager, float period, std::string taskName)
    :laserScan(mlaserScan), visual(visualPtr), PeriodicTask(taskManager, period, taskName)
    {
        taskManager->addTask(this);
        map = new ProbablisticGridMap();
        fastMatching = new FastCorrelatedMatching(map, mlaserScan);
        
        
        //set up ceres matcher options here
        ceres::Solver::Options options;
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        ceresScanMatcher = new CeresScanMatching(options);
    };


    void init();
    void run();
    void cleanup();


    ~FrontEnd()
    {
        delete map;
        delete fastMatching;
    }


    private:
    LaserDataInterface<float> *laserScan;
    ProbablisticGridMap *map;
    FastCorrelatedMatching *fastMatching;

    CeresScanMatching *ceresScanMatcher;

    VisualizationInterface *visual;

    bool isFirstRun = 1;

};


#endif