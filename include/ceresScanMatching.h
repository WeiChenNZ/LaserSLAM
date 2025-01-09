#ifndef _CERES_SCAN_MATCHING_H_
#define _CERES_SCAN_MATCHING_H_

#include "ceres/ceres.h"
#include "laserDataInterface.h"
#include "probablisticGridMap.h"
#include <Eigen/Core>

 
class CeresScanMatching{
    public:
        explicit CeresScanMatching(ceres::Solver::Options& cSO);
        virtual ~CeresScanMatching();

        //disable other constructors
        CeresScanMatching(const CeresScanMatching&) = delete;
        CeresScanMatching& operator=(const CeresScanMatching&) = delete;
        
        void Match(const Eigen::Vector3f & initialPosEstimate,
                   const std::vector<LaserPointXY<float>>& laserScanXY,
                   const ProbablisticGridMap& map,
                   Eigen::Vector3f& poseEstimate,
                   ceres::Solver::Summary* summary) const;

        Eigen::Vector3f getCeresMatcherPose(void);
 

    private:
        ceres::Solver::Options ceresSolverOptions_;
        Eigen::Vector3f resultPose;
};


#endif