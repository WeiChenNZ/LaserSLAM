
#include "ceresScanMatching.h"
#include "occupiedSpaceCostFunction.h"
#include "translationCostFunction.h"
#include "rotationCostFunction.h"

CeresScanMatching::CeresScanMatching(ceres::Solver::Options& cSO){
    ceresSolverOptions_ = cSO;
    resultPose = {0., 0., 0.};
}

//destructor
CeresScanMatching::~CeresScanMatching(){}

void CeresScanMatching::Match(const Eigen::Vector3f & initialPosEstimate,
                        const std::vector<LaserPointXY<float>>& laserScanXY,
                        const ProbablisticGridMap& map,
                        Eigen::Vector3f& poseEstimate,
                        ceres::Solver::Summary* summary)const
{
    double ceresPoseEstimate[3] = {initialPosEstimate(0), initialPosEstimate(1), initialPosEstimate(2)};
    Eigen::Vector2d targetTranslation = {ceresPoseEstimate[0], ceresPoseEstimate[1]};

    ceres::Problem problem;
    
    //grid residual
    problem.AddResidualBlock(
            OccupiedSpaceCostFunction::CreateAutoDiffCostFunction(
                1./sqrt(static_cast<double>(laserScanXY.size())), //scale_factor
                laserScanXY,
                map),
                nullptr, //none loss function
                ceresPoseEstimate); 

    //translation residual
    problem.AddResidualBlock(
            TranslationCostFunction::CreateAutoDiffCostFunction(
                10,
                targetTranslation),
                nullptr,
                ceresPoseEstimate);

    //rotation residual
    problem.AddResidualBlock(
            RotationCostFunction::CreateAutoDiffCostFunction(
                40,
                ceresPoseEstimate[2]),
                nullptr,
                ceresPoseEstimate);
    
    ceres::Solve(ceresSolverOptions_, &problem, summary);
    
    poseEstimate(0) = ceresPoseEstimate[0];
    poseEstimate(1) = ceresPoseEstimate[1];
    poseEstimate(2) = ceresPoseEstimate[2];

    resultPose(poseEstimate);

}

Eigen::Vector3f CeresScanMatching::getCeresMatcherPose(void)
{
    return resultPose;
}