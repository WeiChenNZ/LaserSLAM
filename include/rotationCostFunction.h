#ifndef _ROTATION_COST_FUNCTION_H_
#define _ROTATION_COST_FUNCTION_H_

#include <Eigen/Core>
#include "ceres/ceres.h"

class RotationCostFunction{
    public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(const double scaleFactor, const double targeAngle)
    {
        return new ceres::AutoDiffCostFunction<RotationCostFunction, 1, 3>(
            new RotationCostFunction(scaleFactor, targeAngle)
        );
    }

    template<typename T>
    bool operator()(const T* const pose, T* residual) const {
        residual[0] = scaleFactor_ * (pose[2] - angle_);
        return true;
    }



    private:
    explicit RotationCostFunction(const double scaleFactor, const double targetAngle)
                                 :scaleFactor_(scaleFactor), angle_(targetAngle){}
    
    RotationCostFunction(const RotationCostFunction&) = delete;
    RotationCostFunction& operator=(const RotationCostFunction&) = delete;

    const double scaleFactor_;
    const double angle_;
};

#endif