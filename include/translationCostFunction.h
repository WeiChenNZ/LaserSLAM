#ifndef _TRANSLATION_COST_FUNCTION_H_
#define _TRANSLATION_COST_FUNCTION_H_

#include <Eigen/Core>
#include "ceres/ceres.h"

class TranslationCostFunction{
    public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(const double scaleFactor, const Eigen::Vector2d& targetTranslation)
    {
        return new ceres::AutoDiffCostFunction<TranslationCostFunction,2,3>(
            new TranslationCostFunction(scaleFactor, targetTranslation)
        );
    }

    template<typename T>
    bool operator()(const T* const pose, T* residual) const {
        residual[0] = scaleFactor_ * (pose[0] - x_);
        residual[1] = scaleFactor_ * (pose[1] - y_);
        return true;
    }



    private:
    explicit TranslationCostFunction(const double scaleFactor, const Eigen::Vector2d& targetTranslation)
                                    :scaleFactor_(scaleFactor), 
                                     x_(targetTranslation.x()),
                                     y_(targetTranslation.y()) {}

    TranslationCostFunction(const TranslationCostFunction&) = delete;
    TranslationCostFunction& operator=(const TranslationCostFunction&) = delete;
    
    const double scaleFactor_;
    const double x_;
    const double y_;
};



#endif