
#ifndef _OCCUPIED_SPACE_COST_FUNCTION_H_
#define _OCCUPIED_SPACE_COST_FUNCTION_H_

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include <Eigen/Core>
#include "laserDataInterface.h"
#include "probablisticGridMap.h"

class OccupiedSpaceCostFunction
{
    public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(
        const double mScaleFactor, 
        const std::vector<LaserPointXY<float>> &mLaserScanXY,
        const ProbablisticGridMap& mMap)
    {
            return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction, ceres::DYNAMIC, 3>(
                new OccupiedSpaceCostFunction(mScaleFactor, mLaserScanXY, mMap),
                mLaserScanXY.size()
            );
    }

    template <typename T>
    bool operator()(const T* const pose, T* residual) const{
        Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
        Eigen::Rotation2D<T> rotation(pose[2]);
        Eigen::Matrix<T, 2, 2> rotationMatix = rotation.toRotationMatrix();
        Eigen::Matrix<T, 3, 3> transform;
        transform << rotationMatix, translation, T(0.),T(0.),T(1.);

        const ProbablisticGridMapAdapter adapter(map); 
        ceres::BiCubicInterpolator<ProbablisticGridMapAdapter> interpolater(adapter);

        for(size_t i = 0; i < laserScanXY.size(); ++i)
        {
            const Eigen::Matrix<T, 3, 1> point(T(laserScanXY.at(i).x), T(laserScanXY.at(i).y), T(1.));
            const Eigen::Matrix<T, 3, 1> world = transform * point;

            //transform x,y into index_x, index_y
            T indexX ;
            T indexY ;
            //high resolution map 0.03m
            //high resolution map max index 10000
            indexX =  world[0]/0.03 + 5000.;
            indexY = -world[1]/0.03 + 5000.;
            interpolater.Evaluate(indexX, 
                                  indexY, 
                                  &residual[i]);

            residual[i] = scaleFactor * (1. - residual[i]);
            // residual[i] = scaleFactor*residual[i];
        }

        return true;
    }


    private:

    class ProbablisticGridMapAdapter{
        public:
        enum{DATA_DIMENSION = 1};
        
        explicit ProbablisticGridMapAdapter(const ProbablisticGridMap& mMap):map_(mMap){}

        void GetValue(const int column, const int row, double* const value) const{
            if(row < 0 || row > HighResMapIndex || column < 0 || column > HighResMapIndex){
                *value = 0.0;
            }
            else{
                //need to be  x + y*Index
                // double odd = pow(10.,static_cast<double>(map_.getHighMap()[row + HighResMapIndex * column]));
                double odd = pow(10.,static_cast<double>(map_.getHighMap()[column + HighResMapIndex * row]));
                *value = odd/(1.+odd);
                // *value = static_cast<double>(map_.getHighMap()[row + HighResMapIndex * column]);
            }
        }

        int NumRows() const{
            return HighResMapIndex;
        }

        int NumCols() const{
            return HighResMapIndex;
        }

        private:
        const ProbablisticGridMap& map_;

    };

    //private constructor
    OccupiedSpaceCostFunction(const double sf, const std::vector<LaserPointXY<float>> &ls, const ProbablisticGridMap &m)
    :scaleFactor(sf), laserScanXY(ls), map(m){}

    //disable other construcors
    OccupiedSpaceCostFunction(const OccupiedSpaceCostFunction&) = delete;
    OccupiedSpaceCostFunction& operator = (const OccupiedSpaceCostFunction&) = delete;

    const double scaleFactor;
    const std::vector<LaserPointXY<float>> &laserScanXY;
    const ProbablisticGridMap &map;

};


#endif