#include "frontEnd.h"
#include <iostream>


    void FrontEnd::init(void)
    {

    }


    void FrontEnd::run()
    {
        static int num = 0;
        if(isFirstRun && !laserScan->isDataQueueEmpty())
        {
            //first run, just add laser scan into grid map
            isFirstRun = 0;
            Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
            map->updateHighMap(laserScan->getLaserScanXY(), T);//Eigen::Matrix3f::Identity());
            map->updateLowMap();

            visual->showHighMap(map);
            //visual->showLowMap(map);

            laserScan->popFrontData();//manually delete the first data in the queue

            //sleep(2);
            //printf("-------------------------first cirle--------------------------------------\n");

        }
        else if(!laserScan->isDataQueueEmpty())
        {

            //step1: fast correlated scan matching
            std::cout<<"Start matching ..."<<std::endl;
            fastMatching->matching();
            std::cout<<"matching circle = "<<num++<<std::endl;


//////////////////////////////////////////////////////////////////////////////////////////////

            //step2: nonlinear least-squares method, ceres scan matching 
            Eigen::Vector3f ceresInitialPos= fastMatching->getScanPosVect();
            // Eigen::Vector3f ceresInitialPos = ceresScanMatcher->getCeresMatcherPose();
            Eigen::Vector3f resultPos;
            ceres::Solver::Summary summary;

                
            ceresScanMatcher->Match(ceresInitialPos,
                                    *laserScan->getLaserScanXY(), 
                                    *map, 
                                    resultPos, 
                                    &summary);   
            
            std::cout<<"fast  matching pose:"<<std::endl
                     <<"x = "<<fastMatching->getScanPosVect()[0]<<std::endl
                     <<"y = "<<fastMatching->getScanPosVect()[1]<<std::endl
                     <<"theta = "<<fastMatching->getScanPosVect()[2]*180./M_PI<<std::endl;

            //update fastmatching result equal to ceresmatching result
            fastMatching->setScanPosVect(resultPos);

            std::cout<<"ceres  matching pose:"<<std::endl
                     <<"x = "<<resultPos[0]<<std::endl
                     <<"y = "<<resultPos[1]<<std::endl
                     <<"theta = "<<resultPos[2]*180./M_PI<<std::endl;
             
            Eigen::Matrix3f T;
            Eigen::Rotation2D rotation(resultPos[2]);
            Eigen::Matrix2f rotMatrix = rotation.toRotationMatrix();
            Eigen::Matrix<float, 2, 1> trans(resultPos(0), resultPos(1));
            T << rotMatrix, trans, 0., 0., 1.;

            map->updateHighMap(laserScan->getLaserScanXY(), T); 
///////////////////////////////////////////////////////////////////////////////////////////////////

            // map->updateHighMap(laserScan->getLaserScanXY(), fastMatching->getScanPos());
            map->updateLowMap();

            visual->showLaserScanWithPos(laserScan->getLaserScanXY(), T);
            // visual->showLaserScanWithPos(laserScan->getLaserScanXY(), fastMatching->getScanPos());

            //show highmap needs to calculate 10000*10000 times so it's really slow
            if(num % 10 == 0)
                visual->showHighMap(map);
            //visual->showLowMap(map);

            visual->showPosition(resultPos);

            //delete the first data in the queue
            laserScan->popFrontData();

            std::cout<<"End of one circle"<<std::endl<<std::endl;

            //sleep(2); //delay 10s

        } 
    }


    void FrontEnd::cleanup(){}//do nothing