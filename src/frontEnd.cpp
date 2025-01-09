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
            //第一次运行，不用匹配直接把扫描加入空地图中
            isFirstRun = 0;
            Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
            map->updateHighMap(laserScan->getLaserScanXY(), T);//Eigen::Matrix3f::Identity());
            map->updateLowMap();

            visual->showHighMap(map);
            //visual->showLowMap(map);

            laserScan->popFrontData();//操作完毕后将队列中的第一个元素删除

            //sleep(2);
            //printf("-------------------------first cirle--------------------------------------\n");

        }
        else if(!laserScan->isDataQueueEmpty())
        {

            std::cout<<"Start matching ..."<<std::endl;
            fastMatching->matching();
            std::cout<<"matching circle = "<<num++<<std::endl;



//////////////////////////////////////////////////////////////////////////////////////////////
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

            //show highmap need to calculate 10000*10000 tiems so it's really slow
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