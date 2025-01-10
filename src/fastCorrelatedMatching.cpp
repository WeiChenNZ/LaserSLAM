#include "fastCorrelatedMatching.h"
#include<algorithm>
#include<iostream>

#define PI 3.141592653f

using std::cout;

//branch and bound algorithm
void FastCorrelatedMatching::matching(void)
{
    getLaserScan();

    //angular search window -5° ~ +5°
    //x search window -2 ~ +2
    //y search window -2 ~ +2

    //Step1：search low resolution grid map
    float angle = 0.0, xOffset = 0.0, yOffset = 0.0;
    float mark = 0.0;
    MatchingResult lowResMatchingResult;
    std::vector<MatchingResult> lowResVec;
    std::vector<MatchingResultWithAngle> matchingResult;
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();

    MatchingResult maxMatchingRate;

    // for(int angleIndex = -30; angleIndex <= 30; angleIndex++)
    for(int angleIndex = -5; angleIndex <= 5; angleIndex++)
    {
        angle = (float)angleIndex * M_PI / 180.0 + fastMatchingResult(2);
        //search x y window
        for(int i = -2; i <= 2; i++)
        {
            for(int j = -2; j <= 2; j++)
            {
                xOffset = (float)i * LowResolutionGridSize;
                yOffset = (float)j * LowResolutionGridSize;
                T(0,0) = T(1,1) = cos(angle);
                T(0,1) = -sin(angle);
                T(1,0) = sin(angle);
                T(0,2) = fastMatchingResult(0) + xOffset;
                T(1,2) = fastMatchingResult(1) + yOffset;
                mark = map->getScanProbability(LowRes, &laserScanXY, T);
                //mark = map->getScanProbabilityInLowMap(&laserScanXY, T);
                lowResMatchingResult.i = i;
                lowResMatchingResult.j = j;
                lowResMatchingResult.value = mark;
                lowResMatchingResult.T = T;
                lowResVec.push_back(lowResMatchingResult);
            }
        }
        //find maximum value 
        sort(lowResVec.begin(), lowResVec.end(), 
            [](MatchingResult x,MatchingResult y){return x.value > y.value;});//lamda表达式实现找最大值
        
/*       for(int i = 0; i < lowResVec.size();++i)
        {
            std::cout<<"inter:"<<i<<" "<<lowResVec.at(i).value<<std::endl;
            std::cout<<lowResVec.at(i).i<<" "<<lowResVec.at(i).j<<std::endl;
        }    */ 

        //Step2：find temporary maximun value in the high resolution map grids
        maxMatchingRate = findHighResMatchingResult(angle, lowResVec.begin()->i, lowResVec.begin()->j);

        lowResVec.erase(lowResVec.begin());//pop the first element

        //Step3：chech the other branches and find out if there are larger values in these branches
        while(lowResVec.size() > 0)
        {
            if(lowResVec.begin()->value < maxMatchingRate.value) break;//end 
            else if(lowResVec.begin()->value >= maxMatchingRate.value)
            {
                MatchingResult tempMax = findHighResMatchingResult(angle, lowResVec.begin()->i, lowResVec.begin()->j);
                if( maxMatchingRate.value < tempMax.value)
                {
                    maxMatchingRate.value = tempMax.value;
                    maxMatchingRate.i = tempMax.i;
                    maxMatchingRate.j = tempMax.j;
                    maxMatchingRate.T = tempMax.T;
                }
                lowResVec.erase(lowResVec.begin());
            }
        }     

        //Step4:record the largest value in this angle
        MatchingResultWithAngle result;
        result.angle = angle;
        result.value = maxMatchingRate.value;
        result.T = maxMatchingRate.T;
        //result.value = lowResVec.begin()->value;//maxMatchingRate.value;
        //result.T = lowResVec.begin()->T;//maxMatchingRate.T;
        matchingResult.push_back(result);

        lowResVec.clear();

        //std::cout<<"interation = "<<angleIndex<<" "<<"angle = "<<angle*180./3.14<<" "<<"value = "<<result.value<<" x y = "<<maxMatchingRate.i<<" "<<maxMatchingRate.j<<std::endl;
    }

    //Step5:sort all values in each angle
    sort(matchingResult.begin(), matchingResult.end(), 
        [](MatchingResultWithAngle x,MatchingResultWithAngle y){return x.value > y.value;});//lamda表达式实现找最大值

    // std::cout<<"final result"<<std::endl;
    // for(int i = 0; i < matchingResult.size(); ++i)
    // {
    //     std::cout<<"inter = "<<i<<" "<<"value = "<<matchingResult[i].value<<" angle = "<<matchingResult[i].angle*180./3.14<<std::endl;
    // }  

    //Step6:find out the maximum value and set it to the result
    Twc = matchingResult.begin()->T;
    fastMatchingResult(2) = matchingResult.begin()->angle;
    fastMatchingResult(0) = matchingResult.begin()->T(0,2);
    fastMatchingResult(1) = matchingResult.begin()->T(1,2);

    //debug
    // std::cout<<"final result"<<std::endl;
    // std::cout<<"angle = "<<atan(Twc(1,0)/Twc(0,0))*180./3.14<<" x = "<<Twc(0,2)<<" y = "<<Twc(1,2)<<std::endl;

}


MatchingResult FastCorrelatedMatching::findHighResMatchingResult(float angle, int lowI, int lowJ)
{
    MatchingResult HighResMatchingResult;
    std::vector<MatchingResult> highResVec;
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();

    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            //calculate x,y index in high resolution grid map
            //transfer x,y index into world frame
            int lowOffsetX = int((Twc(0,2) + (float)lowI * LowResolutionGridSize)/LowResolutionGridSize);
            T(0,2) = (float)lowOffsetX * LowResolutionGridSize + (float)i * HighResolutionGridSize;

            int lowOffsetY = int((Twc(1,2) + (float)lowJ * LowResolutionGridSize)/LowResolutionGridSize);
            T(1,2) = (float)lowOffsetY * LowResolutionGridSize + (float)j * HighResolutionGridSize;

            //rotaion matrix
            T(0,0) = T(1,1) = cos(angle);
            T(0,1) = -sin(angle);
            T(1,0) = sin(angle);

            HighResMatchingResult.value = map->getScanProbability(HighRes, &laserScanXY, T);
            //HighResMatchingResult.value = map->getScanProbabilityInHighMap(&laserScanXY, T);
            HighResMatchingResult.i = i;
            HighResMatchingResult.j = j;
            HighResMatchingResult.T = T;
            highResVec.push_back(HighResMatchingResult);
        }
    }

    sort(highResVec.begin(), highResVec.end(), 
            [](MatchingResult x,MatchingResult y){return x.value > y.value;});//lamda expression

    return *highResVec.begin();
}

void FastCorrelatedMatching::getLaserScan(void)
{
    
    laserScanXY.clear();
    //rwLocker.lock();
    for(int i = 0; i < laserScan->getLaserScanXY()->size(); ++i)
    {
        laserScanXY.push_back(laserScan->getLaserScanXY()->at(i));
    }

    //rwLocker.unlock();
}

Eigen::Matrix3f FastCorrelatedMatching::getScanPos(void)
{
    return Twc;
}

Eigen::Vector3f FastCorrelatedMatching::getScanPosVect(void)
{
    return fastMatchingResult;
}

void FastCorrelatedMatching::setScanPosVect(Eigen::Vector3f pose)
{
    fastMatchingResult = pose;
}

float FastCorrelatedMatching::getAngle(void)
{
    return fastMatchingResult(2);
}