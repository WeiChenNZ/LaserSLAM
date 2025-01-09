#include "fastCorrelatedMatching.h"
#include<algorithm>
#include<iostream>

#define PI 3.141592653f

using std::cout;

//使用分支定界算法进行快速匹配
void FastCorrelatedMatching::matching(void)
{
    getLaserScan();
    //角度搜索范围-30～+30度，1度步长
    //尺寸地图搜索区域为1.5米x1.5米，对应的第精度搜索区域5*5栅格
    //Step1：搜索第一层低精度栅格
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
        //遍历5*5低精度栅格
        for(int i = -2; i <= 2; i++)
        // for(int i = -1; i <= 1; i++)
        {
            for(int j = -2; j <= 2; j++)
            // for(int j = -1; j <= 1; j++)
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
        //对低分辨率地图的结果排序，按照从大到小排序
        sort(lowResVec.begin(), lowResVec.end(), 
            [](MatchingResult x,MatchingResult y){return x.value > y.value;});//lamda表达式实现找最大值
        
/*       for(int i = 0; i < lowResVec.size();++i)
        {
            std::cout<<"inter:"<<i<<" "<<lowResVec.at(i).value<<std::endl;
            std::cout<<lowResVec.at(i).i<<" "<<lowResVec.at(i).j<<std::endl;
        }    */ 

           //Step2：找到最大的低分辨率栅格下的高分辨率栅格的最大匹配值，并将其作为临时的最大匹配值
        maxMatchingRate = findHighResMatchingResult(angle, lowResVec.begin()->i, lowResVec.begin()->j);

        lowResVec.erase(lowResVec.begin());//删除第一个元素，相当于删除的第一个分支，因为第一个分支已经计算完了

        //Step3：从剩下的分支下的低分辨率地图开始找，看是否有比零时maxMatchingRate大的，如果有则展开该分支，并计算高精度匹配度，如果匹配度高于maxMatchingRate,则替换
        //直到找不到为止
        while(lowResVec.size() > 0)
        {
            if(lowResVec.begin()->value < maxMatchingRate.value) break;//寻找结束
            else if(lowResVec.begin()->value >= maxMatchingRate.value)
            {
                //展开该栅格
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

        //Step4:记录该角度下的得分
        MatchingResultWithAngle result;
        result.angle = angle;
        result.value = maxMatchingRate.value;
        result.T = maxMatchingRate.T;
        //result.value = lowResVec.begin()->value;//maxMatchingRate.value;
        //result.T = lowResVec.begin()->T;//maxMatchingRate.T;
        matchingResult.push_back(result);

        lowResVec.clear();

        //std::cout<<"interation = "<<angleIndex<<" "<<"angle = "<<angle*180./3.14<<" "<<"value = "<<result.value<<" x y = "<<maxMatchingRate.i<<" "<<maxMatchingRate.j<<std::endl;
        int b =0;
    }

    //Step5:对所有角度下的得分排序
    sort(matchingResult.begin(), matchingResult.end(), 
        [](MatchingResultWithAngle x,MatchingResultWithAngle y){return x.value > y.value;});//lamda表达式实现找最大值

    // std::cout<<"final result"<<std::endl;
    // for(int i = 0; i < matchingResult.size(); ++i)
    // {
    //     std::cout<<"inter = "<<i<<" "<<"value = "<<matchingResult[i].value<<" angle = "<<matchingResult[i].angle*180./3.14<<std::endl;
    // }  

    //Step6:将最大匹配值对应的变换矩阵T赋值给Twc
    Twc = matchingResult.begin()->T;
    fastMatchingResult(2) = matchingResult.begin()->angle;
    fastMatchingResult(0) = matchingResult.begin()->T(0,2);
    fastMatchingResult(1) = matchingResult.begin()->T(1,2);

    //debug
    // std::cout<<"final result"<<std::endl;
    // std::cout<<"angle = "<<atan(Twc(1,0)/Twc(0,0))*180./3.14<<" x = "<<Twc(0,2)<<" y = "<<Twc(1,2)<<std::endl;

    int a = 0;
}

//lowI,LowJ为上一层低分辨率栅格匹配得到的index
MatchingResult FastCorrelatedMatching::findHighResMatchingResult(float angle, int lowI, int lowJ)
{
    MatchingResult HighResMatchingResult;
    std::vector<MatchingResult> highResVec;
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();

    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            //低精度坐标转高精度坐标，需要考虑小数点后的数字如何操作
            //低精度的坐标小数点后的数字丢弃，只保留低精度坐标起始位置
            int lowOffsetX = int((Twc(0,2) + (float)lowI * LowResolutionGridSize)/LowResolutionGridSize);
            T(0,2) = (float)lowOffsetX * LowResolutionGridSize + (float)i * HighResolutionGridSize;

            int lowOffsetY = int((Twc(1,2) + (float)lowJ * LowResolutionGridSize)/LowResolutionGridSize);
            T(1,2) = (float)lowOffsetY * LowResolutionGridSize + (float)j * HighResolutionGridSize;

            //R矩阵
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
            [](MatchingResult x,MatchingResult y){return x.value > y.value;});//lamda表达式实现找最大值

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
    //memcpy(&laserScanXY, laserScan->getLaserScanXY(), laserScan->getLaserScanXY()->size());

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