
#include "probablisticGridMap.h"
#include "assert.h"
#include "float.h"
#include "math.h"
#include <mutex>
#include <unordered_set>

#include <iostream>
using std::cout;
using std::clamp;


void ProbablisticGridMap::setLaserGridProbability_backup(int startIndexX, int startIndexY, int endIndexX, int endIndexY)
{
    HighResMap[endIndexX][endIndexY] += HitProbability;
}

//计算从startIndex出发到endIndex结束的射线
//endIndex的栅格hit概率增加
//射线穿过的其他栅格miss概率增加
//在这个函数中不判断index越界问题，由其他函数判断
//该算法只在高精度地图下使用，因为低精度地图是通过高精度地图生成的，不需要通过射线生成
void ProbablisticGridMap::setLaserGridProbability(int startIndexX, int startIndexY, int endIndexX, int endIndexY)
{
    //从起点栅格的中心出发到终点栅格的中心结束
    if(endIndexX == startIndexX)
    {
        //射线垂直
        if(startIndexY < endIndexY)
        {
            for(int j = startIndexY; j < endIndexY; j++)
            {
                HighResMap[j][startIndexX] += MissProbability;
                HighResMap[j][startIndexX] = std::clamp<float>(HighResMap[j][startIndexX], LogOddMin, LogOddMax);
            }
            HighResMap[endIndexY][endIndexX] += HitProbability;
            HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
        }
        else
        {
            for(int j = startIndexY; j > endIndexY; j--)
            {
                HighResMap[j][startIndexX] += MissProbability;
                HighResMap[j][startIndexX] = std::clamp<float>(HighResMap[j][startIndexX], LogOddMin, LogOddMax);
            }
            HighResMap[endIndexY][endIndexX] += HitProbability;
            HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
        }
        return;
    }


    float k = float(endIndexY - startIndexY) / float(endIndexX - startIndexX);
    //判断|k|是否小于1,如果小于1则说明角度小于45度，大于1则说明角度大于45度（0-90度范围）
    if(k == 0)
    {
        //射线水平
        if(startIndexX < endIndexX)
        {
            for(int i  = startIndexX; i < endIndexX; i++)
            {
                HighResMap[endIndexY][i] += MissProbability;
                HighResMap[endIndexY][i] = std::clamp<float>(HighResMap[endIndexY][i], LogOddMin, LogOddMax);
            }
            HighResMap[endIndexY][endIndexX] += HitProbability;
            HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
        }
        else
        {
            for(int i = startIndexX; i > endIndexX; i--)
            {
                HighResMap[endIndexY][i] += MissProbability;
                HighResMap[endIndexY][i] = std::clamp<float>(HighResMap[endIndexY][i], LogOddMin, LogOddMax);
            }
            HighResMap[endIndexY][endIndexX] += HitProbability;
            HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
        }
    }
    else if(abs(k) < 1)
    {
        //假设栅格的边长为1,则第一步和最后一步只走0.5
        //小于45度时，按照x步长为1,y的步长为k增加
        int xOffset = endIndexX - startIndexX;
        int yOffset = endIndexY - startIndexY;
        float xIncrease = float(abs(xOffset)/xOffset);
        float yIncrease = float(abs(yOffset)/yOffset) * abs(k);
        int xIndex = 0, yIndex = 0;
        float x = float(startIndexX) + 0.5, y = float(startIndexY) + 0.5;

        //走xOffset + 1步从起点走到终点，因为第1和最后一步都是半步长
        for(int i = 0; i < abs(xOffset) + 1; i++)
        {
            if(i == 0)
            {
                //第一步
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x) ;//-1;
                yIndex = int(y);
                HighResMap[yIndex][xIndex] += MissProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else if(i == abs(xOffset))
            {
                //最后一步
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x) ;//-1;
                yIndex = int(y);
                HighResMap[yIndex][xIndex] += HitProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else
            {
                //中间步
                x += xIncrease;
                xIndex = int(x) ;//-1;
                //y方向有可能一步跨越两个格子，如果y1 ！= y2则跨越两个格子，需要更新两个
                int y1 = int(y);
                y += yIncrease;
                int y2 = int(y);
                if(y1 != y2)
                {
                    HighResMap[y1][xIndex] += MissProbability;
                    HighResMap[y1][xIndex] = std::clamp<float>(HighResMap[y1][xIndex], LogOddMin, LogOddMax);
                    HighResMap[y2][xIndex] += MissProbability;
                    HighResMap[y2][xIndex] = std::clamp<float>(HighResMap[y2][xIndex], LogOddMin, LogOddMax);
                }
                else 
                {
                    HighResMap[y1][xIndex] += MissProbability;
                    HighResMap[y1][xIndex] = std::clamp<float>(HighResMap[y1][xIndex], LogOddMin, LogOddMax);
                }
            }
        }


    } 
    else if(abs(k) > 1)
    {
        //假设栅格的边长为1,则第一步和最后一步只走0.5
        //大于45度时，按照x步长为1/k,y的步长为1增加
        int xOffset = endIndexX - startIndexX;
        int yOffset = endIndexY - startIndexY;
        float xIncrease = float(abs(xOffset)/xOffset) / abs(k);
        float yIncrease = float(abs(yOffset)/yOffset);
        int xIndex = 0, yIndex = 0;
        float x = float(startIndexX) + 0.5, y = float(startIndexY) + 0.5;

        //走xOffset + 1步从起点走到终点，因为第1和最后一步都是半步长
        for(int i = 0; i < abs(yOffset) + 1; i++)
        {
            if(i == 0)
            {
                //第一步
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x);
                yIndex = int(y) ;//-1;
                HighResMap[yIndex][xIndex] += MissProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else if(i == abs(yOffset))
            {
                //最后一步
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x);
                yIndex = int(y) ;//-1;
                HighResMap[yIndex][xIndex] += HitProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else
            {
                //中间步
                y += yIncrease;
                yIndex = int(y) ;//-1;
                //x方向有可能一步跨越两个格子，如果x1 ！= x2则跨越两个格子，需要更新两个
                int x1 = int(x);
                x += xIncrease;
                int x2 = int(x);
                if(x1 != x2)
                {
                    HighResMap[yIndex][x1] += MissProbability;
                    HighResMap[yIndex][x1] = std::clamp<float>(HighResMap[yIndex][x1], LogOddMin, LogOddMax);
                    HighResMap[yIndex][x2] += MissProbability;
                    HighResMap[yIndex][x2] = std::clamp<float>(HighResMap[yIndex][x2], LogOddMin, LogOddMax);
                }
                else 
                {
                    HighResMap[yIndex][x1] += MissProbability;
                    HighResMap[yIndex][x1] = std::clamp<float>(HighResMap[yIndex][x1], LogOddMin, LogOddMax);
                }
            }
        }
    }
    else if(abs(k) == 1)
    {
        //射线45度
        int xOffset = endIndexX - startIndexX;
        int yOffset = endIndexY - startIndexY;
        int xIncrease = abs(xOffset)/xOffset;
        int yIncrease = abs(yOffset)/yOffset;

        int i = startIndexX, j = startIndexY;
        while(i != endIndexX)
        {
            HighResMap[j][i] += MissProbability;
            i += xIncrease;
            j += yIncrease;
        }
        HighResMap[endIndexY][endIndexX] += HitProbability;
        HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
    }

    return;

}

// void ProbablisticGridMap::setLaserGridProbability(int startIndexX, int startIndexY, int endIndexX, int endIndexY)
// {
//     //从起点栅格的中心出发到终点栅格的中心结束
//     if(endIndexX == startIndexX)
//     {
//         //射线垂直
//         if(startIndexY < endIndexY)
//         {
//             for(int j = startIndexY; j < endIndexY; j++)
//             {
//                 HighResMap[j][startIndexX] += MissProbability;
//                 // HighResMap[j][startIndexX] = std::clamp<float>(HighResMap[j][startIndexX], LogOddMin, LogOddMax);
//             }
//             HighResMap[endIndexY][endIndexX] += HitProbability;
//             // HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
//         }
//         else
//         {
//             for(int j = startIndexY; j > endIndexY; j--)
//             {
//                 HighResMap[j][startIndexX] += MissProbability;
//                 // HighResMap[j][startIndexX] = std::clamp<float>(HighResMap[j][startIndexX], LogOddMin, LogOddMax);
//             }
//             HighResMap[endIndexY][endIndexX] += HitProbability;
//             // HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
//         }
//         return;
//     }


//     float k = float(endIndexY - startIndexY) / float(endIndexX - startIndexX);
//     //判断|k|是否小于1,如果小于1则说明角度小于45度，大于1则说明角度大于45度（0-90度范围）
//     if(k == 0)
//     {
//         //射线水平
//         if(startIndexX < endIndexX)
//         {
//             for(int i  = startIndexX; i < endIndexX; i++)
//             {
//                 HighResMap[endIndexY][i] += MissProbability;
//                 // HighResMap[endIndexY][i] = std::clamp<float>(HighResMap[endIndexY][i], LogOddMin, LogOddMax);
//             }
//             HighResMap[endIndexY][endIndexX] += HitProbability;
//             // HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
//         }
//         else
//         {
//             for(int i = startIndexX; i > endIndexX; i--)
//             {
//                 HighResMap[endIndexY][i] += MissProbability;
//                 // HighResMap[endIndexY][i] = std::clamp<float>(HighResMap[endIndexY][i], LogOddMin, LogOddMax);
//             }
//             HighResMap[endIndexY][endIndexX] += HitProbability;
//             // HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
//         }
//     }
//     else if(abs(k) < 1)
//     {
//         //假设栅格的边长为1,则第一步和最后一步只走0.5
//         //小于45度时，按照x步长为1,y的步长为k增加
//         int xOffset = endIndexX - startIndexX;
//         int yOffset = endIndexY - startIndexY;
//         float xIncrease = float(abs(xOffset)/xOffset);
//         float yIncrease = float(abs(yOffset)/yOffset) * abs(k);
//         int xIndex = 0, yIndex = 0;
//         float x = float(startIndexX) + 0.5, y = float(startIndexY) + 0.5;

//         //走xOffset + 1步从起点走到终点，因为第1和最后一步都是半步长
//         for(int i = 0; i < abs(xOffset) + 1; i++)
//         {
//             if(i == 0)
//             {
//                 //第一步
//                 x += xIncrease * 0.5;
//                 y += yIncrease * 0.5;
//                 xIndex = int(x) ;//-1;
//                 yIndex = int(y);
//                 HighResMap[yIndex][xIndex] += MissProbability;
//                 // HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
//             }
//             else if(i == abs(xOffset))
//             {
//                 //最后一步
//                 x += xIncrease * 0.5;
//                 y += yIncrease * 0.5;
//                 xIndex = int(x) ;//-1;
//                 yIndex = int(y);
//                 HighResMap[yIndex][xIndex] += HitProbability;
//                 // HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
//             }
//             else
//             {
//                 //中间步
//                 x += xIncrease;
//                 xIndex = int(x) ;//-1;
//                 //y方向有可能一步跨越两个格子，如果y1 ！= y2则跨越两个格子，需要更新两个
//                 int y1 = int(y);
//                 y += yIncrease;
//                 int y2 = int(y);
//                 if(y1 != y2)
//                 {
//                     HighResMap[y1][xIndex] += MissProbability;
//                     // HighResMap[y1][xIndex] = std::clamp<float>(HighResMap[y1][xIndex], LogOddMin, LogOddMax);
//                     HighResMap[y2][xIndex] += MissProbability;
//                     // HighResMap[y2][xIndex] = std::clamp<float>(HighResMap[y2][xIndex], LogOddMin, LogOddMax);
//                 }
//                 else 
//                 {
//                     HighResMap[y1][xIndex] += MissProbability;
//                     // HighResMap[y1][xIndex] = std::clamp<float>(HighResMap[y1][xIndex], LogOddMin, LogOddMax);
//                 }
//             }
//         }


//     } 
//     else if(abs(k) > 1)
//     {
//         //假设栅格的边长为1,则第一步和最后一步只走0.5
//         //大于45度时，按照x步长为1/k,y的步长为1增加
//         int xOffset = endIndexX - startIndexX;
//         int yOffset = endIndexY - startIndexY;
//         float xIncrease = float(abs(xOffset)/xOffset) / abs(k);
//         float yIncrease = float(abs(yOffset)/yOffset);
//         int xIndex = 0, yIndex = 0;
//         float x = float(startIndexX) + 0.5, y = float(startIndexY) + 0.5;

//         //走xOffset + 1步从起点走到终点，因为第1和最后一步都是半步长
//         for(int i = 0; i < abs(yOffset) + 1; i++)
//         {
//             if(i == 0)
//             {
//                 //第一步
//                 x += xIncrease * 0.5;
//                 y += yIncrease * 0.5;
//                 xIndex = int(x);
//                 yIndex = int(y) ;//-1;
//                 HighResMap[yIndex][xIndex] += MissProbability;
//                 // HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
//             }
//             else if(i == abs(yOffset))
//             {
//                 //最后一步
//                 x += xIncrease * 0.5;
//                 y += yIncrease * 0.5;
//                 xIndex = int(x);
//                 yIndex = int(y) ;//-1;
//                 HighResMap[yIndex][xIndex] += HitProbability;
//                 // HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
//             }
//             else
//             {
//                 //中间步
//                 y += yIncrease;
//                 yIndex = int(y) ;//-1;
//                 //x方向有可能一步跨越两个格子，如果x1 ！= x2则跨越两个格子，需要更新两个
//                 int x1 = int(x);
//                 x += xIncrease;
//                 int x2 = int(x);
//                 if(x1 != x2)
//                 {
//                     HighResMap[yIndex][x1] += MissProbability;
//                     // HighResMap[yIndex][x1] = std::clamp<float>(HighResMap[yIndex][x1], LogOddMin, LogOddMax);
//                     HighResMap[yIndex][x2] += MissProbability;
//                     // HighResMap[yIndex][x2] = std::clamp<float>(HighResMap[yIndex][x2], LogOddMin, LogOddMax);
//                 }
//                 else 
//                 {
//                     HighResMap[yIndex][x1] += MissProbability;
//                     // HighResMap[yIndex][x1] = std::clamp<float>(HighResMap[yIndex][x1], LogOddMin, LogOddMax);
//                 }
//             }
//         }
//     }
//     else if(abs(k) == 1)
//     {
//         //射线45度
//         int xOffset = endIndexX - startIndexX;
//         int yOffset = endIndexY - startIndexY;
//         int xIncrease = abs(xOffset)/xOffset;
//         int yIncrease = abs(yOffset)/yOffset;

//         int i = startIndexX, j = startIndexY;
//         while(i != endIndexX)
//         {
//             HighResMap[j][i] += MissProbability;
//             i += xIncrease;
//             j += yIncrease;
//         }
//         HighResMap[endIndexY][endIndexX] += HitProbability;
//         // HighResMap[endIndexY][endIndexX] = std::clamp<float>(HighResMap[endIndexY][endIndexX], LogOddMin, LogOddMax);
//     }

//     return;

// }



//T是scan在世界坐标系下的坐标，转换到栅格map上需要做一个坐标变换
//栅格map（0,0）点在坐标系左上角
void ProbablisticGridMap::updateHighMap(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T)
{
    int scanLen = laserScan->size();
    int startIndexX = 0, startIndexY = 0, endIndexX = 0, endIndexY = 0;
    float pixelSize = 0.0;
    int offsetX = 0, offsetY = 0;

    GridIndex gIndex;
    std::unordered_set<GridIndex, GridIndexHash> uSet;
    

    pixelSize = HighResolutionGridSize;
    offsetX = HighResMapIndex/2;
    offsetY = HighResMapIndex/2;

    //栅格坐标系的x轴向右，y轴向下（和世界坐标系相反）
    startIndexX =  int(T(0,2)/pixelSize) + offsetX;
    startIndexY = -int(T(1,2)/pixelSize) + offsetY;

    assert(startIndexX <= offsetX*2 && startIndexY <= offsetY*2);//判断index是否越界

    Eigen::Vector3f endPos;
    for(int i = 0; i < scanLen; i++)
    {
        //endPos(0) = laserScan->at(i).x;
        //endPos(1) = laserScan->at(i).y;
        endPos(0) = laserScan->at(i).x;
        endPos(1) = laserScan->at(i).y;
        endPos(2) = 1.0;

        endPos = T * endPos;//雷达坐标系转世界坐标系

        //世界坐标系转栅格坐标系
        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2);//判断index是否越界

        //该代码段判断end index是否已经出现过，如果出现过则不用再次设置栅格概率值
        //保证一帧数据只更新一个栅格概率值一次
        gIndex.x = endIndexX;
        gIndex.y = endIndexY;
        if(uSet.find(gIndex) != uSet.end())
        {
            //已经包含了该元素，什么都不做
            //continue;
        }
        uSet.emplace(gIndex);

        setLaserGridProbability(startIndexX, startIndexY, endIndexX, endIndexY);
    }

    //temporary solution , should not use in real practice
    // for(int i = 0; i < 10000; i ++)
    //     for(int j = 0; j < 10000; j ++)
    //     {
    //         if(HighResMap[i][j] > 1.) HighResMap[i][j] = 1.;
    //         else if(HighResMap[i][j] < -1.) HighResMap[i][j] = -1.;
    //     }

}

float ProbablisticGridMap::getScanProbability(int resolution, std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T)
{
    float pixelSize = 0.0;
    int offsetX = 0, offsetY = 0;
    float* map;

    float odd = 0., probability = 0.;

    if(resolution == HighRes) 
    {
        pixelSize = HighResolutionGridSize;
        offsetX = HighResMapIndex/2;
        offsetY = HighResMapIndex/2;
        map = &HighResMap[0][0];
    }
    else if(resolution == LowRes)
    {
        pixelSize = LowResolutionGridSize;
        offsetX = LowResMapIndex/2;
        offsetY = LowResMapIndex/2;
        map = &LowResMap[0][0];
    } 

    Eigen::Vector3f endPos;
    int scanLen = laserScan->size();
    int endIndexX = 0, endIndexY = 0;

    float totalProbablility = 0.0;
    //float probability = 0.0;

    for(int i = 0; i < scanLen; i++)
    {
        endPos(0) = laserScan->at(i).x;
        endPos(1) = laserScan->at(i).y;
        endPos(2) = 1.0;

        endPos = T * endPos;//雷达坐标系转世界坐标系

        //世界坐标系转栅格坐标系
        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        //assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2);//判断index是否越界
        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2 && endIndexX >= 0 && endIndexY >= 0);//判断index是否越界


        //because points of longer distances are hard to match, so they should be assigned large sigma assosiated with their distance
        // probability = map[endIndexY*offsetX*2 + endIndexX] * sqrt(pow((endPos(0) - T(0,2)), 2) + pow((endPos(1) - T(1,2)), 2));
        // totalProbablility += probability;
        
        //bug 
        //need to be  x + y*rowIndex
        // odd = pow(10, map[endIndexX + offsetX*2*endIndexY]);
        // probability = odd / (1 + odd);
        // totalProbablility += probability;

        totalProbablility += map[endIndexX + offsetX*2*endIndexY];

    }

    //uniform with data length
    totalProbablility /= static_cast<float>(scanLen);

    return totalProbablility;

}

float ProbablisticGridMap::getScanProbabilityInHighMap(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T)
{
    float pixelSize = 0.0;
    int offsetX = 0, offsetY = 0;

    pixelSize = HighResolutionGridSize;
    offsetX = HighResMapIndex/2;
    offsetY = HighResMapIndex/2;

    Eigen::Vector3f endPos;
    int scanLen = laserScan->size();
    int endIndexX = 0, endIndexY = 0;

    float totalProbablility = 0.0;
    float probability = 0.0;

    for(int i = 0; i < scanLen; i++)
    {
        endPos(0) = laserScan->at(i).x;
        endPos(1) = laserScan->at(i).y;
        endPos(2) = 1.0;

        endPos = T * endPos;//雷达坐标系转世界坐标系

        //世界坐标系转栅格坐标系
        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        //assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2);//判断index是否越界
        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2 && endIndexX >= 0 && endIndexY >= 0);//判断index是否越界


        //because points of longer distances are hard to match, so they should be assigned large sigma assosiated with their distance
        //probability = map[endIndexX*offsetX*2 + endIndexY] * (pow((endPos(0) - T(0,2)), 2) + pow((endPos(1) - T(1,2)), 2));
        //totalProbablility += probability;
        
        //bug 
        //need to be  x + y*rowIndex
        //totalProbablility += map[endIndexX*offsetX*2 + endIndexY];
        totalProbablility += HighResMap[endIndexY][endIndexX];//*(pow((endPos(0) - T(0,2)), 2) + pow((endPos(1) - T(1,2)), 2));
    }

    totalProbablility /= static_cast<float>(scanLen);

    return totalProbablility;
}

float ProbablisticGridMap::getScanProbabilityInLowMap(std::vector<LaserPointXY<float>>* laserScan, Eigen::Matrix3f T)
{
    float pixelSize = 0.0;
    int offsetX = 0, offsetY = 0;

    pixelSize = LowResolutionGridSize;
    offsetX = LowResMapIndex/2;
    offsetY = LowResMapIndex/2;

    Eigen::Vector3f endPos;
    int scanLen = laserScan->size();
    int endIndexX = 0, endIndexY = 0;

    float totalProbablility = 0.0;
    float probability = 0.0;

    float* map = &LowResMap[0][0];

    for(int i = 0; i < scanLen; i++)
    {
        endPos(0) = laserScan->at(i).x;
        endPos(1) = laserScan->at(i).y;
        endPos(2) = 1.0;

        endPos = T * endPos;//雷达坐标系转世界坐标系

        //世界坐标系转栅格坐标系
        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        //assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2);//判断index是否越界
        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2 && endIndexX >= 0 && endIndexY >= 0);//判断index是否越界


        //because points of longer distances are hard to match, so they should be assigned large sigma assosiated with their distance
        //probability = map[endIndexX*offsetX*2 + endIndexY] * (pow((endPos(0) - T(0,2)), 2) + pow((endPos(1) - T(1,2)), 2));
        //totalProbablility += probability;
        
        //bug 
        //need to be  x + y*rowIndex
        //totalProbablility += map[endIndexX*offsetX*2 + endIndexY];

        //float d1 = map[endIndexX + 2*offsetX* endIndexY];
        //float d2 = LowResMap[endIndexY][endIndexX];

        //std::cout<<"laser point : "<<i<<" d1 = "<<d1<<" d2 = "<<d2<<std::endl;

        totalProbablility += LowResMap[endIndexY][endIndexX];

    }

    return totalProbablility;

}

//低精度地图栅格的更新由高精度地图生成
void ProbablisticGridMap::updateLowMap(void)
{
    //lamda表达式定义一个寻找10*10局部栅格最大值的内联函数
    //用于在高精度栅格内的局部空间寻找最大值并赋值给低精度栅格
    auto findMaxGridVal = [&](int row, int col){
        float maxVal = -FLT_MAX;
        for(int i = row; i < row + 10; i++)
        {
            for(int j = col; j < col + 10; j++)
            {
                if(HighResMap[i][j] > maxVal) maxVal = HighResMap[i][j];
            }
        }
        return maxVal;
    };

    //遍历高精度地图，更新低精度地图
    int m = 0, n = 0;
    for(int i = 0; i < HighResMapIndex; i += 10)
    {
        for(int j = 0; j < HighResMapIndex; j += 10)
        {
            LowResMap[m][n] = findMaxGridVal(i, j);
            n++;

            //if(LowResMap[m][n] != 0.0)
            //cout<<LowResMap[m][n]<<" ";

        }
        m++;
        n = 0;
    }
    //cout<<"\n";
}


const float * ProbablisticGridMap::getHighMap (void) const
{
    return &HighResMap[0][0];
}

const float * ProbablisticGridMap::getLowMap(void) const
{
    return &LowResMap[0][0];
}

void ProbablisticGridMap::clearHighMap(void)
{
    for(int i = 0; i < HighResMapIndex; i++)
    {
        for(int j = 0; j < HighResMapIndex; j++)
        {
            HighResMap[i][j] = 0;
        }
    }
}

void ProbablisticGridMap::clearLowMap(void)
{
    for(int i = 0; i < LowResMapIndex; i++)
    {
        for(int j = 0; j < LowResMapIndex; j++)
        {
            HighResMap[i][j] = 0;
        }
    }
}