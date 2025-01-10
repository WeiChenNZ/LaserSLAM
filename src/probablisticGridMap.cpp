
#include "probablisticGridMap.h"
#include "assert.h"
#include "float.h"
#include "math.h"
#include <mutex>
#include <unordered_set>

#include <iostream>
using std::cout;
using std::clamp;


//set the probabilities of the grids which laser goes through
//the grids on the end index add hit probability
//the grids which laser goes through add miss probability
//probability is represented in the log(Odd(probability)) form
//this function is only used in the high resolution map
void ProbablisticGridMap::setLaserGridProbability(int startIndexX, int startIndexY, int endIndexX, int endIndexY)
{
    //laser line starts from start index and end at end index

    //vertical
    if(endIndexX == startIndexX)
    {
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
    //if |k| is less than 1, then the angle is less than 45°, angle in 0°~90°
    if(k == 0)
    {
        //horizontal
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
        //let grid lenght is 1, then the first and last steps are 0.5
        //if angle is less than 45°, then step lenght on x diretion is 1, on y direction is k
        int xOffset = endIndexX - startIndexX;
        int yOffset = endIndexY - startIndexY;
        float xIncrease = float(abs(xOffset)/xOffset);
        float yIncrease = float(abs(yOffset)/yOffset) * abs(k);
        int xIndex = 0, yIndex = 0;
        float x = float(startIndexX) + 0.5, y = float(startIndexY) + 0.5;

        for(int i = 0; i < abs(xOffset) + 1; i++)
        {
            if(i == 0)
            {
                //the first step
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x) ;//-1;
                yIndex = int(y);
                HighResMap[yIndex][xIndex] += MissProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else if(i == abs(xOffset))
            {
                //the last step
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x) ;//-1;
                yIndex = int(y);
                HighResMap[yIndex][xIndex] += HitProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else
            {
                //middle steps
                x += xIncrease;
                xIndex = int(x) ;//-1;

                //if y1 != y2, then laser goes through two grids on y direction
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
        //let grid lenght is 1, then the first and last steps are 0.5
        //if angle is less than 45°, then step lenght on x diretion is 1, on y direction is k
        int xOffset = endIndexX - startIndexX;
        int yOffset = endIndexY - startIndexY;
        float xIncrease = float(abs(xOffset)/xOffset) / abs(k);
        float yIncrease = float(abs(yOffset)/yOffset);
        int xIndex = 0, yIndex = 0;
        float x = float(startIndexX) + 0.5, y = float(startIndexY) + 0.5;

        for(int i = 0; i < abs(yOffset) + 1; i++)
        {
            if(i == 0)
            {
                //the first step
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x);
                yIndex = int(y) ;//-1;
                HighResMap[yIndex][xIndex] += MissProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else if(i == abs(yOffset))
            {
                //the last step
                x += xIncrease * 0.5;
                y += yIncrease * 0.5;
                xIndex = int(x);
                yIndex = int(y) ;//-1;
                HighResMap[yIndex][xIndex] += HitProbability;
                HighResMap[yIndex][xIndex] = std::clamp<float>(HighResMap[yIndex][xIndex], LogOddMin, LogOddMax);
            }
            else
            {
                //middle steps
                y += yIncrease;
                yIndex = int(y) ;//-1;
                //if x1 != x2, then laser goes through two grids on x direction
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
        //45°
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

    //x direction of the grid frame is right
    //y direction of the grid frame is downside
    startIndexX =  int(T(0,2)/pixelSize) + offsetX;
    startIndexY = -int(T(1,2)/pixelSize) + offsetY;

    assert(startIndexX <= offsetX*2 && startIndexY <= offsetY*2);

    Eigen::Vector3f endPos;
    for(int i = 0; i < scanLen; i++)
    {
        //endPos(0) = laserScan->at(i).x;
        //endPos(1) = laserScan->at(i).y;
        endPos(0) = laserScan->at(i).x;
        endPos(1) = laserScan->at(i).y;
        endPos(2) = 1.0;

        endPos = T * endPos;//transfer from laser frame to world frame

        //transfer from world frame to grid frame
        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2);

        //if the end grid is updated, then don't update it again
        //no use
        gIndex.x = endIndexX;
        gIndex.y = endIndexY;
        if(uSet.find(gIndex) != uSet.end())
        {
            //continue;
        }
        uSet.emplace(gIndex);

        setLaserGridProbability(startIndexX, startIndexY, endIndexX, endIndexY);
    }

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

        endPos = T * endPos;

        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        //assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2);
        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2 && endIndexX >= 0 && endIndexY >= 0);


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

        endPos = T * endPos;

        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2 && endIndexX >= 0 && endIndexY >= 0);


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

        endPos = T * endPos;

        endIndexX =  int(endPos(0) / pixelSize) + offsetX;
        endIndexY = -int(endPos(1) / pixelSize) + offsetY;

        assert(endIndexX <= offsetX*2 && endIndexY <= offsetY*2 && endIndexX >= 0 && endIndexY >= 0);


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

//low resolutin grid map is created from high resolution grid map
//a 10*10 grids region in high resolution grid map is merged into one grid in low resolution grid map
//the value of the grid in the low resolution grid map is the maximum value in the 10*10 region of the high resolution grid map 
void ProbablisticGridMap::updateLowMap(void)
{
    //define a inline function using lamda express
    //find out the maximum value in the 10*10 grids region of high resolution grid map
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