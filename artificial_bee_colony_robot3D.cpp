// Markus Buchholz
// g++ artificial_bee_colony_robot3D.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

// plot
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

//--------Path Planner--------------------------------------------------------------
float xmin = 0.0;
float xmax = 50.0;
float ymin = 0.0;
float ymax = 50.0;
float zmin = 0.0;
float zmax = 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsZ = 25.0;
float obsR = 3.0;

float goalX = 45.0;
float goalY = 45.0;
float goalZ = 45.0;

float startX = 2.0;
float startY = 2.0;
float startZ = 2.0;

float K1 = 0.25;                     //
float K2 = 0.0000000000000000000001; //
//--------------------------------------------------------------------------------
int DIM = 3;
int EVOLUTIONS = 3;
int BEES = 200;
int LIMIT = 100; // static_cast<int>(BEES / 5);

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
    float z;
};

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

float generateRandomX()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);
    return distribution(gen);
}
//--------------------------------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{

    std::vector<float> funcValue;
    Pos Obs{obsX, obsY, obsZ};
    Pos Goal{goalX, goalY, goalZ};

    for (auto &ii : pos)
    {

        funcValue.push_back(K1 * (1 / euclid(Obs, ii)) + K2 * euclid(Goal, ii));
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

std::vector<float> computeFits(std::vector<float> valueFunction)
{

    std::vector<float> fit;

    for (auto &ii : valueFunction)
    {

        if (ii >= 0)
        {

            fit.push_back(1 / (1 + ii));
        }
        else if (ii < 0)
        {

            fit.push_back(1 + std::abs(ii));
        }
    }
    return fit;
}

//--------------------------------------------------------------------------------

float compFit(float valueFunction)
{

    float fit = 0;

    if (valueFunction >= 0)
    {

        fit = (1 / (1 + valueFunction));
    }
    else if (valueFunction < 0)
    {

        fit = (1 + std::abs(valueFunction));
    }

    return fit;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{
    Pos Obs{obsX, obsY, obsZ};
    Pos Goal{goalX, goalY, goalZ};

    return K1 * (1 / euclid(Obs, pos)) + K2 * euclid(Goal, pos);
}
//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    if (Pnew.z < ymin)
    {
        Pnew.z = ymin;
    }

    if (Pnew.z > ymax)
    {
        Pnew.z = ymax;
    }

    return Pnew;
}
//--------------------------------------------------------------------------------
int dimensionToUpdate()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_int_distribution<int> distribution(0, DIM - 1);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

Pos posNew(Pos bee, Pos partner)
{

    int dim_i = dimensionToUpdate();
    Pos Xnew;

    if (dim_i == 0)
    {

        Xnew.x = bee.x + generateRandomX() * (bee.x - partner.x);
        Xnew.y = bee.y;
        Xnew.z = bee.z;
    }
    else if (dim_i == 1)
    {

        Xnew.x = bee.x;
        Xnew.y = bee.y + generateRandomX() * (bee.y - partner.y);
        Xnew.z = bee.z;
    }
    else if (dim_i == 3)
    {

        Xnew.x = bee.x;
        Xnew.y = bee.y;
        Xnew.z = bee.z + generateRandomX() * (bee.z - partner.z);
    }

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------
Pos posScoutNew(Pos best, Pos worst)
{

    Pos Xnew;

    Xnew.x = best.x + generateRandomX() * (best.x - worst.x);

    Xnew.y = best.y + generateRandomX() * (best.y - worst.y);

    Xnew.z = best.z + generateRandomX() * (best.z - worst.z);

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < BEES; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax), valueGenerator(zmin, zmax)});
    }

    return pos;
}

//--------------------------------------------------------------------------------

Pos newPosXY()
{

    Pos pos = {valueGenerator(xmin, xmax), valueGenerator(ymin, ymax), valueGenerator(zmin, zmax) };

    return pos;
}

//-------------------------------------------------------------------------------
bool compareMax(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second > b.second;
}

//-------------------------------------------------------------------------------

// max
std::tuple<Pos, float> findWorstPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMax);

    return best[0];
}

//-------------------------------------------------------------------------------
bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}

//-------------------------------------------------------------------------------

// min
std::tuple<Pos, float> findBestPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMin);

    return best[0];
}

//-------------------------------------------------------------------------------

int choosePartner(int actual)
{

    std::random_device engine;
    std::uniform_int_distribution<int> distribution(0, BEES);

    int r = -1;

    do
    {

        r = distribution(engine);

    } while (r == actual);

    return r;
}

//-------------------------------------------------------------------------------

std::vector<float> computeProbabilities(std::vector<float> currentFitValues)
{

    std::vector<float> prob;

    float sum_prob = std::accumulate(currentFitValues.begin(), currentFitValues.end(), 0);

    for (auto &ii : currentFitValues)
    {

        prob.push_back(ii / sum_prob);
    }

    return prob;
}

//-------------------------------------------------------------------------------

std::vector<Pos> runABC()
{

    std::vector<Pos> currentPositions = initPosXY();
    std::vector<float> currentValueFunction = function(currentPositions);
    std::vector<float> currentFitValues = computeFits(currentValueFunction);
    std::vector<int> currentTrial(BEES, 0);

    for (int jj = 0; jj < EVOLUTIONS; jj++)
    {

        // EMPLOYED - all position are update
        for (int ii = 0; ii < BEES; ii++)
        {

            int partnerBee = choosePartner(ii);
            Pos newPos = posNew(currentPositions[ii], currentPositions[partnerBee]);
            float newValueFunc = func(newPos);
            float newFit = compFit(newValueFunc);
            // maximize
            if (newFit > currentFitValues[ii])
            {

                currentPositions[ii] = newPos;
                currentValueFunction[ii] = newValueFunc;
                currentFitValues[ii] = newFit;
            }
            else if (newFit <= currentFitValues[ii])
            {

                currentTrial[ii] += 1;
            }
        }

        // ONLOOKER - we introduce extra radnom for reducing update

        std::vector<float> prob = computeProbabilities(currentFitValues);
        for (int ii = 0; ii < BEES; ii++)
        {
            if (generateRandom() < prob[ii])
            {
                int partnerBee = choosePartner(ii);

                Pos newPos = posNew(currentPositions[ii], currentPositions[partnerBee]);
                float newValueFunc = func(newPos);
                float newFit = compFit(newValueFunc);
                // maximize
                if (newFit > currentFitValues[ii])
                {

                    currentPositions[ii] = newPos;
                    currentValueFunction[ii] = newValueFunc;
                    currentFitValues[ii] = newFit;
                }
                else if (newFit <= currentFitValues[ii])
                {

                    currentTrial[ii] += 1;
                }
            }

            else if (generateRandom() > prob[ii])
            {
                int Bee = ii; // choosePartner(ii);
                int partnerBee = choosePartner(Bee);
                Pos newPos = posNew(currentPositions[Bee], currentPositions[partnerBee]);
                float newValueFunc = func(newPos);
                float newFit = compFit(newValueFunc);
                // maximize
                if (newFit > currentFitValues[Bee])
                {

                    currentPositions[Bee] = newPos;
                    currentValueFunction[Bee] = newValueFunc;
                    currentFitValues[Bee] = newFit;
                }
                else if (newFit <= currentFitValues[Bee])
                {

                    currentTrial[Bee] += 1;
                }
            }
        }

        // SCOUT -- formula based on algorithm origin article
        for (int ii = 0; ii < BEES; ii++)
        {

            if (currentTrial[ii] > LIMIT)
            {

                auto best = findBestPosFuncValue(currentPositions, currentValueFunction);
                auto bestPos = std::get<0>(best);

                auto worst = findWorstPosFuncValue(currentPositions, currentValueFunction);
                auto worstPos = std::get<0>(worst);

                Pos newPos = posScoutNew(bestPos, worstPos);
                float newValueFunc = func(newPos);
                float newFit = compFit(newValueFunc);
                currentTrial[ii] = 0;
                currentPositions[ii] = newPos;
                currentValueFunction[ii] = newValueFunc;
            }
        }
    }

  

    return currentPositions;
}

//-------------------------------------------------------------------------------
std::tuple<std::vector<float>, std::vector<float>> gen_circle(float a, float b, float r)
{

    std::vector<float> xX;
    std::vector<float> yY;

    for (float dt = -M_PI; dt < M_PI; dt += 0.01)
    {

        xX.push_back(a + r * std::cos(dt));
        yY.push_back(b + r * std::sin(dt));
    }
    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------------------------------------

void plot2D(std::vector<float> xX, std::vector<float> yY)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());

    std::tuple<std::vector<float>, std::vector<float>> circle = gen_circle(obsX, obsY, obsR);

    std::vector<float> xObs = std::get<0>(circle);
    std::vector<float> yObs = std::get<1>(circle);

    plt::plot(xX, yY);
    plt::plot(xObs, yObs);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}
//---------------------------------------------------------------------
void plot3D(std::vector<float> xX, std::vector<float> yY,  std::vector<float> zZ)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());
    std::sort(zZ.begin(), zZ.end());


    plt::plot3(xX, yY, zZ);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}
//---------------------------------------------------------------------


int main()
{

    std::vector<Pos> path = runABC();

    std::vector<float> xX;
    std::vector<float> yY;
    std::vector<float> zZ;

    for (auto &ii : path)
    {
        xX.push_back(ii.x);
        yY.push_back(ii.y);
        zZ.push_back(ii.z);

        std::cout << ii.x << " ," << ii.y << " ," << ii.z <<  "\n";
    }

    plot3D(xX, yY, zZ);
}
