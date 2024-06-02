#include <iostream>
#include <chrono>
#include <thread>
#include "PathPlanner.hpp"
#include "Visualize.hpp"

Visualize *window = new Visualize(1600);
double runTime = 0.01;
double A = 22, B = 14;
optimalPlanner plannerType = PLANNER_RRTSTAR;
planningObjective objectiveType = OBJECTIVE_PATHLENGTH;

using namespace std;

int main()
{
    vector<Point2D> obs;
    
    obs.push_back(Point2D(1, 1, 0));

    Point2D nowPos(0, 0, 0), finalPos(5, 5, 0), Ball(0, 1, 0);

    vector<pair<double, double>> points = plan(runTime, A, B, obs, plannerType, objectiveType, nowPos, finalPos);

    vector<Point2D> targetPos;

    for (auto ptr : points)
    {
        cout<<ptr.first<<" "<<ptr.second<<'\n';
        int angle = (180.0 / 3.14159) * atan2((Ball.y - ptr.second), (Ball.x - ptr.first));
        targetPos.push_back(Point2D(ptr.first, ptr.second, angle));
    }

    window->visualizeGame(targetPos, nowPos, 20, 0.0, obs, Ball);
}