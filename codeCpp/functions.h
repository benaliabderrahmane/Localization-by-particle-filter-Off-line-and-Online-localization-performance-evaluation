#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define _USE_MATH_DEFINES
#define pi 3.14159265

#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <math.h>
#include <random>
#include <fstream>
#include <sstream>
#include <math.h>
#include <numeric>
#include <time.h>
#include <string>


struct Obstacles {
    double posVertex[217][2][2]; //obstacleNumber line column
    double centre [217][2]; //obstacleNumber line
    double distDetect [217]; //obstacleNumber
};

using namespace std;


vector<double> polyxpoly(double x,double x1,double y,double y1,vector<vector<double>> grandObstacle);
bool isInBoxMax(double x,double y,Obstacles obstacle);
vector<vector<double>> particleGenerator(double xMin,double xMax,double yMin, double yMax, double thetaMin, double thetaMax, int N,Obstacles obstacle);
double wrapAngle(double a);
double likelihood(vector<double> rho, vector<double> rho_particles);


#endif