#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define _USE_MATH_DEFINES


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
#include <algorithm>


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
int closest(vector<double> const& vec, double value);
vector<int> selection(vector<double> weights,int N);
double var(vector<double> v);
void check_redistribution(double PoseEstime[], double OldParticles[], double Robot[], double OldRobot[], double SdX, double SdY, double SdTheta, bool &flag1, bool &flag2);


#endif