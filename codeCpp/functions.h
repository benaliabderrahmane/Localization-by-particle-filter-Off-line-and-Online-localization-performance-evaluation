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
#include <numeric>
#include <time.h>
#include <string>
#include <algorithm>
#include <bits/stdc++.h>

#define pdd pair<double, double>

using namespace std;

struct Obstacles {
    double posVertex[217][2][2]; //obstacleNumber line column
    double centre [217][2]; //obstacleNumber line
    double distDetect [217]; //obstacleNumber
};


bool isInBoxMax(double x,double y,Obstacles obstacle);
vector<vector<double>> particleGenerator(double xMin,double xMax,double yMin, double yMax, double thetaMin, double thetaMax, int N,Obstacles obstacle);
double wrapAngle(double a);
double likelihood(vector<double> rho, vector<double> rho_particles);
int closest(vector<double> const& vec, double value);
vector<int> selection(vector<double> weights,int N);
double var(vector<double> v);
void check_redistribution(double PoseEstime[], double OldParticles[], double Robot[], double OldRobot[], double SdX, double SdY, double SdTheta, bool &flag1, bool &flag2);
void displayPoint(pdd P);
pdd Intersection_seg(pdd A, pdd B, pdd C, pdd D);
vector<pdd> Intersection(double x1, double x2, double y1, double y2, vector<vector<double>> map);
void uspatch_act(double Portee, vector<double> angles, vector<vector<double>> obstacle,
vector <double> distDetect, double x, double y, double theta,
vector<double> &DIST, vector<double> &ximp, vector<double> &yimp);

void Mesure_act(double Portee, vector<double> theta, vector<vector<double>> obstacle,
vector <double> distDetect, double xROB,double yROB,double thetaROB,
vector<double> &rho, vector<double> &ximp, vector<double> &yimp);

#endif
