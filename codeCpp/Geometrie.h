#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>
//#include <datas/sensor_data.hpp>
#include <datas/location_data.hpp>
#include <math.h>
#include <cmath>
#define _USE_MATH_DEFINES


std::vector<int> ransaclignefrompole(std::vector<struct polar_coordinate> source, double &a, double &b, double dtol , struct polar_coordinate &pref1, struct polar_coordinate &pref2);
eucclid_coordinate ConvPolToEuclide(polar_coordinate pospol);
bool Computelignecoefs(eucclid_coordinate p1, eucclid_coordinate p2, double &a, double &b);
double GetDistToLine(eucclid_coordinate p, double a, double b, eucclid_coordinate p1, eucclid_coordinate p2);
euclid_position TransphoInEuclide(euclid_position pos_d, double X_sd, double Y_sd, double Theta_sd);
polar_coordinate ConvEuclideToPol(eucclid_coordinate poseuc);
