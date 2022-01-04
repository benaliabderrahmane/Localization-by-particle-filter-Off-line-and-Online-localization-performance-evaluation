#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <numeric>
using namespace std;
#define _USE_MATH_DEFINES

double likelihood(vector<double> rho, vector<double> rho_particles)
{
	double var = 0;
	double size = rho.size();
	vector<double> diff(size);
	vector<double> E(size),ee(size);

	for( int k = 0; k != size; k++){
		diff[k] = rho[k] - rho_particles[k];
	}
	double sum = accumulate(diff.begin(), diff.end(), 0.0);
	double mean = sum / diff.size();
	for( int k = 0; k != size; k++){
		var += (diff[k] - mean) * (diff[k] - mean); 
	}
	var /= diff.size();
	double sigma = sqrt(var);
	for( int k = 0; k != E.size(); k++){
		E[k] = (diff[k] - mean)/sigma;
		ee[k] = E[k] * E[k];
	}
	
	double EE = accumulate(ee.begin(), ee.end(), 0.0);
	double weight = 1/(sqrt(2*M_PI)*sigma)*exp(-0.5*EE);
	cout<<weight<<endl;
	return weight;
}

int main()
{
	vector<double> rho = {1, 2, 30, 4, 5};;
	vector<double> rho_particles = {1, 2, 3, 4, 5};;
	likelihood(rho, rho_particles);
}