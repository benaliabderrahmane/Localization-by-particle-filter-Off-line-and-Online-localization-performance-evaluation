#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <numeric>
using namespace std;
#define _USE_MATH_DEFINES

double wrapAngle(double a)
{	
	if (a > M_PI){
		a = a -2*M_PI;
	}
	else if(a < -M_PI){
		a = a + 2*M_PI;
	}
	return a;	
}

int main()
{
	double x;
	x = wrapAngle(20);
	cout << "end\n" << x << endl;
}