#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <numeric>
#include <cmath>  
using namespace std;

double var(vector<double> v){
    double variance = 0;
    double sum = accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    for( int k = 0; k != v.size(); k++){
        variance += (v[k] - mean) * (v[k] - mean); 
        }
    variance /= v.size();
    return variance;
}


void check_redistribution(double PoseEstime[], double OldParticles[], double Robot[], double OldRobot[], double SdX, double SdY, double SdTheta, bool &flag1, bool &flag2)
{
	double Distance_Particles = sqrt(pow( (PoseEstime[0] - OldParticles[0]) , 2) + pow( (PoseEstime[1] - OldParticles[1]) , 2)); 
	double Distance_robot = sqrt(pow( (Robot[0] - OldRobot[0]) , 2) + pow( (Robot[1] - OldRobot[1]) , 2));
	
	if((abs(Distance_Particles - Distance_robot) > 2.5)){
		flag1 = true;
		cout<<"flag 1:\t"<<"true"<<endl;
	}
	else{
		flag1 = false;
		cout<<"flag 1:\t"<<"false"<<endl;
	}
	if(SdX < 0.5 && SdY < 0.5 && SdTheta < 0.2){
		flag2 = true;
		cout<<"flag 2:\t"<<"true"<<endl;
	}
	else{
		flag2 = false;
		cout<<"flag 2:\t"<<"false"<<endl;
	}
}


int main()
{
	vector<double> particlesx {0, 0, 0};
	vector<double> particlesy {1, 1, 1};
	vector<double> particlest {2, 2, 2};

	double PoseEstime[3];
	double OldParticles[3];
	double Robot[3];
	double OldRobot[3];

	double SdX = sqrt(var(particlesx));
	double SdY = sqrt(var(particlesy));
	double SdTheta = sqrt(var(particlest));
	bool flag1, flag2;
	check_redistribution(PoseEstime, OldParticles, Robot, OldRobot, SdX, SdY, SdTheta, flag1, flag2);
}