#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <numeric>


using namespace std;

struct Obstacles {
    double posVertex[217][2][2]; //obstacleNumber line column
    double centre [217][2]; //obstacleNumber line
    double distDetect [217]; //obstacleNumber
};

/**functions to be done later**/
vector<vector<double>> particleGenerator(double,double,double,double,double,double,int,Obstacles);
double wrapAngle(double);
MesureAct()
double likelihood();
vector<double> selection(vector<double>,int);
bool checkRedistribution();



int main()
{
string fname = "obstacles.csv";

Obstacles Obstacles1;

	vector<vector<string>> obstacles;
	vector<string> row;
	string line, data;

	fstream file (fname, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
			stringstream str(line);

			while(getline(str, data, ','))
				row.push_back(data);
			obstacles.push_back(row);
		}
	}
	else
		cout<<"Could not open the file\n";

	for(int i=0;i<obstacles.size();i++)
	{
		Obstacles1.posVertex[i][0][0] = stod(obstacles[i][0]);
		Obstacles1.posVertex[i][0][1] = stod(obstacles[i][1]);
		Obstacles1.posVertex[i][1][0] = stod(obstacles[i][2]);
		Obstacles1.posVertex[i][1][1] = stod(obstacles[i][3]);
		Obstacles1.centre[i][0] = stod(obstacles[i][4]);
		Obstacles1.centre[i][1] = stod(obstacles[i][5]);
		Obstacles1.distDetect[i] = stod(obstacles[i][6]);
	}

	/**particle filter main loop**/
	int N = 200; //particle number (need to be an even number)

    int portee = 4; //portee of sensors



    vector<vector<double>> particles1; //particles in the first part of the map
    vector<vector<double>> particles2; //particles in the second part of the map
    vector<vector<double>> particles; //our particles (combination of both at first)

    particles1 = particleGenerator(26.5747,29.02,-0.269984,56,-M_PI,M_PI,N/2,Obstacles1);
    particles2 = particleGenerator(-5,26.5747,-0.269984,3,-M_PI,M_PI,N/2,Obstacles1);
    particles1.insert( particles1.end(), particles2.begin(), particles2.end() );
    particles = particles1;

    bool finTrajectoire = 0; // to be updagted in the loop

    vector<double> iNextGeneration(N,0);
    vector<double> poids(N,1/N); //vector of N element all equal to 1/N
    double poseEstimate[3]; //estimated position of the robot

    while(finTrajectoire == 0)
    {
        //control des particules:
        double dx = 0;//get x deplacement from odometry
        double dy = 0;//get y deplacement from odometry
        double dtheta = 0;//get theta deplacement from odometry
        for (int j=0;j<=N;j++)
        {
            particles[j][0] = particles[j][0]+dx; //new x for particle number j
            particles[j][1] = particles[j][1]+dy; //new y for particle number j
            particles[j][3] = wrapAngle(particles[j][2]+dtheta); //new theta for particle number j
        }

        rhoRobot = 0; //get robot measurement to be used in likelihood

        //start measurement for particles:

        for (int k=0;k<=N;k++)
        {
            rhoParticles = MesureAct();
            //calculate likelihood:
            poids[k] = likelihood(rhoRobot,rhoParticles);
        }

        //calculate the estimated position:
        sumPoids = accumulate(poids.begin(), poids.end(), 0);
        poseEstimate=[0;0;0];
        for (i=0;i<=N;i++)
        {
            poseEstimate[0] = poseEstimate[0]+particles[i][0]*poids[i];
            poseEstimate[1] = poseEstimate[1]+particles[i][1]*poids[i];
            poseEstimate[2] = poseEstimate[2]+particles[i][2]*poids[i];
        }
        poseEstimate[0] = poseEstimate[0]/sumPoids;
        poseEstimate[1] = poseEstimate[1]/sumPoids;
        poseEstimate[2] = poseEstimate[2]/sumPoids;

        //selection:
        iNextGeneration = selection(poids,N);



        //check for redistribution:
        bool flagRedistribution = checkRedistribution();
        ig (flagRedistribution)
        {
            //redistribute particels
                particles1 = particleGenerator(26.5747,29.02,-0.269984,56,-M_PI,M_PI,N/2,Obstacles1);
                particles2 = particleGenerator(-5,26.5747,-0.269984,3,-M_PI,M_PI,N/2,Obstacles1);
                particles1.insert( particles1.end(), particles2.begin(), particles2.end() );
                particles = particles1;
        }


    }




	return 0;
}
