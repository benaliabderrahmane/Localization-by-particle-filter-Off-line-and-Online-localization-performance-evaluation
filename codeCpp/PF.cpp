#include "PF.h"
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <localization/lza.hpp>
#include <pid/rpath.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <string>


#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "Time_management.h"
#include "Formatings.h"
#include "Geometrie.h"


//With US
//void PFThread(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<laser_4m_record> &las1, SafeShared<laser_4m_record> &las2, SafeShared<std::vector<struct polar_coordinate>> &US, bool verbose, bool Logging,SafeShared<euclid_position> &Estimation){
//Without US
void PFThread(SafeShared<int> &Mode, SafeShared<euclid_position> &odom, SafeShared<laser_4m_record> &las1, SafeShared<laser_4m_record> &las2, bool verbose, bool Logging, SafeShared<euclid_position> &Estimation){
/*********INIT VARIABLES************/
int presentMode;
struct timespec StartTime, EndTime, Time, LauncheTime, TimeEstime;
struct timespec Las1Old, Las2Old, Las1Time, Las2Time;
struct timespec Todo, Toldodo;
struct timespec TimeLog;
laser_4m_record relevel1, relevel2;
euclid_position odoposP3D,odoposP3D_Old, Val_estime;

std::vector<struct polar_coordinate> Telemetries;
std::vector<struct polar_coordinate> OldLas1,OldLas2;
int Running=0;
double lsleep;

std::vector<double> Logs;
std::vector<double> LZA_LAS;
std::vector<struct timespec> LogTime;
std::vector<int> LogMode;

int N = 200; //particle number (need to be an even number)
int portee = 4; //portee of sensors

vector<vector<double>> particles1; //particles in the first part of the map
vector<vector<double>> particles2; //particles in the second part of the map
vector<vector<double>> particles; //our particles (combination of both at first)

particles1 = particleGenerator(26.5747,29.02,-0.269984,56,-M_PI,M_PI,N/2,Obstacles1);
particles2 = particleGenerator(-5,26.5747,-0.269984,3,-M_PI,M_PI,N/2,Obstacles1);
particles1.insert( particles1.end(), particles2.begin(), particles2.end());
particles = particles1;

vector<double> iNextGeneration(N,0);
vector<double> poids(N,1/N); //vector of N element all equal to 1/N
double poseEstimate[3]; //estimated position of the robot

/**********Get MAP file***************/
//  std::string mapfolder = PID_PATH("PF_Files");
//  std::cout<<"Map folder : "<<mapfolder<<std::endl;
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

	//transform obstacles
	vector <vector<double>> GrandObstacle;
	vector <double> distDetect;
	for (int i = 0; i<217;i++)
	{
		vector<double> temp;
		temp.push_back(Obstacles1.posVertex[i][0][0]);
		temp.push_back(Obstacles1.posVertex[i][0][1]);
		GrandObstacle.push_back(temp);
		vector<double> temp1;
		temp1.push_back(Obstacles1.posVertex[i][1][0]);
		temp1.push_back(Obstacles1.posVertex[i][1][1]);
		GrandObstacle.push_back(temp1);
		distDetect.push_back(Obstacles1.distDetect[i]);
	}
  //Load and read the map.****************************************************************

  /**********INIT TIMERS*****************/
  clock_gettime(CLOCK_REALTIME, &Time);
  StartTime=Time;
  Las1Old=Time;
  Las2Old=Time;
  Todo=Time;
  Toldodo=Time;
  LauncheTime=Time;
  TimeEstime=Time;

  /***************INIT LOG File*************/
    std::string filename = PID_PATH("+PANORAMA_Log/"+std::to_string(LauncheTime.tv_sec)+"/PF_LOG.txt");
    std::ofstream savefile;
    if(Logging){
      savefile.open(filename,std::ios::out);
    }

    /**********Ending INIT Thread*************/
    LauncheTime=Mode.Get(presentMode);
    Mode.Set(PF_IDLEMODE,LauncheTime);
    presentMode=PF_IDLEMODE;
    Las2Old=LauncheTime;
    Las1Old=LauncheTime;

    /***************MAIN LOOP*********************/
    Running=1;
    while(Running){
        Mode.Get(presentMode);
        clock_gettime(CLOCK_REALTIME, &StartTime);
        switch (presentMode) {
          case PF_IDLEMODE:
            break;
          case (PF_INITMODE || PF_RUNMODE):
            /*******************UPDATE SENSORS Reading (Odo, Las1+Las2)***********************/
            Todo=odom.Get(odoposP3D);
            Las2Time=las2.Get(relevel2);
            Las1Time=las1.Get(relevel1);
            if (isafter(Las2Time,Las2Old)){
              OldLas2=format_Telemetre(relevel2.data,relevel2.last_lenght,0.03,0,M_PI);
              Las2Old=Las2Time;
            }
            if (isafter(Las1Time,Las1Old)){
              OldLas1=format_Telemetre(relevel1.data,relevel1.last_lenght,0.03,0,0.7031*M_PI/180);
              Las1Old=Las1Time;
            }

            Telemetries.clear();
            if(isafter(Las2Old,LauncheTime) && !(enoughttimepassed(StartTime, Las2Old, 1000))){
              Telemetries.insert(Telemetries.end(),OldLas2.begin(),OldLas2.end());
            }
            if(isafter(Las1Old,LauncheTime) &&  !(enoughttimepassed(StartTime, Las1Old, 1000))){
              Telemetries.insert(Telemetries.end(),OldLas1.begin(),OldLas1.end());
            }

            /*******************************************************/
            /******************PF start*****************************/
            /*******************************************************/
            //control of particuls:
            double dx = odoposP3D.pos.x-odoposP3D_Old.pos.x;//get x deplacement from odometry
            double dy = odoposP3D.pos.y-odoposP3D_Old.pos.y;//get y deplacement from odometry
            double dtheta = odoposP3D.dir.theta-odoposP3D_Old.dir.theta;//get theta deplacement from odometry
            odoposP3D_Old = odoposP3D;
            for (int j=0;j<=N;j++)
            {
                particles[j][0] = particles[j][0]+dx; //new x for particle number j
                particles[j][1] = particles[j][1]+dy; //new y for particle number j
                particles[j][2] = wrapAngle(particles[j][2]+dtheta); //new theta for particle number j
            }

            vector<double> rhoRobot(3,0); //get robot measurement to be used in likelihood
            // use this one laser_4m_record L1, L2;

            //start measurement for particles:
            for (int k=0;k<=N;k++)
            {
                vector<double> rhoParticles(3,0);
                vector<double> ximp;
                vector<double> yimp;
                rhoParticles = MesureAct();
                vector<double> theta={-3.1416, -2.2440, -1.3464, -0.4488, 0.4488, 1.3464, 2.2440, 3.1416};
                Mesure_act(Portee,theta,obstacle,distDetect,particles[k][0],particles[k][0],particles[k][0],rhoParticles,ximp,yimp);
                //calculate likelihood:
                poids[k] = likelihood(rhoRobot,rhoParticles);
            }

            //calculate the estimated position:
            auto sumPoids = accumulate(poids.begin(), poids.end(), 0);
            double poseEstimate[3]={0,0,0};
            for (auto i=0;i<=N;i++)
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
            bool flag1 = false;
            bool flag2 = false;
            checkRedistribution(PoseEstime,OldParticles,Robot,OldRobot,SdX,SdY,SdTheta,&flag1,&flag2);
            bool flagRedistribution = flag1 && flag2;
            if (flagRedistribution)
            {
                //redistribute particels
                    particles1 = particleGenerator(26.5747,29.02,-0.269984,56,-M_PI,M_PI,N/2,Obstacles1);
                    particles2 = particleGenerator(-5,26.5747,-0.269984,3,-M_PI,M_PI,N/2,Obstacles1);
                    particles1.insert( particles1.end(), particles2.begin(), particles2.end() );
                    particles = particles1;
                    for (int k=0;k<=N;k++)
                    {
                        poids[k] = 1/N;
                    }
            }

            // check for convergance
            double wsd [3] = {0,0,0};

            double meanX = 0;
            double meanY = 0;
            double meanTheta = 0;

            for (int ii = 0; ii<N; ii++)
            {
                meanX += particles[ii][0];
                meanY += particles[ii][1];
                meanTheta += particles[ii][2];
            }

            meanX /= N;
            meanY /= N;
            meanTheta /= N;

            for (int ii = 0; ii<N; ii++)
            {
                wsd[0] += poids[ii]*(particles[ii][0]-meanX)*(particles[ii][0]-meanX); // we didn't use pow because particles[ii][0] is a vector not a double
                wsd[1] += poids[ii]*(particles[ii][1]-meanY)*(particles[ii][1]-meanY);
                wsd[2] += poids[ii]*(particles[ii][2]-meanTheta)*(particles[ii][2]-meanTheta);
            }
            auto sumWeights= accumulate(poids.begin(), poids.end(), 0);
            wsd[0] /= ((N-1)/N*sumWeights);
            wsd[1] /= ((N-1)/N*sumWeights);
            wsd[2] /= ((N-1)/N*sumWeights);

            bool flagConvergance = false;

            if (wsd[0]<2 && wsd[1]<2 && wsd[2]<0.5)
            {
                //particles filter converged we can send info if we want
                flagConvergance = true;
                // send data if run mode
            }

            if (presentMode == PF_RUNMODE && flagConvergance)
            {
              Estimation.Set(Val_estime,TimeEstime);
            }
            /*******************************************************/
            /******************PF end*******************************/
            /*******************************************************/
            break;
        }
        if(Logging){
          clock_gettime(CLOCK_REALTIME, &TimeLog);
          /***************Logging**************/
          savefile<<(StartTime.tv_nsec+(StartTime.tv_sec*1000000000))<<","<<presentMode;
          ///Anything
          savefile<<"\n";
        }

        //Computing for a 10Hz maximum frequency
        lsleep=delayusleep(StartTime, 100);
        if(lsleep<0){
          lsleep=0;
        }
        usleep(lsleep);
    }

    /****************Closing thread***************/
    if(Logging){
      savefile.close();
    }
}
