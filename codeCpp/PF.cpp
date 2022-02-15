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




#include "Time_management.h"
#include "Formatings.h"
#include "Geometrie.h"


using namespace std;

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

odoposP3D_Old.pos.x = 0;
odoposP3D_Old.pos.y = 0;
odoposP3D_Old.dir.theta = 0;
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

// for estimating laser readings of particles
vector<double> rhoParticles(3,0);
vector<double> ximp;
vector<double> yimp;
int NR = 32;//number of rays 

vector<double> theta=linspace(-M_PI, M_PI, NR+1);
theta.pop_back(); //delete last element as it's the same as the first one
for(int i=0; i!=theta.size(); i++)
{
  // round angles to the closest angle of the 1022 rayes
  theta[i] = round(theta[i]*(1022/(2*M_PI)))*(2*M_PI/1022);
}
vector<int> indexes; 

bool firstIteration = true; //bool if it's first iteration of the simulation
bool flagConvergance = false; // flag for convergance 
bool flagRedistribution = fase; // flag for redistribution 

euclid_position tempParticle; //temporary particle to be used in control part

vector<double> rhoRobot(NR,0); //get robot measurement to be used in likelihood

double wsd [3] = {0,0,0}; //weighted standard deviation for x, y and theta 
double meanX = 0; // mean of all particles x position
double meanY = 0; // mean of all particles y position
double meanTheta = 0; // mean of all particles theta position

double poseEstimate[3]={0,0,0}; //estimated position of the robot

double ssl=0; //right and left wheel displacement
double ssr=0;
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
          case PF_INITMODE:
          case PF_RUNMODE:
            /*******************UPDATE SENSORS Reading (Odo, Las1+Las2)***********************/
            Todo=odom.Get(odoposP3D);
            Las2Time=las2.Get(relevel2);
            Las1Time=las1.Get(relevel1);
            if (isafter(Las2Time,Las2Old))
            {
              OldLas2=format_Telemetre(relevel2.data,relevel2.last_lenght,0.03,0,M_PI);
              Las2Old=Las2Time;
            }
            if (isafter(Las1Time,Las1Old))
            {
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



            if (firstIteration)
            {
              //get indexes of Telemetries to use in simulation
              for (int i=0; i<NR; i++)
              {
                for(int j=0; j<Telemetries.size();j++)
                {
                  if (abs(theta[i]-Telemetries[j].dir.theta)<0.001)
                    {
                      indexes.push_back(j);
                    }
                }
              }
              firstIteration = false;
            }

            /*******************************************************/
            /******************PF start*****************************/
            /*******************************************************/
            //control of particles:
            Recomput_Weeldist(odoposP3D_Old,odoposP3D,ssl,ssr);
            odoposP3D_Old = odoposP3D;
            for (int j=0;j<=N;j++)
            {
              tempParticle.pos.x = particles[j][0];
              tempParticle.pos.y = particles[j][1];
              tempParticle.dir.theta = particles[j][2];
              SimuOdo(tempParticle,tempParticle,ssl,ssr);
              particles[j][0]= tempParticle.pos.x;
              particles[j][1]= tempParticle.pos.y;
              particles[j][2]= tempParticle.dir.theta;
            }
            
            for(int j=0; j<indexes.size();j++)
            {
              //get robot measurement to be used in likelihood
              rhoRobot[j] = Telemetries[indexes[j]].dist;
            }
            

            //start measurement for particles:
            for (int k=0;k<=N;k++)
            {
              Mesure_act(Portee,theta,obstacle,distDetect,particles[k][0],particles[k][0],particles[k][0],rhoParticles,ximp,yimp);
              //calculate likelihood:
              poids[k] = likelihood(rhoRobot,rhoParticles);
            }

            //calculate the estimated position:
            auto sumPoids = accumulate(poids.begin(), poids.end(), 0);
            poseEstimate[0] = 0;
            poseEstimate[1] = 0;
            poseEstimate[2] = 0;

            for (int i=0;i<=N;i++)
            {
                poseEstimate[0] = poseEstimate[0]+particles[i][0]*poids[i];
                poseEstimate[1] = poseEstimate[1]+particles[i][1]*poids[i];
                poseEstimate[2] = poseEstimate[2]+particles[i][2]*poids[i];
            }
            poseEstimate[0] = poseEstimate[0]/sumPoids;
            poseEstimate[1] = poseEstimate[1]/sumPoids;
            poseEstimate[2] = poseEstimate[2]/sumPoids;

            Val_estime.pos.x = poseEstimate[0];
            Val_estime.pos.y = poseEstimate[1];
            Val_estime.dir.theta = poseEstimate[2];
            clock_gettime(CLOCK_REALTIME, &TimeEstime);

            //selection:
            iNextGeneration = selection(poids,N);
            //get the new particles 
            particles = testInext(iNextGeneration, Particles, N, Obstacles1);

            // check for convergance
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
                wsd[0] += poids[ii]*(particles[ii][0]-meanX)*(particles[ii][0]-meanX); // we didn't use pow because particles[ii][0] is a vector of double not a double
                wsd[1] += poids[ii]*(particles[ii][1]-meanY)*(particles[ii][1]-meanY);
                wsd[2] += poids[ii]*(particles[ii][2]-meanTheta)*(particles[ii][2]-meanTheta);
            }

            wsd[0] = sqrt(wsd[0]/((N-1)/N*sumPoids));
            wsd[1] = sqrt(wsd[1]/((N-1)/N*sumPoids));
            wsd[2] = sqrt(wsd[2]/((N-1)/N*sumPoids));

            if (wsd[0]<2 && wsd[1]<2 && wsd[2]<0.5)
            {
                //particles filter converged we can send info if we want
                flagConvergance = true;
            }
            else
            {
                flagConvergance = false;
            }

            //check for redistribution:
            if (wsd[0]<0.3 && wsd[1]<0.3)
            {
              flagRedistribution = true;
            }
            else
            {
              flagRedistribution = false;
            }

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

            if (presentMode == PF_RUNMODE && flagConvergance)
            {
              // send data if run mode and if we converged
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
          savefile << (StartTime.tv_nsec+(StartTime.tv_sec*1000000000)) << "," << presentMode << ",";
          savefile << (TimeLog.tv_nsec+(TimeLog.tv_sec*1000000000)) << ",";
          savefile << (StartTime.tv_nsec+(StartTime.tv_sec*1000000000)) << ",";
          savefile << (Las1Old.tv_nsec+(Las1Old.tv_sec*1000000000)) << ",";
          savefile << (Las2Old.tv_nsec+(Las2Old.tv_sec*1000000000)) << ",";
          savefile << (Todo.tv_nsec+(Todo.tv_sec*1000000000)) << ",";
          savefile << (Toldodo.tv_nsec+(Toldodo.tv_sec*1000000000)) << ",";
          savefile << (LauncheTime.tv_nsec+(LauncheTime.tv_sec*1000000000)) << ",";
          savefile << (TimeEstime.tv_nsec+(TimeEstime.tv_sec*1000000000)) << ",";
          savefile << odom.pos.x << ",";
          savefile << odom.pos.y << ",";
          savefile << odom.pos.theta << ",";
          savefile << poseEstimate[0] << ",";
          savefile << poseEstimate[1] << ",";
          savefile << poseEstimate[2] << ",";
          savefile << wsd[0] << ",";
          savefile << wsd[1] << ",";
          savefile << wsd[2] << ",";
          savefile << N << ",";
          savefile << NR << ",";
          savefile << flagConvergance << ",";
          savefile << flagRedistribution << ",";

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
