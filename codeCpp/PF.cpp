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
euclid_position odoposP3D, Val_estime;

std::vector<struct polar_coordinate> Telemetries;
std::vector<struct polar_coordinate> OldLas1,OldLas2;
int Running=0;
double lsleep;

std::vector<double> Logs;
std::vector<double> LZA_LAS;
std::vector<struct timespec> LogTime;
std::vector<int> LogMode;

/**********Get MAP file***************/

  std::string mapfolder =PID_PATH("PF_Files");
  std::cout<<"Map folder : "<<mapfolder<<std::endl;
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
            break;
          case PF_RUNMODE:
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
            /******************PF HERE******************************/
            /*******************************************************/

            //// TO send a result:
            //Estimation.Set(Val_estime,TimeEstime);

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
