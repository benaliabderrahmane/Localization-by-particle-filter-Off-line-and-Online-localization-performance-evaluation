
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <limits>
#include <thread>
#include <mutex>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>


#include "P3DX_Module.h"
#include "Time_management.h"
#include "Formatings.h"
#include <datas/location_data.hpp>
#include <Tools/Safe_shared.hpp>
#include <pfoa/path_finding.hpp>
#include <pid/rpath.h>
#include <pioneer_robot/objp3dx.hpp>
#include <yocto/wattmeter.h>
#include <LaptopBattery/battery.h>

#include "Geometrie.h"

/*
TODO: Preparig for repathing on the flight
TODO: Implementing turning
*/


void ComputCeentering();
void testfunction(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<euclid_position> &Recalage, SafeShared<laser_4m_record> &releveLas1, SafeShared<laser_4m_record> &releveLas2, SafeShared<double> &vit, bool verbose, bool Logging, pioneer_robot::PioneerP3DX &rob,  pfoa::waypoint *Path);

euclid_position Compute_repose(euclid_position RelocPose, P3D_Consignes_Moteurs consignes, double dtus);
polar_coordinate Rearange_Impact(polar_coordinate impact);

void PioneerThread(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<euclid_position> &Recalage, SafeShared<laser_4m_record> &releveLas1, SafeShared<laser_4m_record> &releveLas2, SafeShared<double> &vit, bool verbose, bool Logging, SafeShared<std::vector<pfoa::waypoint>> &Path){//, SafeShared<int> &LengthPath){
  struct timespec TimeMode, TimeModeOld;
  struct timespec TimeLoc, TimeLocOld;
  struct timespec TimeReLocOld, TimeReLoc;
  struct timespec Time;
  struct timespec TimeLas1;
  struct timespec TimeLas2;
  struct timespec TimePath,TimePathOld;
  struct timespec TimeStart,TimeWat,TimeLog;
  euclid_position RelocPose, Sortie_Odo, OriginalRelocPose;
  laser_4m_record L1, L2;
  std::vector<struct polar_coordinate> Telemetries;
  std::vector<struct polar_coordinate> Tmp_telemetries;
  std::vector<double> Logs;
  std::vector<struct timespec> LogTimeOdo;
  std::vector<int> LogMode;
  int logline=1;
  int lsleep;


  int PresentMode;
  TimeMode = Mode.Get(PresentMode);
  pioneer_robot::PioneerP3DX rob ("/dev/ttyUSB0");
  if (!rob.is_Setup()){
    Mode.Set(P3D_HALTMOD,TimeMode);
    return;
  }
  printf("Robot ok\n");

  std::string suivitdechemin = PID_PATH("+PANORAMA_Log/suivitdechemin.txt");
  pfoa::PFOA folowpath ((char*)suivitdechemin.c_str(),false);
  std::cout << "P3DX INIT - PFOA created" << '\n';
  double Entreroue=ENTRE_ROUE;//0.328239;//0.3564514342;//0.33;//0.1905*2;
  double Rayonroue=0.095;
  double lwvel, rwvel;
  double vmax, v;
  vit.Get(v);//0.4;
  //waypoint *Path = NULL;
  pfoa::waypoint *MyPath = NULL;
  std::vector<pfoa::waypoint> Pathtmp;
  int Nwaypoint;
  TimePathOld=Path.Get(Pathtmp);
  Nwaypoint=Pathtmp.size();
  //TimePathOld=LengthPath.Get(Nwaypoint);
  MyPath=(pfoa::waypoint *)malloc(sizeof(pfoa::waypoint)*360);
  for(int p=0; p<Nwaypoint; ++p){
    MyPath[p]=Pathtmp[p];
  }
  double ds, dtet, dsangle;
  int Reloc_flag=0;

  /////////////Beeps planning
  char Beeplength=2;
  char Beep1[2];
  Beep1[0]=25;
  Beep1[1]=75;
  char Beep2[2];
  Beep2[0]=50;
  Beep2[1]=81;
  double StartDelay_s=1;//27;//
  int BeepState=-1;
  ////////////////////////////////

  char BeepVictorylength=10;
  char BeepVictory[10];
  BeepVictory[0]=(char)6;
  BeepVictory[1]=(char)60;
  BeepVictory[2]=(char)1;
  BeepVictory[3]=(char)0;
  BeepVictory[4]=(char)6;
  BeepVictory[5]=(char)60;
  BeepVictory[6]=(char)1;
  BeepVictory[7]=(char)0;
  BeepVictory[8]=(char)20;
  BeepVictory[9]=(char)75;
/*
  char BeepVictorylength=40;
  char BeepVictory[40];
  BeepVictory[0]=(char)6;
  BeepVictory[1]=(char)60;
  BeepVictory[2]=(char)1;
  BeepVictory[3]=(char)0;
  BeepVictory[4]=(char)6;
  BeepVictory[5]=(char)60;
  BeepVictory[6]=(char)1;
  BeepVictory[7]=(char)0;
  BeepVictory[8]=(char)6;
  BeepVictory[9]=(char)60;
  BeepVictory[10]=(char)1;
  BeepVictory[11]=(char)0;
  BeepVictory[12]=(char)9;
  BeepVictory[13]=(char)60;
  BeepVictory[14]=(char)1;
  BeepVictory[15]=(char)0;
  BeepVictory[16]=(char)9;
  BeepVictory[17]=(char)56;
  BeepVictory[18]=(char)1;
  BeepVictory[19]=(char)0;
  BeepVictory[20]=(char)12;
  BeepVictory[21]=(char)56;
  BeepVictory[22]=(char)1;
  BeepVictory[23]=(char)0;
  BeepVictory[24]=(char)6;
  BeepVictory[25]=(char)60;
  BeepVictory[26]=(char)6;
  BeepVictory[27]=(char)0;
  BeepVictory[28]=(char)6;
  BeepVictory[29]=(char)58;
  BeepVictory[30]=(char)1;
  BeepVictory[31]=(char)0;
  BeepVictory[32]=(char)25;
  BeepVictory[33]=(char)60;
  BeepVictory[34]=(char)1;
  BeepVictory[35]=(char)0;
  BeepVictory[36]=(char)1;
  BeepVictory[37]=(char)0;
  BeepVictory[38]=(char)1;
  BeepVictory[39]=(char)0;
  ////////////////////////////////
/**/
  double Crot=0;

  P3D_Consignes_Moteurs consignes;
  polar_coordinate impact={{0,0,1},6};
  polar_coordinate impactOA={{0,0,1},6};
  P3D_Robot_State R_state,Oldstate;
  euclid_position CurentRob_Pos, LastRob_Pos;
  double DistRG, DistRD;
  uint8_t bumps[10]={0,0,0,0,0,0,0,0,0,0};

  folowpath.Set_OA_Type(0);//Pas d'evitement
  folowpath.Set_Vmax(v);
  //TODO:Gerer le rayon smz dynamiquement.
  folowpath.safeDistancetoImpact=0.7;//0.6 de base légèrement augmenté pour V=0.56
  vmax=v;
  folowpath.Set_odom_param(Entreroue,Rayonroue*2,7);
  folowpath.activation_Evitement_Obstacle=0;
//  folowpath.k_rabbit=1.8;//1 Gestion du depacement lièvre 1-10 +élev=moins de dépassement.

  double dtus=-1;
  struct timespec LauncheTime;
  int encoleft,encoright;
  float offy=MyPath[0].y;
  float offx=MyPath[0].x;
  folowpath.Set_Path(MyPath,Nwaypoint);
  std::cout << "P3DX INIT - PFOA Set uped" << '\n';

  Recalage.Get(RelocPose);

  rob.relocate(RelocPose.pos.x,RelocPose.pos.y,RelocPose.dir.theta);/////////////////////////////////////////////////////////////////
//  rob.relocate(0,1.1,0);
  rob.start_log();
  rob.set_Motor_Power(true);
  //bob.set_Drift_Factor(drift);
//  rob.set_Tick_Correction(0.999,1.0008);// empirique~
//  rob.set_Tick_Correction(0.9995,1.00095);// empirique~
//  rob.set_Tick_Correction(0.9995,1.00088);// empirique~
  rob.set_Tick_Correction(0.999,1.00095);// empirique~
//  rob.set_Tick_Correction(0.9999628924,1.00003710761);//
  rob.set_Odom_Carac(Rayonroue*2,Rayonroue*2, Entreroue);
  rob.set_Encoder_Stream_Status(true);
  rob.set_Wheel_Velocity(0,0);
  std::cout << "P3DX INIT - Robot Init" << '\n';
  double USs[16];

  std::thread t2([&rob](){
  while (rob.is_Setup()) {
    rob.update_Status();
  }
  });

  std::cout << "P3DX INIT - Update status go" << '\n';
  //////////////////////////////////////////////////////////////////////////////
  TimeMode = Mode.Get(PresentMode);
  TimeStart=TimeMode;
	clock_gettime(CLOCK_REALTIME, &Time);
	TimeMode=Time;
	TimeModeOld=Time;
  TimeLoc=Time;
	TimeLocOld=Time;
  TimeReLocOld=Time;
	TimeReLoc=Time;
  TimeLas1=Time;
  TimeLas2=Time;

  rob.get_Encodeur_Pos(R_state.x_pos,R_state.y_pos,R_state.theta_pos);
  while(R_state.x_pos!=RelocPose.pos.x && R_state.y_pos!=RelocPose.pos.y && R_state.theta_pos!=RelocPose.dir.theta){
    rob.relocate(RelocPose.pos.x,RelocPose.pos.y,RelocPose.dir.theta);/////////////////////////////////////////////////////////////////
    usleep(5000);
    rob.get_Encodeur_Pos(R_state.x_pos,R_state.y_pos,R_state.theta_pos);
  }

  std::cout << "P3DX INIT - Pos Init Seted" << '\n';

  /***************************************************************************/
  double Yoctopower, Yoctoenergy;
  long Yoctotime;
  yocto::YoctoWattmeter wattmetre;
  bool connect = wattmetre.connect_any_wattmetre();
  if(!connect){
    std::cout<<"Init P3D ECHEC : Echec de connexion au wattmetre"<<std::endl;
    return;
  }
  std::cout << "P3DX INIT - YoctoWattmeter Init" << '\n';
  /***************************************************************************/
  float InstantlaptopConsumption=0;
  battery batlaptop;
  /***************************************************************************/

  FILE * savefile = NULL;
  if(Logging){
    std::string filename = PID_PATH("+PANORAMA_Log/"+std::to_string(TimeStart.tv_sec)+"/P3DX_LOG.txt");
    savefile=fopen(filename.c_str(),"w");
    if (savefile==NULL)
    {
      printf("Echec de l'ouverture du fichier de sauvegarde : %s\n",filename.c_str());
      return;
    }
    std::cout<<"P3DX:saving in : "<<filename<<std::endl;
  }

  /*TODO : remove after test
  rob.Send_Beeps(BeepVictorylength,BeepVictory);
  sleep(5);
  /**/
  PresentMode=P3D_IDLEMODE;
  Mode.Set(PresentMode,TimeMode);
  LauncheTime=Time;
  int folowing=0;
  std::cout << "P3DX INIT - Ready!" << '\n';

/*Testé et validé.
    OriginalRelocPose.pos.y=1;
    OriginalRelocPose.pos.x=8;
    OriginalRelocPose.dir.theta=0.1;
    consignes.cmde_vitesse_RG=0.1;
    consignes.cmde_vitesse_RD=0.1;
    clock_gettime(CLOCK_REALTIME, &Time);
    sleep(1);
    clock_gettime(CLOCK_REALTIME, &TimeReLoc);
    dtus=Subtimeus(TimeReLoc,Time);
    OriginalRelocPose=Compute_repose(OriginalRelocPose, consignes, dtus);//Recalcule du deplacement depuis la mesure a partir de la dernière commande envoyé.
    std::cout << "Compute_repose test res: X="<<OriginalRelocPose.pos.x<<" - Y="<< OriginalRelocPose.pos.y<<" - Theta="<< OriginalRelocPose.dir.theta<< "DT(us)="<< dtus << '\n';
    std::cout << "///////////////////////////////////////////////////////////////////////////////////////" << '\n';
*/

  //YOLO: tmp!!
  //rob.set_Us_On_Off(false);
  while (1)
	{
    DistRG=0;
    DistRD=0;
    TimeMode = Mode.Get(PresentMode);
    if(isafter(TimeMode,TimeModeOld)){
      rob.Send_Beeps(Beeplength,Beep1);
      usleep(3000);
    }
    TimeModeOld=TimeMode;
    //Reset flags
    if(verbose){
      std::cout << "P3DX Mode : "<< PresentMode << '\n';
    }
    Reloc_flag=0;//De base il n'y a pas de relocalisation.
    if(PresentMode==P3D_HALTMOD){
      std::cout << "P3DX Halt mode SET!" << '\n';
      break;
    }
    //
    if(PresentMode<0){
      std::cout << "P3DX Bad present mode : folowing out!" << '\n';
      folowing=1;
      break;
    }
    clock_gettime(CLOCK_REALTIME, &Time);
    TimePath=Path.Get(Pathtmp);
    if(isafter(TimePath,TimePathOld)){
      Nwaypoint=Pathtmp.size();
      std::cout << "New path : " << '\n';
      for(int p=0; p<Nwaypoint; ++p){
        std::cout << " {" <<Pathtmp[p].x<<":"<<Pathtmp[p].y<<":"<<Pathtmp[p].D<<"} ";
        MyPath[p]=Pathtmp[p];
      }
      std::cout << "New path end NWaypoint : " << Nwaypoint << '\n';

//      folowpath.~PFOA();
//      pfoa::PFOA folowpath ((char*)suivitdechemin.c_str(),false);
      folowpath.Set_OA_Type(0);//Pas d'evitement
      folowpath.Set_Vmax(v);
      folowpath.safeDistancetoImpact=0.75;//0.6 de base légèrement augmenté pour V=0.56
      folowpath.Set_odom_param(Entreroue,Rayonroue*2,7);
      folowpath.activation_Evitement_Obstacle=0;
      folowpath.Set_Path(MyPath,Nwaypoint);
      folowing=0;
      TimePathOld=TimePath;
    }

    if(PresentMode==P3D_TESTING){/*
      TimeReLoc = Recalage.Get(RelocPose);
      if(isafter(TimeReLoc,TimeReLocOld)){//new localisation estimation have been receave : local recomputing.
        //Initialisarion de R_state
        //Peut faire mieux.
        R_state.x_pos=RelocPose.pos.x;
        R_state.y_pos=RelocPose.pos.y;
        R_state.theta_pos=RelocPose.dir.theta;
        TimeReLocOld=TimeReLoc;//Remplacement du temps de relocalisation
        Reloc_flag=1;//Flag de relocalisation
        rob.relocate(R_state.x_pos,R_state.y_pos,R_state.theta_pos);//Recalage pioneer.
      }else{
        rob.get_Encodeur_Pos(R_state.x_pos,R_state.y_pos,R_state.theta_pos);
      }
      TimePath=Path.Get(Pathtmp);
      Nwaypoint=Pathtmp.size();
      if(isafter(TimePath,TimePathOld)){
        for(int p=0; p<Nwaypoint; ++p){
          MyPath[p]=Pathtmp[p];
        }
        TimePathOld=TimePath;
      }

      testfunction(Mode, Odopos, Recalage, releveLas1, releveLas2, vit, verbose, Logging, rob, MyPath);

      Sortie_Odo.pos.x=R_state.x_pos;
      Sortie_Odo.pos.y=R_state.y_pos;
      Sortie_Odo.pos.z=0;
      Sortie_Odo.dir.theta=R_state.theta_pos;
      Sortie_Odo.dir.phi=0;
      Sortie_Odo.dir.psy=0;
      Oldstate=R_state;
      clock_gettime(CLOCK_REALTIME, &TimeLoc);
      Odopos.Set(Sortie_Odo,TimeLoc);

      if(Logging){
        fprintf(savefile,"%ld",(TimeLoc.tv_nsec+(TimeLoc.tv_sec*1000000000)));
        fprintf(savefile,",%d",PresentMode);
        fprintf(savefile,",%f",Sortie_Odo.pos.x);
        fprintf(savefile,",%f",Sortie_Odo.pos.y);
        fprintf(savefile,",%f",Sortie_Odo.dir.theta);
        fprintf(savefile,",%f",-1.0);
        fprintf(savefile,",%f",1.0);
        fprintf(savefile,",%f",-1.0);
        fprintf(savefile,",%f",999.0);
        fprintf(savefile,",%d",-8);
        fprintf(savefile,"\n");
      }
      continue;*/
    }

    if(PresentMode==P3D_FREEZING){
      std::cout << "P3D: Freezed please input 42 to start again" << '\n';
      std::cin >> BeepState;
      if(BeepState==42){
        std::cout << "P3D: Unfreez input accepted mission continue in 10s." << '\n';
        sleep(10);
        //On feinte l'évitement d'obstacle : Kidnapping => no DT detecté
        R_state.u=0;
        folowpath.Traitement_PFS(R_state,&impact,&consignes);
        PresentMode=P3D_IDLEMODE;
        continue;
      }
      std::cout << "P3D: Invalid unfreez value : please input 42 to start again." << '\n';
    }
    if(PresentMode==P3D_STARTMODE){
      Logging=false;//On ne loge pas pendant le décompte.
//      std::cout << "P3D: STARTING Mode!!" << '\n';
      switch (BeepState) {
        case -1:
          BeepState=0;
          std::cout << "P3D: STARTING Mode!! Beep1" << '\n';
          rob.Send_Beeps(Beeplength,Beep1);
          break;
        case 0:
          if(enoughttimepassed(Time,TimeMode,(StartDelay_s/2)*1000)){
            BeepState=1;
            std::cout << "P3D: STARTING Mode!! Beep2" << '\n';
            rob.Send_Beeps(Beeplength,Beep1);
          }
          break;
        case 1:
          if(enoughttimepassed(Time,TimeMode,(StartDelay_s-2.5)*1000)){
            BeepState=2;
            rob.Send_Beeps(Beeplength,Beep1);
            std::cout << "P3D: STARTING Mode!! Beep3" << '\n';
          }
          break;
        case 2:
          if(enoughttimepassed(Time,TimeMode,(StartDelay_s-1.5)*1000)){
            BeepState=3;
            rob.Send_Beeps(Beeplength,Beep1);
            std::cout << "P3D: STARTING Mode!! Beep4" << '\n';
          }
          break;
        case 3:
          if(enoughttimepassed(Time,TimeMode,(StartDelay_s-0.5)*1000)){
            BeepState=4;
            rob.Send_Beeps(Beeplength,Beep2);
            std::cout << "P3D: STARTING Mode!! Beep5" << '\n';
          }
          break;
        default:
          break;
      }
//      std::cout << "P3D: STARTING Mode!! Time passed?" << '\n';
      if(enoughttimepassed(Time,TimeMode,(StartDelay_s)*1000)){
        Mode.Set(P3D_IDLEMODE,Time);
        Logging=true;//Fin du délais on commence a loguer
        wattmetre.reset_wattmetre();
        std::cout << "P3D: STARTING Mode!! out" << '\n';
      }
//      break;
    }
    rwvel=0;
    lwvel=0;
    if(PresentMode!=P3D_IDLEMODE && PresentMode!=P3D_STARTMODE)
    {//Robot is mooving.
      vit.Get(v);//0.4;
      if(v!=vmax){
        vmax=v;
        folowpath.Set_Vmax(v);
        //TODO:Gerer le rayon smz dynamiquement.
        folowpath.safeDistancetoImpact=0.65;//0.6 de base légèrement augmenté pour V=0.56
      }
      if(verbose){
        std::cout<<"P3DX_T---------------------------------------------------------"<<std::endl;
      }
      Telemetries.clear();
      TimeReLoc = Recalage.Get(OriginalRelocPose);
      while(OriginalRelocPose.dir.theta>M_PI){
        OriginalRelocPose.dir.theta=OriginalRelocPose.dir.theta-2*M_PI;
      }
      while(OriginalRelocPose.dir.theta<(-1*M_PI)){
        OriginalRelocPose.dir.theta=OriginalRelocPose.dir.theta+2*M_PI;
      }
      RelocPose=OriginalRelocPose;
      folowpath.Set_OA_Type(1);//Evitement
      folowpath.activation_Evitement_Obstacle=1;

      rob.get_Encodeur_Pos(R_state.x_pos,R_state.y_pos,R_state.theta_pos);

      CurentRob_Pos.pos.x=R_state.x_pos;
      CurentRob_Pos.pos.y=R_state.y_pos;
      CurentRob_Pos.dir.theta=R_state.theta_pos;
      LastRob_Pos.pos.x=Oldstate.x_pos;
      LastRob_Pos.pos.y=Oldstate.y_pos;
      LastRob_Pos.dir.theta=Oldstate.theta_pos;
      Recomput_Weeldist(LastRob_Pos,CurentRob_Pos, DistRG, DistRD);


      if(isafter(TimeReLoc,TimeReLocOld)){//new localisation estimation have been receave : local recomputing.
//        dtus=Subtimeus(TimeReLoc,Time);
        dtus=Subtimeus(Time, TimeReLoc);
        //Initialisarion de R_state
        //Peut faire mieux.
        RelocPose=Compute_repose(RelocPose, consignes, dtus);//Recalcule du deplacement depuis la mesure a partir de la dernière commande envoyé.
        R_state.x_pos=RelocPose.pos.x;
        R_state.y_pos=RelocPose.pos.y;
        R_state.theta_pos=RelocPose.dir.theta;
        TimeReLocOld=TimeReLoc;//Remplacement du temps de relocalisation
        Reloc_flag=1;//Flag de relocalisation
        rob.relocate(R_state.x_pos,R_state.y_pos,R_state.theta_pos);//Recalage pioneer.
        if(verbose){
          std::cout<<"RELOC PIONEER!!!!!!!!!!!!!!!!!!!"<<std::endl;
          std::cout << "Recalage Original : X=" << OriginalRelocPose.pos.x << " : Y=" << OriginalRelocPose.pos.y << " : Theta=" << OriginalRelocPose.dir.theta << '\n';
          std::cout << "Recalage : X=" << RelocPose.pos.x << " : Y=" << RelocPose.pos.y << " : Theta=" << RelocPose.dir.theta << '\n';
          std::cout << "Recalage : dtus=" << dtus <<std::endl;
        }
      }
      clock_gettime(CLOCK_REALTIME, &TimeLoc);//Recuperation date a la quelle la localisation a ete recuperee ou adoptee.
      rob.get_Velocity(rwvel,lwvel);
      R_state.u=(Rayonroue/2)*(rwvel+lwvel);

      /*UPdating US for others*/
      rob.get_US_Dist(USs);
      Telemetries=format_US(USs);
      //TODO : update new shared variable
      Telemetries.clear();
      /**/
      switch (PresentMode)
      {
      case P3D_SMZUSMODE:
        rob.get_US_Dist(USs);
        Telemetries=format_US(USs);
        break;
      case P3D_SMZLASMODE:
        TimeLas1 = releveLas1.Get(L1);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0.7031*M_PI/180);
        break;
      case P3D_SMZLAS2MODE:
        TimeLas1 = releveLas1.Get(L1);
        TimeLas2 = releveLas2.Get(L2);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0.7031*M_PI/180);
        Tmp_telemetries=format_Telemetre(L2.data,L2.last_lenght,0.03,0,M_PI);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        break;
      case P3D_SMZLASUSMODE:
        TimeLas1 = releveLas1.Get(L1);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0.7031*M_PI/180);
        rob.get_US_Dist(USs);
        Tmp_telemetries=format_US(USs);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        break;
      case P3D_SMZLAS2USMODE:
        TimeLas1 = releveLas1.Get(L1);
        TimeLas2 = releveLas2.Get(L2);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0.7031*M_PI/180);
        Tmp_telemetries=format_Telemetre(L2.data,L2.last_lenght,0.03,0,M_PI);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        rob.get_US_Dist(USs);
        Tmp_telemetries=format_US(USs);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        break;
      case P3D_CENTERUSMODE:
        rob.get_US_Dist(USs);
        Telemetries=format_US(USs);
        break;
      case P3D_CENTERLASMODE:
        TimeLas1 = releveLas1.Get(L1);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0);
        break;
      case P3D_CENTERLAS2MODE:
        TimeLas1 = releveLas1.Get(L1);
        TimeLas2 = releveLas2.Get(L2);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0);
        Tmp_telemetries=format_Telemetre(L2.data,L2.last_lenght,0.03,0,M_PI);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        break;
      case P3D_CENTERLASUSMODE:
        TimeLas1 = releveLas1.Get(L1);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0);
        rob.get_US_Dist(USs);
        Tmp_telemetries=format_US(USs);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        break;
      case P3D_CENTERLAS2USMODE:
        TimeLas1 = releveLas1.Get(L1);
        TimeLas2 = releveLas2.Get(L2);
        Telemetries=format_Telemetre(L1.data,L1.last_lenght,0.03,0,0);
        Tmp_telemetries=format_Telemetre(L2.data,L2.last_lenght,0.03,0,M_PI);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        rob.get_US_Dist(USs);
        Tmp_telemetries=format_US(USs);
        Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());
        break;
      case P3D_NOOAMODE:
        folowpath.Set_OA_Type(0);//Pas d'evitement
        folowpath.activation_Evitement_Obstacle=0;
        break;
      default:
        break;
      }

      if(Reloc_flag==1){//On vien d'envoyer une commande de recalage au robot, il faut attendre un minimum avant de r'envoyer une commande.
        lsleep=delayusleep(TimeLoc, 5);
        if(lsleep<0){
          lsleep=0;
        }
        usleep(lsleep);
//        usleep(max(delayusleep(TimeLoc, 5),0));
      }
      rob.get_Bumpers_Values(bumps);
      for (int i=0; i<10; i++){
        if(bumps[i]==1){
          folowing=1;
          std::cout<<"ABBORD !!!!!!  -----  P3D:Bumpping Bumper N°"<< i <<" !!!!!"<<std::endl;
          consignes.cmde_vitesse_RG=0;
          consignes.cmde_vitesse_RD=0;
          PresentMode=P3D_BUMPER_ERRORMODE;
          Mode.Set(P3D_BUMPER_ERRORMODE,Time);
          break;
        }
      }

      if(PresentMode==P3D_SMZUSMODE || PresentMode==P3D_SMZLASMODE || PresentMode==P3D_SMZLAS2MODE || PresentMode==P3D_SMZLASUSMODE || PresentMode==P3D_SMZLAS2USMODE || PresentMode==P3D_NOOAMODE){
        impact.dist=90000;
        for(int i=0; i<Telemetries.size(); i++){
          if(Telemetries[i].dist<impact.dist && Telemetries[i].dist>0.05){
            impact=Telemetries[i];
          }
        }
        if(verbose){
          std::cout<<"== Impact dist : "<< impact.dist << " Angle : "<< impact.dir.theta <<" =="<<std::endl;
          std::cout<<"Rstate X:"<<R_state.x_pos<<" - Y:"<<R_state.y_pos<<" - Theta:"<<R_state.theta_pos<<std::endl;
        }
        //Patchwork pour essayer d'améliorer l'évitement d'obstacle.
        impactOA=impact;
        impact=Rearange_Impact(impact);
        folowing=folowpath.Traitement_PFS(R_state,&impact,&consignes);
        impact=impactOA;
        if(verbose){
          std::cout<<"== Consignes left: "<< consignes.cmde_vitesse_RG << " - Right : "<< consignes.cmde_vitesse_RD <<" =="<<std::endl;
          std::cout<<"Rstate X:"<<R_state.x_pos<<" - Y:"<<R_state.y_pos<<" - Theta:"<<R_state.theta_pos<<std::endl;
        }
        if(folowing==1){
          Mode.Set(P3D_ENDINGMODE,Time);
        }
        if(verbose){
          std::cout<<"folowing :"<<folowing<<std::endl;
        }
      }else{
        if (PresentMode==P3D_TURNING){
          v;//Quand on tourne on se sert de Vmax pour indiquer l'orientation souhaité.
          while(v>M_PI){
            v=v-2*M_PI;
          }
          while(v<-1*M_PI){
            v=v+2*M_PI;
          }
          Crot=((v-R_state.theta_pos)/fabs(R_state.theta_pos-v))*0.05;
          if(fabs(R_state.theta_pos-v)>M_PI){
            consignes.cmde_vitesse_RG=Crot;
            consignes.cmde_vitesse_RD=-1*Crot;
          }else{
            consignes.cmde_vitesse_RG=-1*Crot;
            consignes.cmde_vitesse_RD=Crot;
          }
          std::cout<<"Diforient = " << fabs(R_state.theta_pos-v) << std::flush;
          if(fabs(R_state.theta_pos-v)<0.2){
            consignes.cmde_vitesse_RG=0;
            consignes.cmde_vitesse_RD=0;
            Mode.Set(P3D_IDLEMODE,Time);
          }
        }else{
          if(PresentMode==P3D_CENTERUSMODE || PresentMode==P3D_CENTERLASMODE || PresentMode==P3D_CENTERLAS2MODE || PresentMode==P3D_CENTERLASUSMODE ||  PresentMode==P3D_CENTERLAS2USMODE){
            std::cout<<"Error : P3DX Using Non implemented Centerring mode."<<std::endl;
            consignes.cmde_vitesse_RG=0;
            consignes.cmde_vitesse_RD=0;
            PresentMode=P3D_INVALIDMOD_ERRORMODE;
            Mode.Set(P3D_INVALIDMOD_ERRORMODE,Time);
          }else{
            std::cout<<"Error : P3DX Using Unknown mode."<<std::endl;
            consignes.cmde_vitesse_RG=0;
            consignes.cmde_vitesse_RD=0;
            PresentMode=P3D_INVALIDMOD_ERRORMODE;
            Mode.Set(P3D_INVALIDMOD_ERRORMODE,Time);
          }
        }
      }
      rob.set_Wheel_Velocity(consignes.cmde_vitesse_RG,consignes.cmde_vitesse_RD);
      // Set the exite.
      Sortie_Odo.pos.x=R_state.x_pos;
      Sortie_Odo.pos.y=R_state.y_pos;
      Sortie_Odo.pos.z=0;
      Sortie_Odo.dir.theta=R_state.theta_pos;
      Sortie_Odo.dir.phi=0;
      Sortie_Odo.dir.psy=0;
      Oldstate=R_state;
      clock_gettime(CLOCK_REALTIME, &TimeLoc);
      Odopos.Set(Sortie_Odo,TimeLoc);
      TimeLocOld=TimeLoc;
    }
    if(PresentMode!=P3D_STARTMODE)
    {//Robot is mooving.
      clock_gettime(CLOCK_REALTIME, &TimeWat);
      wattmetre.record();
      wattmetre.getValues(&Yoctopower, &Yoctoenergy, &Yoctotime);
      InstantlaptopConsumption=batlaptop.power_now();
      if(verbose)
      {
      std::cout << "Yocto power = " << Yoctopower << " - Yocto energy = " << Yoctoenergy << " - Yocto Time = " << Yoctotime << " - Laptopconsumption = "<< InstantlaptopConsumption <<'\n';
      }

      //Logging
      if(Logging){
        clock_gettime(CLOCK_REALTIME, &TimeLog);
        /*
        Logs.push_back(Sortie_Odo.pos.x);
        Logs.push_back(Sortie_Odo.pos.y);
        Logs.push_back(Sortie_Odo.dir.theta);
        Logs.push_back(consignes.cmde_vitesse_RG);
        Logs.push_back(consignes.cmde_vitesse_RD);
        Logs.push_back(impact.dist);
        Logs.push_back(impact.dir.theta);
        Logs.push_back(Reloc_flag);
        logline=8;
        LogTimeOdo.push_back(TimeLoc);
        LogMode.push_back(PresentMode);
        */
        fprintf(savefile,"%ld",(Time.tv_nsec+(Time.tv_sec*1000000000)));
        fprintf(savefile,",%ld",(TimeLoc.tv_nsec+(TimeLoc.tv_sec*1000000000)));
        fprintf(savefile,",%d",PresentMode);
        fprintf(savefile,",%f",Sortie_Odo.pos.x);
        fprintf(savefile,",%f",Sortie_Odo.pos.y);
        fprintf(savefile,",%f",Sortie_Odo.dir.theta);
        fprintf(savefile,",%f",consignes.cmde_vitesse_RG);
        fprintf(savefile,",%f",consignes.cmde_vitesse_RD);
        fprintf(savefile,",%f",impact.dist);
        fprintf(savefile,",%f",impact.dir.theta);
        fprintf(savefile,",%d",Reloc_flag);
        //Ajout Pour LZA
        fprintf(savefile,",%ld",(TimeReLocOld.tv_nsec+(TimeReLocOld.tv_sec*1000000000)));
        fprintf(savefile,",%f",OriginalRelocPose.pos.x);
        fprintf(savefile,",%f",OriginalRelocPose.pos.y);
        fprintf(savefile,",%f",OriginalRelocPose.dir.theta);
        fprintf(savefile,",%f",RelocPose.pos.x);
        fprintf(savefile,",%f",RelocPose.pos.y);
        fprintf(savefile,",%f",RelocPose.dir.theta);
        fprintf(savefile,",%ld",(TimeWat.tv_nsec+(TimeWat.tv_sec*1000000000)));
        fprintf(savefile,",%f",Yoctopower);
        fprintf(savefile,",%f",Yoctoenergy);
        fprintf(savefile,",%ld",Yoctotime);
        fprintf(savefile,",%f",InstantlaptopConsumption);
        fprintf(savefile,",%f",dtus);
        fprintf(savefile,",%ld",(TimeLog.tv_nsec+(TimeLog.tv_sec*1000000000)));
        fprintf(savefile,",%f",DistRG);
        fprintf(savefile,",%f",DistRD);
        fprintf(savefile,",%d",folowing);
        fprintf(savefile,",%d",folowpath.intrusion);
	      fprintf(savefile,",%d",folowpath.obstruction);
        fprintf(savefile,",%f",lwvel);
        fprintf(savefile,",%f",rwvel);
        fprintf(savefile,"\n");
      }
    }
    if(PresentMode==P3D_IDLEMODE){
      if(verbose){
        std::cout << "P3D:IDLEMODE!" << '\n';
      }
      consignes.cmde_vitesse_RG=0;
      consignes.cmde_vitesse_RD=0;
      rob.set_Wheel_Velocity(consignes.cmde_vitesse_RG,consignes.cmde_vitesse_RD);
    }

    if(PresentMode==P3D_ENDINGMODE){
      std::cout << "Ending sucess!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << '\n';
//      rob.Send_Beeps(BeepVictorylength,BeepVictory);
//      sleep(3);
    }
    //usleep(max(delayusleep(Time, 100),0));
    lsleep=delayusleep(Time, 100);
    if(lsleep<0){
      lsleep=0;
    }
    usleep(lsleep);
  }

  consignes.cmde_vitesse_RG=0;
  consignes.cmde_vitesse_RD=0;
  rob.set_Wheel_Velocity(consignes.cmde_vitesse_RG,consignes.cmde_vitesse_RD);
  usleep(5000);
  Mode.Get(PresentMode);
  Mode.Set(P3D_HALTMOD,TimeLoc);
  std::cout<<"P3DX:shuting down"<<std::endl;
  rob.shutdown();
  if(Logging){
    fclose (savefile);
  }
  t2.join();
  std::cout<<"P3DX:end"<<std::endl;
}



void testfunction(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<euclid_position> &Recalage, SafeShared<laser_4m_record> &releveLas1, SafeShared<laser_4m_record> &releveLas2, SafeShared<double> &vit, bool verbose, bool Logging, pioneer_robot::PioneerP3DX &rob, pfoa::waypoint *Path){
  double USs[16];
  int runtest=3;
  int i;
  eucclid_coordinate Relevee={0,0,0};
  std::vector<struct polar_coordinate> Tmp_telemetries;
  switch (runtest) {
    case 1:
      rob.get_US_Dist(USs);
      i=15;
      std::cout << "USValues i="<< i <<" : "<<USs[i] << std::flush;
      Tmp_telemetries=format_US(USs);
      std::cout << " : Recalcule dans le reper robot Dist= "<<Tmp_telemetries[i].dist <<" : Theta = "<< Tmp_telemetries[i].dir.theta*180/M_PI << std::flush;
      Relevee=ConvPolToEuclide(Tmp_telemetries[i]);
      std::cout << " : X= "<<Relevee.x<< " : Y= "<<Relevee.y << std::flush;
  //  for(int i=0;i<16;i++){
  //    std::cout << USs[i] << " : " << std::flush;
  //  }
      std::cout <<'\n';
      usleep(100000);
      break;
    case 2:
      std::cout << "Waypoint1 : X="<< Path[1].x <<" : Y=" << Path[1].y << " : S="<< Path[1].D << '\n';
      usleep(1000000);
      break;
    case 3:
      rob.set_Wheel_Velocity(-0.02,0.02);
      break;
  }
}


void ComputCeentering(std::vector<struct polar_coordinate> Telemetries, P3D_Consignes_Moteurs &consignes){
  std::vector<int> IselectedsTmp, PoidLefts, PoidRight;
  std::vector<double> Alefts, Blefts, Aright, Bright;
  double amid,bmid;
  struct polar_coordinate Point1,Point2;
  std::vector<struct polar_coordinate> Point1Left,Point2Left,Point1Right,Point2Right;
  int sizeinit,sizeleft;
  //Recuperation de la droite qui réunis le plus de point => Ce sera l'axe supposé de notre couloir.
  ransaclignefrompole(Telemetries, amid, bmid, 0.01, Point1, Point2);
  //On isole tout les point du coter droit de l'orientation.
  //tans qu'il reste plus de 30%des points du coté ou 4Droites estimés
    //RANSAC sur les points qu'il reste du coté
    //Enregistrement des caractéristiques de la droite a,b,Poid
    //On enlève les points selectionees du relevee.

  //On isole tout les point du coter gauche de l'orientation.
  //tans qu'il reste plus de 30%des points du coté ou 4Droites estimés
    //RANSAC sur les points qu'il reste du coté
    //Enregistrement des caractéristiques de la droite a,b,Poid
    //On enlève les points selectionees du relevee.

  //Quels droites sont les bonnes a prendre...
  //On prend en référence la droite la plus "lourde"

  // Pour chacune des droite appartement a l'autre coté de l'orientation observé.
    //Si l'orientation de la droite ne difert pas trop
    //Calcule de la "distance" qui sépart les deux droites.
    //Si distance inférieur a ce qui est connu sauvegarde

  //Generation de la trajectoire a ateindre/suivre.

  //Generation de la commande.




  //Ou alors:
  //Generation d'une commande En direction de l'arrivée donnée :
  //Prise des points qui sont entre pi/2 et -pi/2 aurout de cette trajectoire en deux nuages distinct.
  //Calcule du barycentre de ces deux nuages. Chaque barycentre génèreune
  //"force de répultion" sur la commande proportiollement a l'inverce du careede la distance.
}



euclid_position Compute_repose(euclid_position RelocPose, P3D_Consignes_Moteurs consignes, double dtus){
  double Entreroue=ENTRE_ROUE;//0.328239;//0.3564514342;//;0.33;//0.1905*2;
  double V= (consignes.cmde_vitesse_RG+consignes.cmde_vitesse_RD)/2;
  double dt= (consignes.cmde_vitesse_RD-consignes.cmde_vitesse_RG)/Entreroue;
  euclid_position Posfin={{0,0,0},{0,0,0}};
  Posfin.pos.x=RelocPose.pos.x+(V*dtus/1000000)*cos(RelocPose.dir.theta+(dt*dtus/1000000)/2);//m
  Posfin.pos.y=RelocPose.pos.y+(V*dtus/1000000)*sin(RelocPose.dir.theta+(dt*dtus/1000000)/2);;//m
  Posfin.dir.theta=RelocPose.dir.theta+dt*dtus/1000000;//rad
  return Posfin;
}

polar_coordinate Rearange_Impact(polar_coordinate impact){
/*
  if(abs(impact.dir.theta)>(0.85*M_PI/2))
  {
    if(impact.dir.theta>0)
    {
      impact.dir.theta=impact.dir.theta-M_PI/20;
    }
    else
    {
      impact.dir.theta=impact.dir.theta+M_PI/20;
    }
  }
  /*
  if(abs(impact.dir.theta)>M_PI/4){
    if(impact.dir.theta>0){
      impact.dir.theta=impact.dir.theta-M_PI/8;
    }else{
      impact.dir.theta=impact.dir.theta+M_PI/8;
    }
  }
  else{
    impact.dir.theta=impact.dir.theta*0.9;
  }*/
  return impact;
}
