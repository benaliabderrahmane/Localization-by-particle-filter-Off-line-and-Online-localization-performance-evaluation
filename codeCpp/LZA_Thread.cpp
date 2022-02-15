#include "LZA_Thread.h"
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

struct LZACorel{
  localization::ROBLOC pos;
  double corel;
};
bool CompLZA(LZACorel me, LZACorel is){
  return (me.corel<is.corel);
}

//void Recomput_Weeldist(euclid_position Oldstate,euclid_position R_state, double &ssg, double &ssd);
//void compute_Fenetre_Error_Odo(euclid_position R_state, double distLeft, double distRight, Eigen::MatrixXd &P, double &x1, double &x2, double &y1, double &y2);
//void SimuOdo(euclid_position StartPos,euclid_position &SimPos, double DistRG, double DistRD);
void Format_LAS(laser_4m_record Las1, laser_4m_record Las2, double DIST[]);
bool isLZAResult(localization::ROBLOC LZAres,euclid_position Odosource, double tetTol, bool Trusted,euclid_position LastRecal);
localization::ROBLOC MultiresLZA(localization::LZALocalization lzamod, double Las[57], double dist, P3D_Robot_State Rob_State);
//void FusionLZA(std::vector<double> ixOdo,std::vector<double> iyOdo,std::vector<double> itOdo, localization::ROBLOC LZAres, int Mode, localization::ROBLOC &RelocVal));
bool FusionLZA(double Iodo[6],int Mode, euclid_position LZARes,euclid_position OdoEstime, euclid_position &RelocVal);

bool isLZAusable(euclid_position odoposP3D);

double GetRTY(std::vector<struct polar_coordinate> Mesures);

void LZAthread(SafeShared<euclid_position> &RecalageLoc, SafeShared<laser_4m_record> &las1, SafeShared<laser_4m_record> &las2, SafeShared<euclid_position> &odom, SafeShared<int> &Mode, SafeShared<int> &ModeCompas, SafeShared<euclid_position> &LZA_res, SafeShared<double> &Compas_res, bool verbose, bool Logging){
  bool running=true;
  bool new_estime_flag=false;
  bool followSimStateOld, followSimState=false;
  int presentMode, presentModeCompas;
  struct timespec StartTime, Time, TLas1, TLas2, Todo, Toldodo, LauncheTime, TimeLocOld, TimeEstime;
  laser_4m_record relevel1, relevel2;
  euclid_position odoposP3D, odopos, OdoPosOld, offset, oldOdosim, OdoposSim, OdoEstime, Val_Recalage, LZA_estime;
  double distLeft, distRight;
  double datas_loc[57];
  double Dsleeped;
  double EstimeCompas, LastCompas, LZACompas;
  double Total_Reloc=0;
  std::vector<struct polar_coordinate> Telemetries;
  std::vector<struct polar_coordinate> Mesures_Droite;
  std::vector<struct polar_coordinate> OldLas1,OldLas2;
  struct timespec Las1Old, Las2Old, Las1Time, Las2Time;
  struct timespec TimeFormat,TimeLZA;
  struct timespec TimeCompas,TimeLog;
  int LZArun=0;

  std::vector<double> Logs;
  std::vector<double> LZA_LAS;
  std::vector<struct timespec> LogTime;
  std::vector<int> LogMode;
  std::vector<double> Ixinit(2),Iyinit(2),Itinit(2),Ix(2),Iy(2),It(2);
  double drecalage,thetad;
  Ixinit=initInterval(-0.58675,0.48832);
  Iyinit=initInterval(-0.52998,0.54509);
  Itinit=initInterval(-0.53878,0.53629);
  euclid_position LastLoc, EstimeInit, LoopDestect_pose;
  double Ds_loop_detect;
  int consfail=0;
  int logline=0;
  int Reloc_Flag, SleepReloc_Flag=0;

  OdoPosOld.pos.x=0;
  OdoPosOld.pos.y=1.1;
  OdoPosOld.pos.z=0;
  OdoPosOld.dir.phi=0;
  OdoPosOld.dir.psy=0;
  OdoPosOld.dir.theta=0;
  OdoposSim=OdoPosOld;
  Val_Recalage.pos.x=999;
  Val_Recalage.pos.y=999;
  Val_Recalage.pos.z=0;
  Val_Recalage.dir.phi=0;
  Val_Recalage.dir.psy=0;
  Val_Recalage.dir.theta=999;
  LZA_estime=Val_Recalage;
  LastLoc=Val_Recalage;
  double IOdo[6]={0,0,0,0,0,0};



  bool simuon=false;
  double x1,x2,y1,y2,t1,t2;
  double x1sim,x2sim,y1sim,y2sim=0;
  bool verif, logneed;
  std::vector<euclid_position> Histo_Odo;
  std::vector<localization::ROBLOC> Histo_LZA;
  int Nloc, Nlocoffset, icoherence;
  double dtol;
  int lsleep;

  std::string mapfolder =PID_PATH("LZA_files");
  std::cout<<"Map folder : "<<mapfolder<<std::endl;
  localization::LZALocalization lzamod (mapfolder.c_str(),0,0,0);//LZALocalization map exactly on Experimental Map. on a observe un ecart de 0.4 rad pout le theta en offset...
  lzamod.set_Fenetre(-2,-2,4,4);
  lzamod.set_Position(0,1.1,0);
  P3D_Robot_State etat_robot;
  localization::ROBLOC loclza;


  Eigen::MatrixXd PLZA(3,3);
  bool Monoparticular;
/**************************************************************************
//  PLZA << pow(0.038383,2),0,0,0,pow(0.037831,2),0,0,0,pow(0.023118,2);
PLZA << pow(0.1554,2),0,0,0,pow(0.1736,2),0,0,0,pow(0.2353,2);
  int Repet_Lost=19;//20;//11;
  int Repet_maintain=14;//10;
  double dist_reobserve_m=0.4*1.7605;//1.5941;//1.5754;
  double Ttol=(3.44*M_PI)/2;//3.0156;//2*M_PI/57;
  int Failtol=21;//19;
    Monoparticular=true;
/*****************************************************************/
  Monoparticular=false;
//  PLZA << pow(0.3186,2),0,0,0,pow(0.1727,2),0,0,0,pow(0.1671,2);
  PLZA << pow(0.1182,2),0,0,0,pow(0.1432,2),0,0,0,pow(0.1622,2);//Original
//  PLZA << pow(0.1182,2),0,0,0,pow(0.1432,2),0,0,0,pow(0.13,2);
  int Repet_Lost=19;//6;
  int Repet_maintain=14;//6;
  double dist_reobserve_m=0.4*0.55;//0.467;//2.4966;
  double Ttol=M_PI/4;//2.31;//3.8983/3;//4.066;//
  int Failtol=19;//18;//19;
  lzamod.set_DXteCoef(2.57);
  lzamod.set_DXpourCoef(3.46);
/****************************************************************/

  int Fail_Count=0;
  Eigen::MatrixXd P(3,3);
  P=PLZA;
  Eigen::MatrixXd PSim(3,3);
  PSim=PLZA;
  double Ttolecrase;
  bool Loc_requis=true;
  bool Heated=false;
  bool Localised_once=false;
  euclid_position HeatedPos=OdoPosOld;
  euclid_position NewHeatedPos=OdoPosOld;

	clock_gettime(CLOCK_REALTIME, &Time);
  StartTime=Time;
  TLas1=Time;
  TLas2=Time;
  Todo=Time;
  Toldodo=Time;
  LauncheTime=Time;
  TimeLocOld=Time;
  TimeEstime=Time;

  std::string filename = PID_PATH("+PANORAMA_Log/"+std::to_string(LauncheTime.tv_sec)+"/LZA_LOG.txt");
  std::ofstream savefile;
  if(Logging){
    savefile.open(filename,std::ios::out);
  }
  std::string filenameRl = PID_PATH("+PANORAMA_Log/"+std::to_string(LauncheTime.tv_sec)+"/Right_Las_LOG.txt");
  std::ofstream savefileRl;
  if(Logging){
    savefileRl.open(filenameRl,std::ios::out);
  }

  LauncheTime=Mode.Get(presentMode);
  Mode.Set(LZA_IDLEMODE,LauncheTime);
  presentMode=LZA_IDLEMODE;
  Las2Old=LauncheTime;
  Las1Old=LauncheTime;

  Nlocoffset=0;
  LZACompas=999;

  struct timespec OtherRelocTime, OldOtherRelocTime, P3DPosTime;
  euclid_position OtherRelocPos, P3DPos;
  double YRT=999;

  while(running){
    ModeCompas.Get(presentModeCompas);
    Mode.Get(presentMode);
    clock_gettime(CLOCK_REALTIME, &StartTime);
    logneed=false;
    switch (presentModeCompas) {
      case COMPAS_IDLEMODE:
        EstimeCompas=999;
        LastCompas=EstimeCompas;
        break;
      case COMPAS_OBSERVEMODE:
      case COMPAS_RUNNINGMODE:
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
        clock_gettime(CLOCK_REALTIME, &TimeCompas);
        EstimeCompas=Compute_ThetaCompas(Telemetries,odoposP3D);
        //if(EstimeCompas!=999){
          LastCompas=EstimeCompas;
        //}
        logneed=true;
        if(presentModeCompas==COMPAS_RUNNINGMODE)
        {
          Compas_res.Set(EstimeCompas,TimeCompas);
        }
        break;
      case COMPAS_LOGLAS:
        Todo=odom.Get(odoposP3D);
        Las2Time=las2.Get(relevel2);
        Las1Time=las1.Get(relevel1);
        if (isafter(Las2Time,Las2Old)){
          OldLas2=format_Telemetre(relevel2.data,relevel2.last_lenght,0.03,0,M_PI);
          Las2Old=Las2Time;
        }
        if (isafter(Las1Time,Las1Old)){
          //OldLas1=format_Telemetre(relevel1.data,relevel1.last_lenght,0.03,0,0.7031*M_PI/180);
          OldLas1=format_Telemetre(relevel1.data,relevel1.last_lenght,0.03,0,0);
          Las1Old=Las1Time;
        }

        Telemetries.clear();
        if(isafter(Las2Old,LauncheTime) && !(enoughttimepassed(StartTime, Las2Old, 1000))){
          Telemetries.insert(Telemetries.end(),OldLas2.begin(),OldLas2.end());
        }
        if(isafter(Las1Old,LauncheTime) &&  !(enoughttimepassed(StartTime, Las1Old, 1000))){
          Telemetries.insert(Telemetries.end(),OldLas1.begin(),OldLas1.end());
        }
        clock_gettime(CLOCK_REALTIME, &Time);

        savefileRl<<(StartTime.tv_nsec+(StartTime.tv_sec*1000000000))<<","<<presentModeCompas;
        for(struct polar_coordinate E:Telemetries){
          savefileRl<<','<<E.dist<<','<<E.dir.theta;
        }
        savefileRl<<"\n";
        break;
      case COMPAS_RTMODE:
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
        Mesures_Droite.clear();
        for(int i=0; i<Telemetries.size();i++){
          if(Telemetries[i].dir.theta<(-M_PI/4) && Telemetries[i].dir.theta>(-3*M_PI/4)){
            Mesures_Droite.push_back(Telemetries[i]);
          }
        }
        clock_gettime(CLOCK_REALTIME, &TimeCompas);
        EstimeCompas=Compute_ThetaCompas(Telemetries,odoposP3D);
        //if(EstimeCompas!=999){
          LastCompas=EstimeCompas;
        //}
        YRT=GetRTY(Mesures_Droite);
        logneed=true;
        break;
      default:
        break;
    }
    Reloc_Flag=0;
    SleepReloc_Flag=0;
    followSimStateOld=followSimState;
    switch (presentMode) {
      case LZA_IDLEMODE:
        Todo=odom.Get(OdoPosOld);
        P << pow(0.8,2),0,0,0,pow(0.5,2),0,0,0,pow(0.2062,2);//Non actif on ne sait donc pas bien ou on se trouve.
        Val_Recalage.pos.x=999;
        Val_Recalage.pos.y=999;
        Val_Recalage.pos.z=0;
        Val_Recalage.dir.phi=0;
        Val_Recalage.dir.psy=0;
        Val_Recalage.dir.theta=999;
        LastLoc=Val_Recalage;
        Nloc=0;
        LZArun=0;
        Fail_Count=0;
        followSimState=false;
        Localised_once=false;
        break;
      case LZA_HALTINGMODE:
        running = false;
        break;
      case LZA_FUS_INIT_COMPASMODE:
      case LZA_INIT_COMPASMODE:
        LZACompas=EstimeCompas;
      case LZA_FUS_INITMODE:
      case LZA_INITMODE: //On préchauffe LZA => On execute LZA mais on n'ecrase pas l'odometrie.
        //Recupération LAS-Odometrie.
        if(!Heated){
          HeatedPos.pos.x=0;
          HeatedPos.pos.y=0;
          HeatedPos.dir.theta=0;
        }
        las2.Get(relevel2);
        las1.Get(relevel1);
        Todo=odom.Get(odoposP3D);
        OtherRelocTime=RecalageLoc.Get(OtherRelocPos);
        clock_gettime(CLOCK_REALTIME, &TimeEstime);

        if (!isLZAusable(odoposP3D)){
            LZArun=0;
            break;
        }
        LZArun=1;
        //Lecture des recalage.
        //P3DPos=odoposP3D;
        //Si un nouveau recalage a lieux ET que la position odométrique lue est a moins de 10cm
        if(isafter(OtherRelocTime,OldOtherRelocTime)&&((sqrt((odoposP3D.pos.x-OtherRelocPos.pos.x)*(odoposP3D.pos.x-OtherRelocPos.pos.x)+(odoposP3D.pos.y-OtherRelocPos.pos.y)*(odoposP3D.pos.y-OtherRelocPos.pos.y)))<0.1)){
          //Alors on supose que la relocalisation a été prise en compte par l'odométrie
          // - P3D=Recalage
          // - MAJ de la date du dernier recalage
          OldOtherRelocTime=OtherRelocTime;
          OdoPosOld=OtherRelocPos;
        }
        //SI L'odometrie est postérieur a la dernière estimation on continu.
        if(isafter(Todo,Toldodo)){
          logneed=true;
          Format_LAS(relevel1, relevel2,datas_loc);
          clock_gettime(CLOCK_REALTIME, &TimeFormat);
          if (!isLZAusable(odoposP3D)){
            break;
          }


          //Estimation OdoRob.
          Recomput_Weeldist(OdoPosOld,odoposP3D,distLeft,distRight);

          //Calcule fenêtre avec Erreur Odo
          if(!Heated){
            compute_Fenetre_Error_Odo(OdoPosOld,distLeft,distRight,P,x1,x2,y1,y2,t1,t2);
            odopos=odoposP3D;
          }else{
            SimuOdo(HeatedPos,NewHeatedPos,distLeft,distRight);
            compute_Fenetre_Error_Odo(HeatedPos,distLeft,distRight,P,x1,x2,y1,y2,t1,t2);
            odopos=NewHeatedPos;
          }
          Dsleeped=sqrt(pow((Val_Recalage.pos.x-odopos.pos.x),2)+pow((Val_Recalage.pos.y-odopos.pos.y),2));
          if (Dsleeped<dist_reobserve_m){//On c'est relocalisé il y a peut on ne fais rien.
            SleepReloc_Flag=1;
            OdoPosOld=odoposP3D;
            Toldodo=Todo;
            HeatedPos = NewHeatedPos;
            break;
          }/**/
          SleepReloc_Flag=0;

          if(Localised_once){
            drecalage=sqrt(pow((LastLoc.pos.x-odopos.pos.x),2)+pow((LastLoc.pos.y-odopos.pos.y),2));
            if(drecalage!=0){
              thetad=atan2(odopos.pos.y-LastLoc.pos.y,odopos.pos.x-LastLoc.pos.x);
              Ix=Iadd(Isub(initInterval(LastLoc.pos.x,LastLoc.pos.x),
                          Ixinit),
                      Imul(Icos(Isub(initInterval(thetad,thetad),
                                Itinit)),
                          initInterval(drecalage,drecalage)));
              Iy=Iadd(Isub(initInterval(LastLoc.pos.y,LastLoc.pos.y),
                          Iyinit),
                      Imul(Icos(Isub(initInterval(thetad,thetad),
                                      Itinit)),
                          initInterval(drecalage,drecalage)));
            }else{
              Ix=Isub(initInterval(LastLoc.pos.x,LastLoc.pos.x),Ixinit);
              Iy=Isub(initInterval(LastLoc.pos.y,LastLoc.pos.y),Iyinit);
            }
            Ix=Iadd(Icentering(initInterval(x1,x2)),Ix);
            Iy=Iadd(Icentering(initInterval(y1,y2)),Iy);
            IVals(Ix,x2,x1);
            IVals(Iy,y2,y1);
          }else{
            x1=x1+1+consfail*0;//1;//2;
            x2=x2-1-consfail*0;//1;//2;
            y1=y1+1+consfail*0;//1;//2;
            y2=y2-1-consfail*0;//1;//2;
          }
          if(Monoparticular){
          //Si on suis une supposition
          if(followSimState==false){//On ne simule pas d'estimation.
            //EstimeLZA centré sur l'estimation du robot et la fenetre assoiee.
            lzamod.set_Fenetre(x1,y1,x2,y2);//forcage de la fenetre autour de la position odométrique.
            etat_robot.x_pos=odopos.pos.x;
            etat_robot.y_pos=odopos.pos.y;
            etat_robot.theta_pos=odopos.dir.theta;
            loclza=lzamod.Localise(datas_loc,4.0,etat_robot);
            if(presentMode==LZA_INIT_COMPASMODE){
                loclza.theta=LZACompas;
            }
            if(isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)){
              LZA_estime.pos.x=loclza.x;
              LZA_estime.pos.y=loclza.y;
              LZA_estime.dir.theta=loclza.theta;
              new_estime_flag=true;
            }

            if (isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)){//Si LZA nous donne un resultat valide.
                //initialisation de la position a partir de la quelle on simule a partir de la nouvelle position.
                Nloc=1;
                followSimState=true;
                //if(fabs(odopos.dir.theta-loclza.theta)>(Ttol)){
                //if((fabs(odopos.dir.theta-loclza.theta)>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)<M_PI)||
              //(((2*M_PI)-fabs(odopos.dir.theta-loclza.theta))>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)>M_PI)){
                  //oldOdosim.dir.theta=odopos.dir.theta;
                //}else{
                oldOdosim.dir.theta=loclza.theta;
                //}
                oldOdosim.pos.x=loclza.x;
                oldOdosim.pos.y=loclza.y;
                LastLoc=oldOdosim;
                PSim=P;
              }else{
                Nloc=0;
                Fail_Count=0;
                followSimState=false;
                Loc_requis=true;
              }
          }else{//On a une position a simuler.
            //Calcule fenêtre avec Erreur Odo simulee
            compute_Fenetre_Error_Odo(oldOdosim,distLeft,distRight,PSim,x1sim,x2sim,y1sim,y2sim,t1,t2);
            SimuOdo(oldOdosim,OdoposSim, distLeft, distRight);
            oldOdosim=OdoposSim;
            //EstimeLZA centré sur l'estimation de position.
            lzamod.set_Fenetre(x1sim,y1sim,x2sim,y2sim);//forcage de la fenetre autour de la position odométrique.
            etat_robot.x_pos=OdoposSim.pos.x;
            etat_robot.y_pos=OdoposSim.pos.y;
            etat_robot.theta_pos=OdoposSim.dir.theta;
            //EstimeLZA centré sur l'estimation de position.
            loclza=lzamod.Localise(datas_loc,4.0,etat_robot);
            if(presentMode==LZA_INIT_COMPASMODE){
                loclza.theta=LZACompas;
            }
            //clock_gettime(CLOCK_REALTIME, &TimeEstime);
            if(isLZAResult(loclza,OdoposSim,Ttol,Localised_once,Val_Recalage)){//L'erreur odometrique simulee permet tout de même d'avoire un resultat.
              LZA_estime.pos.x=loclza.x;
              LZA_estime.pos.y=loclza.y;
              LZA_estime.dir.theta=loclza.theta;
              new_estime_flag=true;

              //increment validation simulation.
              Nloc++;
            }else{//L'erreur odometrique ne permet pas de valider un resultat, reset de la simulation.
              lzamod.set_Fenetre(x1,y1,x2,y2);//forcage de la fenetre autour de la position odométrique.
              etat_robot.x_pos=odopos.pos.x;
              etat_robot.y_pos=odopos.pos.y;
              etat_robot.theta_pos=odopos.dir.theta;
              loclza=lzamod.Localise(datas_loc,4.0,etat_robot);
              if(presentMode==LZA_INIT_COMPASMODE){
                  loclza.theta=LZACompas;
              }
              //clock_gettime(CLOCK_REALTIME, &TimeEstime);
              if(isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)){
                LZA_estime.pos.x=loclza.x;
                LZA_estime.pos.y=loclza.y;
                LZA_estime.dir.theta=loclza.theta;
                new_estime_flag=true;
                //Reset Simu
                Nloc=1;
                followSimState=true;
                //if(fabs(odopos.dir.theta-loclza.theta)>(Ttol)){

                //if((fabs(odopos.dir.theta-loclza.theta)>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)<M_PI)||
                  //(((2*M_PI)-fabs(odopos.dir.theta-loclza.theta))>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)>M_PI)){
                  //oldOdosim.dir.theta=odopos.dir.theta;
                //}else{
                oldOdosim.dir.theta=loclza.theta;
                //}
                oldOdosim.pos.x=loclza.x;
                oldOdosim.pos.y=loclza.y;
                EstimeInit=LZA_estime;
                PSim=P;
              }else{//Simu en cours : Simu Echec : localisation Echec
                if(Fail_Count<Failtol){
                  Fail_Count++;
                }else{
                  //Echec de localisation : perdu
                  Fail_Count=0;
                  consfail++;
                  Nloc=0;
                  followSimState=false;
                  Loc_requis=true;
                }
              }
            }
          }
        }else{///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          lzamod.set_Fenetre(x1,y1,x2,y2);//forcage de la fenetre autour de la position odométrique.
          etat_robot.x_pos=odopos.pos.x;
          etat_robot.y_pos=odopos.pos.y;
          etat_robot.theta_pos=odopos.dir.theta;
          loclza=lzamod.Localise(datas_loc,4.0,etat_robot);

          if(isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)&&((presentMode!=LZA_INIT_COMPASMODE && presentMode!=LZA_FUS_INIT_COMPASMODE)||fabs(LZACompas)<=2*M_PI)){
            if(!followSimState){
                Nloc=Nloc+1;
            }else{
              Nloc=1;
              followSimState=true;
              Fail_Count=0;
            }
          }else{
            if(Fail_Count<Failtol){
              Fail_Count++;
            }else{
              Nloc=0;
              Fail_Count=0;
              followSimState=false;
              Loc_requis=true;
              Fail_Count=0;
              consfail++;
            }
          }
        }
          //OdoPosOld=odoposP3D;
          //Toldodo=Todo;
          //HeatedPos = NewHeatedPos;

          //Si les contition de relocalisation sont remplies:
          if((Loc_requis && Nloc>=Repet_Lost)||(!Loc_requis && Nloc>=Repet_maintain)){
            Localised_once=true;
            if(Monoparticular){
            //Set estimations
            Val_Recalage=OdoposSim;
            TimeLocOld=TimeEstime;
            LastLoc=EstimeInit;
            Reloc_Flag=1;
            //Reset P
            P=PSim;
            //Set offset.
            HeatedPos=Val_Recalage;
            Toldodo=TimeEstime;
            Loc_requis=false;
            consfail=0;
            Nloc=0;
            Fail_Count=0;
            Heated=true;
          }else{
            if((presentMode==LZA_INIT_COMPASMODE || presentMode==LZA_FUS_INIT_COMPASMODE) && fabs(LZACompas)<2*M_PI){
              loclza.theta=LZACompas;
            }
            HeatedPos.pos.x=loclza.x;
            HeatedPos.pos.y=loclza.y;
            HeatedPos.dir.theta=loclza.theta;

            HeatedPos.pos.x=HeatedPos.pos.x+0.2*cos(HeatedPos.dir.theta);
            HeatedPos.pos.y=HeatedPos.pos.y+0.2*sin(HeatedPos.dir.theta);

            Val_Recalage=HeatedPos;
            LastLoc=HeatedPos;
            Toldodo=TimeEstime;
            P=PLZA;
            Reloc_Flag=1;
            Loc_requis=false;
            Nloc=0;
            consfail=0;
            Fail_Count=0;
            Heated=true;
          }
          }
          if(presentMode==LZA_FUS_INIT_COMPASMODE||presentMode==LZA_FUS_INITMODE){
          //Intersection des intervals entre la position odométrique reconstituée de l'incertitude de LZA
          }
        }
        clock_gettime(CLOCK_REALTIME, &TimeLZA);
        break;
      case LZA_BACKGROUNDMODE:
        //en Background : datas_loc on enregistre simplement les lasers observés
        las2.Get(relevel2);
        las1.Get(relevel1);
        Format_LAS(relevel1, relevel2,datas_loc);
        Todo=odom.Get(OdoPosOld);
        P << pow(0.8,2),0,0,0,pow(0.5,2),0,0,0,pow(0.2062,2);//Non actif on ne sait donc pas bien ou on se trouve.
        Val_Recalage.pos.x=999;
        Val_Recalage.pos.y=999;
        Val_Recalage.pos.z=0;
        Val_Recalage.dir.phi=0;
        Val_Recalage.dir.psy=0;
        Val_Recalage.dir.theta=999;
        LastLoc=Val_Recalage;
        Nloc=0;
        consfail=0;
        Fail_Count=0;
        followSimState=false;
        Localised_once=false;
        Fail_Count=0;
        logneed=true;
        break;
      case LZA_FUS_RUNING_COMMPASMODE:
      case LZA_RUNING_COMMPASMODE:
        LZACompas=EstimeCompas;
      case LZA_FUS_RUNINGMODE:
      case LZA_RUNINGMODE:////Confirmation par Coherence avec une simulation odométrique. Working
        if (verbose){
          std::cout<<"LZA_T==============================================================="<<std::endl;
        }
        if(Heated){//premier cycle de sortie LZA sauté : relocalisation robot avec
        /*  if(presentMode==LZA_RUNING_COMMPASMODE && LastCompas!=999){
            HeatedPos.dir.theta=LastCompas;
          }/**/
          OdoPosOld=HeatedPos;
          RecalageLoc.Set(HeatedPos, Toldodo);
          Total_Reloc++;
          Toldodo=StartTime;
          Heated=false;
          break;
        }
        //Recupération LAS-Odometrie.
        las2.Get(relevel2);
        las1.Get(relevel1);
        Todo=odom.Get(odopos);
        clock_gettime(CLOCK_REALTIME, &TimeEstime);
        //SI L'odometrie est postérieur a la dernière estimation on continu.
        if(isafter(Todo,Toldodo)){
          logneed=true;

          Format_LAS(relevel1, relevel2,datas_loc);
          clock_gettime(CLOCK_REALTIME, &TimeFormat);

          if (!isLZAusable(odoposP3D)){
            LZArun=0;
            break;
          }
          LZArun=1;
          //Estimation OdoRob.
          Recomput_Weeldist(OdoPosOld,odopos,distLeft,distRight);
          //Calcule fenêtre avec Erreur Odo
          compute_Fenetre_Error_Odo(OdoPosOld,distLeft,distRight,P,x1,x2,y1,y2,t1,t2);

          Dsleeped=sqrt(pow((Val_Recalage.pos.x-odopos.pos.x),2)+pow((Val_Recalage.pos.y-odopos.pos.y),2));
          if (Dsleeped<dist_reobserve_m){//On c'est relocalisé il y a peut on ne fais rien.
            SleepReloc_Flag=1;
            OdoPosOld=odopos;
            Toldodo=Todo;
            break;
          }/**/
          SleepReloc_Flag=0;
          std::cout << "LZA--Sleeped" << '\n';

          if(Localised_once){
            drecalage=sqrt(pow((LastLoc.pos.x-odopos.pos.x),2)+pow((LastLoc.pos.y-odopos.pos.y),2));
            if(drecalage!=0){
              thetad=atan2(odopos.pos.y-LastLoc.pos.y,odopos.pos.x-LastLoc.pos.x);
              Ix=Iadd(Isub(initInterval(LastLoc.pos.x,LastLoc.pos.x),
                          Ixinit),
                      Imul(Icos(Isub(initInterval(thetad,thetad),
                                Itinit)),
                          initInterval(drecalage,drecalage)));
              Iy=Iadd(Isub(initInterval(LastLoc.pos.y,LastLoc.pos.y),
                          Iyinit),
                      Imul(Icos(Isub(initInterval(thetad,thetad),
                                      Itinit)),
                          initInterval(drecalage,drecalage)));
            }else{
              Ix=Isub(initInterval(LastLoc.pos.x,LastLoc.pos.x),Ixinit);
              Iy=Isub(initInterval(LastLoc.pos.y,LastLoc.pos.y),Iyinit);
            }
            Ix=Iadd(Icentering(initInterval(x1,x2)),Ix);
            Iy=Iadd(Icentering(initInterval(y1,y2)),Iy);
            IVals(Ix,x2,x1);
            IVals(Iy,y2,y1);
          }else{
            x1=x1+1+consfail*0;//1;//2;
            x2=x2-1-consfail*0;//1;//2;
            y1=y1+1+consfail*0;//1;//2;
            y2=y2-1-consfail*0;//1;//2;
          }

          if(x1>500){
            std::cout<<"C'est quoi ce bronx la fenêtre!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<'\n';
            std::cout<<"Distance depuis le recalage = "<<drecalage<<" : Dtheta = "<<thetad<<'\n';
            std::cout<<"X = ["<<x2<<","<<x1<<"] : Y = ["<<y2<<","<<y1<<"]"<<'\n';
            std::cout<<"LastLoc X = "<<LastLoc.pos.x<<": Y = "<<LastLoc.pos.y<<'\n';
            std::cout<<"odopos X = "<<odopos.pos.x<<": Y = "<<odopos.pos.y<<'\n';
          }
          if(Monoparticular){
          //Si on suis une supposition
          if(followSimState==false){//On ne simule pas d'estimation.
            //EstimeLZA centré sur l'estimation du robot et la fenetre assoiee.
            lzamod.set_Fenetre(x1,y1,x2,y2);//forcage de la fenetre autour de la position odométrique.
            etat_robot.x_pos=odopos.pos.x;
            etat_robot.y_pos=odopos.pos.y;
            etat_robot.theta_pos=odopos.dir.theta;
            loclza=lzamod.Localise(datas_loc,4.0,etat_robot);
            std::cout << "LZA--LOCALISE" << '\n';
            if(presentMode==LZA_RUNING_COMMPASMODE || presentMode==LZA_FUS_RUNING_COMMPASMODE){
                loclza.theta=LZACompas;
            }
            //clock_gettime(CLOCK_REALTIME, &TimeEstime);
            if(isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)){
              LZA_estime.pos.x=loclza.x;
              LZA_estime.pos.y=loclza.y;
              LZA_estime.dir.theta=loclza.theta;
              new_estime_flag=true;
              std::cout << "LZA--RES" << '\n';
            }
              if (isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)){//Si LZA nous donne un resultat valide.
              //initialisation de la position a partir de la quelle on simule a partir de la nouvelle position.
              std::cout << "LZA--FIRSTLOC" << '\n';
              Nloc=1;
              followSimState=true;
              //if(fabs(odopos.dir.theta-loclza.theta)>(Ttol)){
              //if((fabs(odopos.dir.theta-loclza.theta)>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)<M_PI)||
                //(((2*M_PI)-fabs(odopos.dir.theta-loclza.theta))>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)>M_PI)){
                //oldOdosim.dir.theta=odopos.dir.theta;
              //}else{/**/
                oldOdosim.dir.theta=loclza.theta;
              //}/**/
              oldOdosim.pos.x=loclza.x;
              oldOdosim.pos.y=loclza.y;
              LastLoc=oldOdosim;
              PSim=P;
              }else{
                Nloc=0;
                consfail++;
                followSimState=false;
                Fail_Count=0;
                Loc_requis=true;
                std::cout << "LZA--Not a result :" << '\n';
                std::cout << "LZA--Estim X :" << loclza.x <<" - Y : "<< loclza.y <<" - Thet : "<< loclza.theta << '\n';
                std::cout << "LZA--Odo X :" << odopos.pos.x <<" - Y : "<< odopos.pos.y <<" - Thet : "<< odopos.dir.theta << '\n';
                std::cout << "LZA--ttol = "<< Ttol << '\n';
              }
          }else{//On a une position a simuler.
            std::cout << "LZA--SIMU" << '\n';
            //Calcule fenêtre avec Erreur Odo simulee
            compute_Fenetre_Error_Odo(oldOdosim,distLeft,distRight,PSim,x1sim,x2sim,y1sim,y2sim,t1,t2);
            SimuOdo(oldOdosim,OdoposSim, distLeft, distRight);
            oldOdosim=OdoposSim;
            //EstimeLZA centré sur l'estimation de position.
            lzamod.set_Fenetre(x1sim,y1sim,x2sim,y2sim);//forcage de la fenetre autour de la position odométrique.
            etat_robot.x_pos=OdoposSim.pos.x;
            etat_robot.y_pos=OdoposSim.pos.y;
            etat_robot.theta_pos=OdoposSim.dir.theta;
            //EstimeLZA centré sur l'estimation de position.
            loclza=lzamod.Localise(datas_loc,4.0,etat_robot);
            if(presentMode==LZA_RUNING_COMMPASMODE){
                loclza.theta=LZACompas;
            }
            //clock_gettime(CLOCK_REALTIME, &TimeEstime);
            if(isLZAResult(loclza,OdoposSim,Ttol,Localised_once,Val_Recalage)){//L'erreur odometrique simulee permet tout de même d'avoire un resultat.
              LZA_estime.pos.x=loclza.x;
              LZA_estime.pos.y=loclza.y;
              LZA_estime.dir.theta=loclza.theta;
              new_estime_flag=true;

              //increment validation simulation.
              Nloc++;
            }else{//L'erreur odometrique ne permet pas de valider un resultat, reset de la simulation.
              lzamod.set_Fenetre(x1,y1,x2,y2);//forcage de la fenetre autour de la position odométrique.
              etat_robot.x_pos=odopos.pos.x;
              etat_robot.y_pos=odopos.pos.y;
              etat_robot.theta_pos=odopos.dir.theta;
              loclza=lzamod.Localise(datas_loc,4.0,etat_robot);
              if(presentMode==LZA_RUNING_COMMPASMODE){
                  loclza.theta=LZACompas;
              }
              //clock_gettime(CLOCK_REALTIME, &TimeEstime);
              if(isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)){
                LZA_estime.pos.x=loclza.x;
                LZA_estime.pos.y=loclza.y;
                LZA_estime.dir.theta=loclza.theta;
                new_estime_flag=true;
                //Reset Simu
                Nloc=1;
                followSimState=true;
                //if(fabs(odopos.dir.theta-loclza.theta)>(Ttol)){
                //if((fabs(odopos.dir.theta-loclza.theta)>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)<M_PI)||
                  //(((2*M_PI)-fabs(odopos.dir.theta-loclza.theta))>(Ttol)&&fabs(odopos.dir.theta-loclza.theta)>M_PI)){
                  //oldOdosim.dir.theta=odopos.dir.theta;
                //}else{
                  oldOdosim.dir.theta=loclza.theta;
                //}
                oldOdosim.pos.x=loclza.x;
                oldOdosim.pos.y=loclza.y;
                LastLoc=oldOdosim;
                PSim=P;
              }else{//Simu en cours : Simu Echec : localisation Echec
                if(Fail_Count<Failtol){
                  Fail_Count++;
                }else{
                  //Echec de localisation : perdu
                  Nloc=0;
                  consfail++;
                  Fail_Count=0;
                  followSimState=false;
                  Loc_requis=true;
                }
              }
            }
          }
        }else{///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          lzamod.set_Fenetre(x1,y1,x2,y2);//forcage de la fenetre autour de la position odométrique.
          etat_robot.x_pos=odopos.pos.x;
          etat_robot.y_pos=odopos.pos.y;
          etat_robot.theta_pos=odopos.dir.theta;
          loclza=lzamod.Localise(datas_loc,4.0,etat_robot);

          if(isLZAResult(loclza,odopos,Ttol,Localised_once,Val_Recalage)&&((presentMode!=LZA_RUNING_COMMPASMODE && presentMode!=LZA_FUS_RUNING_COMMPASMODE)||fabs(LZACompas)<=4*M_PI)){
            if(!followSimState){
                Nloc=Nloc+1;
            }else{
              Nloc=1;
              followSimState=true;
              Fail_Count=0;
            }
          }else{
            if(Fail_Count<Failtol){
              Fail_Count++;
            }else{
              consfail++;
              Nloc=0;
              Fail_Count=0;
              followSimState=false;
              Loc_requis=true;
            }
          }
        }

          OdoPosOld=odopos;
          Toldodo=Todo;

          //Si les contition de relocalisation sont remplies:
          if((Loc_requis && Nloc>=Repet_Lost)||(!Loc_requis && Nloc>=Repet_maintain)){
            Localised_once=true;
            if(Monoparticular){
            //Set estimations
            Val_Recalage=OdoposSim;
            /*if(presentMode==LZA_RUNING_COMMPASMODE && LastCompas!=999){
              Val_Recalage.dir.theta=LastCompas;
            }/**/
            //RecalageLoc.Set(Val_Recalage, TimeEstime);
            TimeLocOld=TimeEstime;
            Reloc_Flag=1;
            LastLoc=EstimeInit;
            //Reset P
            P=PSim;
            //Set offset.
            OdoPosOld=Val_Recalage;
            Toldodo=TimeEstime;
            Loc_requis=false;
            Nloc=0;
            consfail=0;
            Fail_Count=0;
            RecalageLoc.Set(Val_Recalage, TimeEstime);
          }else{
            if((presentMode==LZA_RUNING_COMMPASMODE || presentMode==LZA_FUS_RUNING_COMMPASMODE) && fabs(LZACompas)<2*M_PI){
              loclza.theta=LZACompas;
            }
            Val_Recalage.pos.x=loclza.x;
            Val_Recalage.pos.y=loclza.y;
            Val_Recalage.dir.theta=loclza.theta;
            //Intersection des intervals entre la position odométrique reconstituée de l'incertitude de LZA
            //(double Iodo[6], int Mode, euclid_position LZARes,euclid_position OdoEstime,euclid_position &RelocVal)
            IOdo[0]=x2;
            IOdo[1]=x1;
            IOdo[2]=y2;
            IOdo[3]=y1;
            IOdo[4]=0;
            IOdo[5]=0;
            if(FusionLZA(IOdo,presentMode,Val_Recalage,odopos,Val_Recalage)){
              //RecalageLoc.Set(Val_Recalage, TimeEstime);
              //Ajou du décalage de 20cm en XS constaté dans le modèle d'erreur.
              Val_Recalage.pos.x=Val_Recalage.pos.x+0.2*cos(Val_Recalage.dir.theta);
              Val_Recalage.pos.y=Val_Recalage.pos.y+0.2*sin(Val_Recalage.dir.theta);
              Toldodo=TimeEstime;
              LastLoc=Val_Recalage;
              P=PLZA;
              Reloc_Flag=1;
              Loc_requis=false;
              Nloc=0;
              consfail=0;
              Fail_Count=0;
              OdoPosOld=Val_Recalage;
              Toldodo=TimeEstime;
              RecalageLoc.Set(Val_Recalage, TimeEstime);
              Total_Reloc++;
            }else{//on enlève le point de localisation qui avais permit le recalage et on ajoute simplement un echec.
              Nloc--;
              Fail_Count++;
            }
          }
          std::cout << "Recalage : X=" << Val_Recalage.pos.x << " : Y=" << Val_Recalage.pos.y << " : Theta=" << Val_Recalage.dir.theta << '\n';
          }
        }
        clock_gettime(CLOCK_REALTIME, &TimeLZA);
        break;
      default:
        break;
    }
    //Logging
    if(!(presentMode==LZA_IDLEMODE && presentModeCompas==COMPAS_IDLEMODE) && logneed && Logging){
      clock_gettime(CLOCK_REALTIME, &TimeLog);
      /*
      //Odopose
      Logs.push_back(OdoPosOld.pos.x);//--1
      Logs.push_back(OdoPosOld.pos.y);
      Logs.push_back(OdoPosOld.dir.theta);
      //Estimation LZA
      Logs.push_back(loclza.x);
      Logs.push_back(loclza.y);
      Logs.push_back(loclza.theta);
      //Fenetre
      Logs.push_back(x1);
      Logs.push_back(x2);
      Logs.push_back(y1);
      Logs.push_back(y2);
      //longueur fenetre --10
      Logs.push_back(abs(x1-x2));
      Logs.push_back(abs(y1-y2));
      //Nombre de localisation sucessive.
      Logs.push_back(Nloc);
      //Recalage
      Logs.push_back(Reloc_Flag);
      Logs.push_back(distLeft);
      Logs.push_back(distRight);

      //Odopose
      Logs.push_back(OdoposSim.pos.x);
      Logs.push_back(OdoposSim.pos.y);
      Logs.push_back(OdoposSim.dir.theta);
      //Fenetre -- 19
      Logs.push_back(x1sim);
      Logs.push_back(x2sim);
      Logs.push_back(y1sim);
      Logs.push_back(y2sim);
      //longueur fenetre
      Logs.push_back(abs(x1sim-x2sim));
      Logs.push_back(abs(y1sim-y2sim));
      //Recalage --25
      Logs.push_back(SleepReloc_Flag);

      Logs.push_back(Val_Recalage.pos.x);
      Logs.push_back(Val_Recalage.pos.y);
      Logs.push_back(Val_Recalage.dir.theta);
      Logs.push_back(Repet_Lost);
      Logs.push_back(Repet_maintain);
      if(Loc_requis){
        Logs.push_back(1);
      }else{
        Logs.push_back(0);
      }
      Logs.push_back(Nlocoffset); //--33
      if(followSimStateOld){
        Logs.push_back(1);
      }else{
        Logs.push_back(0);
      }
      if(followSimState){
        Logs.push_back(1);
      }else{
        Logs.push_back(0);
      }*/
      //////////////////////////////////////////////////////////////////////////////
      savefile<<(StartTime.tv_nsec+(StartTime.tv_sec*1000000000))<<","<<presentMode;
      savefile<<","<<(TimeFormat.tv_nsec+(TimeFormat.tv_sec*1000000000));
      savefile<<","<<(TimeLog.tv_nsec+(TimeLog.tv_sec*1000000000));
      savefile<<","<<OdoPosOld.pos.x;
      savefile<<","<<OdoPosOld.pos.y;
      savefile<<","<<OdoPosOld.dir.theta;
      savefile<<","<<(TimeLZA.tv_nsec+(TimeLZA.tv_sec*1000000000));
      savefile<<","<<LZArun;/////////////////////////////////////////////////////Ajout
      savefile<<","<<loclza.x;
      savefile<<","<<loclza.y;
      savefile<<","<<loclza.theta;
      savefile<<","<<x1;
      savefile<<","<<x2;//10
      savefile<<","<<y1;
      savefile<<","<<y2;
      savefile<<","<<fabs(x1-x2);
      savefile<<","<<fabs(y1-y2);
      savefile<<","<<Nloc;//15
      savefile<<","<<Fail_Count;
      savefile<<","<<Reloc_Flag;
      savefile<<","<<distLeft;
      savefile<<","<<distRight;
      savefile<<","<<OdoposSim.pos.x;//20
      savefile<<","<<OdoposSim.pos.y;
      savefile<<","<<OdoposSim.dir.theta;
      savefile<<","<<x1sim;
      savefile<<","<<x2sim;
      savefile<<","<<y1sim;//25
      savefile<<","<<y2sim;
      savefile<<","<<fabs(x1sim-x2sim);
      savefile<<","<<fabs(y1sim-y2sim);
      savefile<<","<<SleepReloc_Flag;
      savefile<<","<<Val_Recalage.pos.x;//30
      savefile<<","<<Val_Recalage.pos.y;
      savefile<<","<<Val_Recalage.dir.theta;
      if(Loc_requis){
        savefile<<","<<1;
      }else{
        savefile<<","<<0;
      }
      savefile<<","<<Nlocoffset; //--34
      if(followSimStateOld){//35
        savefile<<","<<1;
      }else{
        savefile<<","<<0;
      }
      if(followSimState){//36
        savefile<<","<<1;
      }else{
        savefile<<","<<0;
      }
      savefile<<","<<(TimeCompas.tv_nsec+(TimeCompas.tv_sec*1000000000));
      savefile<<","<<presentModeCompas;
      savefile<<","<<EstimeCompas;
      savefile<<","<<LastCompas;
      savefile<<","<<(TimeEstime.tv_nsec+(TimeEstime.tv_sec*1000000000));
      if(Heated){//41
        savefile<<","<<1;
      }else{
        savefile<<","<<0;
      }
      savefile<<","<<YRT;
      savefile<<"\n";

      logline=35;
      for (int j = 0; j < 57; j++) {
        LZA_LAS.push_back(datas_loc[j]);
      }
      LogTime.push_back(StartTime);
      LogMode.push_back(presentMode);
      if(new_estime_flag){//New LZA estimation for Kalman
        LZA_res.Set(LZA_estime,Time);
        new_estime_flag=false;
      }
    }

      //usleep(delayusleep(StartTime, 10));
      lsleep=delayusleep(StartTime, 100);
      if(lsleep<0){
        lsleep=0;
      }
      usleep(lsleep);
  }
  std::cout << "LZA_T : Sortie de la boucle" << '\n';
  if(Logging){
    /*std::string filename = PID_PATH("+PANORAMA_Log/LZA_LOG_t_"+std::to_string(LauncheTime.tv_sec)+".txt");
    std::ofstream savefile;
    savefile.open(filename,std::ios::out);
    std::cout << "LZA_T : logging LZA in "<<filename << '\n';
    for(int i=0; i<LogTime.size(); i++){
      savefile<<(LogTime[i].tv_nsec+(LogTime[i].tv_sec*1000000000))<<","<<LogMode[i];
      for(int j=0; j<logline; j++){
        savefile<<","<<Logs[i*logline+j];
      }
      savefile<<"\n";
    }*/
    savefile.close();

    std::string filenamelas = PID_PATH("+PANORAMA_Log/"+std::to_string(LauncheTime.tv_sec)+"/LZA_LAS_LOG.txt");
    FILE * savefile2=fopen(filenamelas.c_str(),"w");
    if (savefile2==NULL)
    {
      printf("Echec de l'ouverture du fichier de sauvegarde : %s\n",filenamelas.c_str());
      return;
    }

    std::cout << "LZA_T : logging LASin "<<filenamelas  << '\n';

    for(int i=0; i<LogTime.size(); i++){
      fprintf(savefile2,"%ld",(LogTime[i].tv_nsec+(LogTime[i].tv_sec*1000000000)));
      for(int j=0; j<57; j++){
        fprintf(savefile2,",%f",LZA_LAS[i*57+j]);
      }
      fprintf(savefile2,"\n");
    }
    fclose (savefile2);
  }

  std::cout << "LZA_T : Nombre de relocalisationtotal :"<< Total_Reloc<< '\n';
  std::cout << "LZA_T : END" << '\n';
}


void Format_LAS(laser_4m_record Las1, laser_4m_record Las2, double DIST[]){
  double datas_loc[57];
  double dist_las_robot_all[1024];
  double ang_las_robot_all[1024];

//  Telemetries=format_Telemetre(Las1.data,Las1.last_lenght,0.03,0,0);
//  Tmp_telemetries=format_Telemetre(Las2.data,Las2.last_lenght,0.03,0,M_PI);
//  Telemetries.insert(Telemetries.end(),Tmp_telemetries.begin(),Tmp_telemetries.end());

  for(int i=0;i<1024;i++)
  {
    if(i>=0 && i<256)//de -PI à -PI/2
    {
      dist_las_robot_all[i] = compute_value(Las2,340+i, 4.095);
      ang_las_robot_all[i] = Las2.data[340+i].dir.theta-M_PI;//i*(2*M_PI/1024); //
    }

    if(i>=256 && i<768)//de -PI/2 a +PI/2
    {
      dist_las_robot_all[i] = compute_value(Las1,84+i-256, 4.095);
      ang_las_robot_all[i] = Las1.data[84+i-256].dir.theta;//i*(2*M_PI/1024);  // ? voir comment c'est consid?r? dans la localisation
    }
    if(i>=768 && i<1024)
    {
      dist_las_robot_all[i] = compute_value(Las2, 84+i-768, 4.095);
      ang_las_robot_all[i] = Las2.data[84+i-768].dir.theta+M_PI;//-2*M_PI + i*(2*M_PI/1024);  // ? voir comment c'est consid?r? dans la localisation
    }
  }

//  std::cout << "Angle laser selectionnee = \n[";
  for(int j=0;j<57;j++){
      DIST[j]= dist_las_robot_all[j*18];
//      std::cout << ang_las_robot_all[j*18] << ' ';
      if (DIST[j] <= 0.06 || DIST[j] > 4)//4.095)
      {
        DIST[j] = 4;//.095;//portee max des lasers
      }
    }
//    std::cout << "]\n";
}

bool isLZAResult(localization::ROBLOC LZAres,euclid_position Odosource, double tetTol, bool Trusted,euclid_position LZArecal){
//  return !((LZAres.x==999 || LZAres.y==999 || LZAres.theta > 50)&&
//  (fabs(LZAres.theta-Odosource.dir.theta)<tetTol||fabs(LZAres.theta-Odosource.dir.theta)>(2*M_PI-tetTol)));
  if(LZAres.x==999){
    return false;
  }
  /*
  //Tentative de détection de recule trop important.
  double Ares=atan2(LZAres.y-Odosource.pos.y,LZAres.x-Odosource.pos.x);
  while(Ares>M_PI){
    Ares=Ares-2*M_PI;
  }
  while(Ares<=-1*M_PI){
    Ares=Ares+2*M_PI;
  }
  Ares=abs(Ares-Odosource.dir.theta);
  if(Ares>M_PI){
    Ares=(2*M_PI)-Ares;
  }
  if(sqrt(pow(LZAres.x-Odosource.pos.x,2)+pow(LZAres.y-Odosource.pos.y,2))>0.5 && Ares>7*M_PI/12){
    return false;
  }*/
  if(fabs(LZAres.theta-Odosource.dir.theta)<tetTol){
    return true;
  }
  if(fabs(2*M_PI-fabs(LZAres.theta-Odosource.dir.theta))<tetTol){
    return true;
  }
  return false;//!Trusted;//Si perdu on valide quand même si non on se méfie.
}


localization::ROBLOC MultiresLZA(localization::LZALocalization lzamod, double Las[57], double dist, P3D_Robot_State Rob_State){
  localization::ROBLOC locLZA;
  locLZA.x=999;
  locLZA.y==999;
  locLZA.theta=999;
/*  std::vector<LZACorel> Perf;
  LZACorel Perftmp;
  std::vector<localization::ROBLOC> vResLZA;
  std::vector<double> vResLZACorels;
  lzamod.ListLocalise(Las,dist,Rob_State,vResLZA,vResLZACorels);
  for(int j=0;j<vResLZACorels.size();++j){
    Perftmp.pos=vResLZA[j];
    Perftmp.corel=vResLZACorels[j];
    Perf.push_back(Perftmp);
  }
  sort(Perf.begin(),Perf.end(),CompLZA);
  for(int i=Perf.size()-1;i>=0;i--){
    if(fabs(Perf[i].pos.theta-Rob_State.theta_pos)<0.9*M_PI/2){
      locLZA.x=Perf[i].pos.x;
      locLZA.y=Perf[i].pos.y;
      locLZA.theta=Perf[i].pos.theta;
      break;
    }
  }*/
  return locLZA;
}

bool FusionLZA(double Iodo[6], int Mode, euclid_position LZARes,euclid_position OdoEstime,euclid_position &RelocVal){
  std::vector<double> ixLZA,iyLZA,itLZA,ixOrient,iyOrient,itOrient,ixInter,iyInter,itInter;
//  if(Mode==LZA_INITMODE || Mode==LZA_RUNINGMODE || Mode==LZA_INIT_COMPASMODE || Mode==LZA_RUNING_COMMPASMODE){
    RelocVal=LZARes;
    return true;
//  }
  if(Mode==LZA_FUS_INITMODE || Mode==LZA_FUS_RUNINGMODE ){
    ixLZA.push_back(-1.7474);
    ixLZA.push_back(0.90218);
    iyLZA.push_back(-0.28856);
    iyLZA.push_back(0.58067);
    itLZA.push_back(-0.13833);
    itLZA.push_back(0.19347);
  }else{
    if(Mode==LZA_FUS_INIT_COMPASMODE || Mode==LZA_FUS_RUNING_COMMPASMODE){
      ixLZA.push_back(-1.2913);
      ixLZA.push_back(0.10229);
      iyLZA.push_back(-0.26734);
      iyLZA.push_back(0.33004);
      itLZA.push_back(-0.12905);
      itLZA.push_back(0.10479);
    }
  }
  double ZonesMAP[][5]={{25,35,4,60,M_PI/2},{-5,24,-1,3,0},{24,35,-1,4,0}};
  //Si on ignore comment orienté l'erreur : On considéreur une erreur carré la plus grande possible
  ixOrient=Imul(initInterval(-1,1),ixLZA);
  iyOrient=Imul(initInterval(-1,1),iyLZA);
  ixOrient=Iunion(ixOrient,iyOrient);
  iyOrient=ixOrient;
  itOrient=Imul(initInterval(-1,1),itLZA);
  int Izone=-1;
  for(int i=0;i<3;i++){
    if(OdoEstime.pos.x>ZonesMAP[i][0]&&OdoEstime.pos.x<ZonesMAP[i][1]&&OdoEstime.pos.y>ZonesMAP[i][2]&&OdoEstime.pos.y<ZonesMAP[i][3]){
      Izone=i;
      break;
    }
  }
  switch (Izone) {
    case 0://Couloir 1
      if(fabs(ZonesMAP[Izone][4]-OdoEstime.dir.theta)<4*M_PI/12){//Retour
//        RoboDir=1;
        iyOrient=ixLZA;//L'arrière du robot est en Y negatif
        ixOrient=Imul(initInterval(-1,-1),iyLZA);
        itOrient=itLZA;
      }
      if(fabs(ZonesMAP[Izone][4]-OdoEstime.dir.theta)>8*M_PI/12){//Aller
//        RoboDir=2;
          iyOrient=Imul(initInterval(-1,-1),ixLZA); //L'arière du robot et en Y Positif
          ixOrient=iyLZA;
          itOrient=itLZA;
      }
    break;
    case 1://Couloir 2
      if(fabs(ZonesMAP[Izone][4]-OdoEstime.dir.theta)<4*M_PI/12){//Retour
//        RoboDir=1;
        iyOrient=iyLZA;//Condition d'établissement du model
        ixOrient=ixLZA;
        itOrient=itLZA;
      }
      if(fabs(ZonesMAP[Izone][4]-OdoEstime.dir.theta)>8*M_PI/12){//Aller
//        RoboDir=2;
        iyOrient=Imul(initInterval(-1,-1),iyLZA);//Le robot se déplace en sens invers de la conditon de création du modèle.
        ixOrient=Imul(initInterval(-1,-1),ixLZA);
        itOrient=itLZA;
      }
    break;
    default:
    break;
  }
  ixOrient=Isub(initInterval(LZARes.pos.x,LZARes.pos.x),ixOrient);
  iyOrient=Isub(initInterval(LZARes.pos.y,LZARes.pos.y),iyOrient);
  itOrient=Isub(initInterval(LZARes.dir.theta,LZARes.dir.theta),itOrient);
  ixInter=Iintersec(ixOrient,initInterval(Iodo[0],Iodo[1]));
  iyInter=Iintersec(iyOrient,initInterval(Iodo[2],Iodo[3]));
  itInter=Iintersec(itOrient,initInterval(Iodo[4],Iodo[5]));
  if(!IisEmpty(ixInter)&&!IisEmpty(iyInter)&&!IisEmpty(itInter)){
    RelocVal.pos.x=Imid(ixInter);
    RelocVal.pos.y=Imid(iyInter);
    RelocVal.dir.theta=Imid(itInter);
    return true;
  }
  return false;
}


bool isLZAusable(euclid_position odoposP3D){/*
  if(odoposP3D.pos.y>53.88)
    return false;
  if(odoposP3D.pos.y>47.88 && odoposP3D.pos.y<50.88)
    return false;
  if(odoposP3D.pos.x>22.38 && odoposP3D.pos.x<26.38)
    return false;
  if(odoposP3D.pos.x>13.38 && odoposP3D.pos.x<17.58)
    return false;*/
  return true;
}


double GetRTY(std::vector<struct polar_coordinate> Mesures){
  //std::vector<int> ransaclignefrompole(std::vector<struct polar_coordinate> source, double &a, double &b, double dtol , struct polar_coordinate &pref1, struct polar_coordinate &pref2);
  //double GetDistToLine(eucclid_coordinate p, double a, double b, eucclid_coordinate p1, eucclid_coordinate p2);
  std::vector<int> Iransac;
  double a,b,dtol;
  struct polar_coordinate pref1, pref2;
  eucclid_coordinate p0,p1,p2;
  std::vector<struct polar_coordinate> Tmp_telemetries;
  int ifind;

  p0.x=0;
  p0.y=0;
  p0.z=0;

  a=9000;
  Tmp_telemetries=Mesures;
//  std::cout << "Will get RTY" << '\n';
  while(a>15 && Tmp_telemetries.size()>(Mesures.size()*0.1)){
    for(int i=0; i<Iransac.size(); i++){
      ifind=Iransac.size()-1-i;
      Tmp_telemetries.erase(Tmp_telemetries.begin()+Iransac[ifind]);
    }
    Iransac=ransaclignefrompole(Tmp_telemetries, a, b, 0.01, pref1, pref2);
//    std::cout << "afind : "<< a << '\n';
  }
  if(a>15){
//    std::cout << "nofind return 999" << '\n';
    return 999;
  }
//  std::cout << "return estime" << '\n';
  return GetDistToLine(p0, a, b, ConvPolToEuclide(pref1), ConvPolToEuclide(pref2));

}
