#include "Formatings.h"

#include <stdio.h>
#include <stdlib.h>
#include <datas/sensor_data.hpp>
#include <vector>
#include <cmath>
#include "Geometrie.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>


float compute_value(laser_4m_record m1, int i, double limit){
  if(0.02<m1.data[i].dist<limit){
    return m1.data[i].dist;
  }
  if (0.02<m1.data[i-1].dist<limit && 0.02<m1.data[i+1].dist<limit){
    return (m1.data[i-1].dist+m1.data[i+1].dist)/2;
  }
  //si on arrive ici (m1[1] >= limite && (m1[i+1]>=limite || m1[i-1]>=limite)
  //Sois les trois mesures sont a la limite sois une seul ne l'est pas (elle deviendra la "mesure reel")
  if(m1.data[i-1].dist<m1.data[i+1].dist && m1.data[i-1].dist>0.02){
    return m1.data[i-1].dist;
  }
  return m1.data[i+1].dist>0.02?m1.data[i+1].dist:m1.data[i].dist;
}

std::vector<struct polar_coordinate> format_Telemetre(polar_coordinate Values[], int length, float offsetX, float offsetY, float offsetTheta){
  double Xstart,Xend,Ystart,Yend;
  polar_coordinate tmpres={{0,0,0},0};
  std::vector<struct polar_coordinate> Res;
  for(int i=0;i<length;i++){
    Xstart=Values[i].dist*cos(Values[i].dir.theta);
    Ystart=Values[i].dist*sin(Values[i].dir.theta);
    Xend=cos(offsetTheta)*Xstart+sin(offsetTheta)*Ystart+offsetX;
    Yend=-1*sin(offsetTheta)*Xstart+cos(offsetTheta)*Ystart+offsetY;
    tmpres.dist=sqrt(Xend*Xend+Yend*Yend);
    tmpres.dir.theta=atan2(Yend,Xend);
    Res.push_back(tmpres);
  }
  return Res;
}

std::vector<struct polar_coordinate> format_US(double US[]){
  std::vector<struct polar_coordinate> Res;
  double Xstart,Xend,Ystart,Yend;
  eucclid_coordinate Relevee={0,0,0};
  polar_coordinate tmpres={{0,0,0},0};
  double Pas=(20*2*M_PI)/360;
  double Tright=-1*M_PI;
  double Tleft=M_PI;
  double Thetas_Deg_US[]={90,50,30,10,-10,-30,-50,-90,-90,-130,-150,-170,170,150,130,90};
//  double Thetas_US[16]={Tright,Tright+Pas,Tright+2*Pas,Tright+3*Pas,Tright+4*Pas,Tright+5*Pas,Tright+6*Pas,Tleft,Tleft,Tleft+Pas,Tleft+2*Pas,Tleft+3*Pas,Tleft+4*Pas,Tleft+5*Pas,Tleft+6*Pas,Tleft+7*Pas,Tright};
  for(int i=0;i<16;i++){
    tmpres.dist=US[i]+0.1;//Aproximation
    tmpres.dir.theta=(Thetas_Deg_US[i]/180)*M_PI;
    Relevee=ConvPolToEuclide(tmpres);
    if(i<8){//US Front
      //LES US se "rejoignes" environ 8CM en avant des roues.
      Relevee.x=Relevee.x+0.08;
    }else{//US rear
      //LES US se "rejoignes" environ 16CM en arrière des roues.
      Relevee.x=Relevee.x-0.16;
    }
    tmpres=ConvEuclideToPol(Relevee);
    Res.push_back(tmpres);
  }
  return Res;
}


std::vector<pfoa::waypoint> format_Path(std::vector<struct eucclid_coordinate> Points){
  double dist=0;
  std::vector<pfoa::waypoint> Path;
  eucclid_coordinate lastPoint=Points[0];
  for(int i=0;i<Points.size();i++){
    dist=dist+sqrt(pow(lastPoint.x-Points[i].x,2)+pow(lastPoint.y-Points[i].y,2));
    Path.push_back({Points[i].x,Points[i].y,dist});
    lastPoint=Points[i];
  }
}

void Recomput_Weeldist(euclid_position Oldstate,euclid_position R_state, double &ssg, double &ssd){
  double L=ENTRE_ROUE;
  float dx=R_state.pos.x-Oldstate.pos.x;
  float dy=R_state.pos.y-Oldstate.pos.y;
  float dt=R_state.dir.theta-Oldstate.dir.theta;
  float ds=sqrt(pow(dx,2)+pow(dy,2));
  ssg=(ds-(dt*L)/2);
  ssd=(dt*L)+ssg;
}

void SimuOdo(euclid_position Oldstate,euclid_position &R_state, double DistRG, double DistRD){
  double L=ENTRE_ROUE;
  float ds = (DistRG + DistRD)/2;
  float dt = (DistRD-DistRG)/L;
  R_state.pos.x=Oldstate.pos.x+ds*cos(Oldstate.dir.theta+dt/2);
  R_state.pos.y=Oldstate.pos.y+ds*sin(Oldstate.dir.theta+dt/2);
  R_state.dir.theta=Oldstate.dir.theta+dt;
}

void compute_Fenetre_Error_Odo(euclid_position R_state, double distLeft, double distRight, Eigen::MatrixXd &P, double &x1, double &x2, double &y1, double &y2, double &t1, double &t2){
  Eigen::MatrixXd OdoCov(2,2);
  OdoCov<< 0.001780754260856,0,0,0.001691065861758;
  Eigen::MatrixXd OdoCovtmp(2,2);
  Eigen::MatrixXd Dist(2,2);
  Dist<< 1.0,0.0,0.0,1.0;
  Eigen::MatrixXd Jacobrob(3,3);
  Jacobrob << 1,0,0,0,1,0,0,0,1;
  Eigen::MatrixXd Jacobdist(3,2);
  Eigen::MatrixXd Pstart(3,3);
  Pstart=P;

  double Lodo=ENTRE_ROUE;
  double ds,dt;
  double demicos,demisin;
  int iex, iey, iet;
  double Ex, Ey, OrientEy, ExSim, EySim, OrientEySim;
  double ext1, ext2, eyt1, eyt2;
  euclid_position OdoEstime;

  //Compute odometries errors.
  Dist(0,0)=distRight;
  Dist(1,1)=distLeft;
  OdoCovtmp = OdoCov*Dist;
  ds=(distRight+distLeft)/2;
  dt=(distRight-distLeft)/Lodo;
  Jacobrob(0,2)=-1*ds*sin(R_state.dir.theta+dt/2);
  Jacobrob(1,2)=ds*cos(R_state.dir.theta+dt/2);

  demicos=(cos(R_state.dir.theta+dt/2))/2;
  demisin=(sin(R_state.dir.theta+dt/2))/2;

  Jacobdist(0,0)=demicos-(ds/Lodo)*demisin;
  Jacobdist(0,1)=demicos+(ds/Lodo)*demisin;
  Jacobdist(1,0)=demisin+(ds/Lodo)*demicos;
  Jacobdist(1,1)=demisin-(ds/Lodo)*demicos;
  Jacobdist(2,0)=1/Lodo;
  Jacobdist(2,1)=-1/Lodo;

  P=Jacobrob*Pstart*Jacobrob.transpose()+Jacobdist*OdoCovtmp*Jacobdist.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> VecpropSolvP(P);
  Eigen::MatrixXd ValsProps=VecpropSolvP.pseudoEigenvalueMatrix();
  Eigen::MatrixXd VecsProps=VecpropSolvP.pseudoEigenvectors();
  //Ey = val prop la plus grande puis Ex puis Et

  if(ValsProps(0,0)>ValsProps(1,1)){//0 sup a 1
    if(ValsProps(2,2)>ValsProps(0,0)){//2-0-1
      iex = 0;
      iey = 2;
      iet = 1;
    }
    else if(ValsProps(2,2)>ValsProps(1,1)){//0-2-1
      iex = 2;
      iey = 0;
      iet = 1;
    }else{//0-1-2
      iex = 1;
      iey = 0;
      iet = 2;
    }
  }else{//1 sup a 0
    if(ValsProps(2,2)>ValsProps(1,1)){//2-1-0
      iex = 1;
      iey = 2;
      iet = 0;
    }
    else if(ValsProps(2,2)>ValsProps(0,0)){//1-2-0
      iex = 2;
      iey = 1;
      iet = 0;
    }else{//1-0-2
      iex = 0;
      iey = 1;
      iet = 2;
    }
  }
  Ex = pow(ValsProps(iex,iex),(1/3.0));
  Ey = pow(ValsProps(iey,iey),(1/3.0));
  OrientEy = atan2(VecsProps(1,iey),VecsProps(2,iey));
  SimuOdo(R_state,OdoEstime,distLeft,distRight);
  ext1=abs(Ex*cos(R_state.dir.theta+OrientEy-M_PI/2));
  eyt1=abs(Ex*sin(R_state.dir.theta+OrientEy-M_PI/2));
  ext2=abs(Ey*cos(R_state.dir.theta+OrientEy));
  eyt2=abs(Ey*sin(R_state.dir.theta+OrientEy));
//  ext1=max(ext1,ext2);
  if(ext2>ext1){
    ext1=ext2;
  }
//  eyt1=max(eyt1,eyt2);
  if(eyt2>eyt1){
    eyt1=eyt2;
  }
  x1=OdoEstime.pos.x+ext1;
  x2=OdoEstime.pos.x-ext1;
  y1=OdoEstime.pos.y+eyt1;
  y2=OdoEstime.pos.y-eyt1;
  t2=R_state.dir.theta+dt-pow(fabs(ValsProps(iet,iet)),(1/3.0));
  t1=R_state.dir.theta+dt+pow(fabs(ValsProps(iet,iet)),(1/3.0));

}


void compute_Fenetre_Error_Odo_Rob(euclid_position R_state, double distLeft, double distRight, Eigen::MatrixXd &P, eucclid_coordinate &PEShort,eucclid_coordinate &PELong){
  /*Renvois les coordonees des points des axes representant l'elipse d'incertitude.*/
  Eigen::MatrixXd OdoCov(2,2);
  OdoCov<< 0.001780754260856,0,0,0.001691065861758;
  Eigen::MatrixXd OdoCovtmp(2,2);
  Eigen::MatrixXd Dist(2,2);
  Dist<< 1.0,0.0,0.0,1.0;
  Eigen::MatrixXd Jacobrob(3,3);
  Jacobrob << 1,0,0,0,1,0,0,0,1;
  Eigen::MatrixXd Jacobdist(3,2);
  Eigen::MatrixXd Pstart(3,3);
  Pstart=P;

  double Lodo=ENTRE_ROUE;
  double ds,dt;
  double demicos,demisin;
  int iex, iey, iet;
  double Ex, Ey, OrientEy, ExSim, EySim, OrientEySim;
  double ext1, ext2, eyt1, eyt2;
  euclid_position OdoEstime;

  //Compute odometries errors.
  Dist(0,0)=distRight;
  Dist(1,1)=distLeft;
  OdoCovtmp = OdoCov*Dist;
  ds=(distRight+distLeft)/2;
  dt=(distRight-distLeft)/Lodo;
  Jacobrob(0,2)=-1*ds*sin(R_state.dir.theta+dt/2);
  Jacobrob(1,2)=ds*cos(R_state.dir.theta+dt/2);

  demicos=(cos(R_state.dir.theta+dt/2))/2;
  demisin=(sin(R_state.dir.theta+dt/2))/2;

  Jacobdist(0,0)=demicos-(ds/Lodo)*demisin;
  Jacobdist(0,1)=demicos+(ds/Lodo)*demisin;
  Jacobdist(1,0)=demisin+(ds/Lodo)*demicos;
  Jacobdist(1,1)=demisin-(ds/Lodo)*demicos;
  Jacobdist(2,0)=1/Lodo;
  Jacobdist(2,1)=-1/Lodo;

  P=Jacobrob*Pstart*Jacobrob.transpose()+Jacobdist*OdoCovtmp*Jacobdist.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> VecpropSolvP(P);
  Eigen::MatrixXd ValsProps=VecpropSolvP.pseudoEigenvalueMatrix();
  Eigen::MatrixXd VecsProps=VecpropSolvP.pseudoEigenvectors();
  //Ey = val prop la plus grande puis Ex puis Et

  if(ValsProps(0,0)>ValsProps(1,1)){//0 sup a 1
    if(ValsProps(2,2)>ValsProps(0,0)){//2-0-1
      iex = 0;
      iey = 2;
      iet = 1;
    }
    else if(ValsProps(2,2)>ValsProps(1,1)){//0-2-1
      iex = 2;
      iey = 0;
      iet = 1;
    }else{//0-1-2
      iex = 1;
      iey = 0;
      iet = 2;
    }
  }else{//1 sup a 0
    if(ValsProps(2,2)>ValsProps(1,1)){//2-1-0
      iex = 1;
      iey = 2;
      iet = 0;
    }
    else if(ValsProps(2,2)>ValsProps(0,0)){//1-2-0
      iex = 2;
      iey = 1;
      iet = 0;
    }else{//1-0-2
      iex = 0;
      iey = 1;
      iet = 2;
    }
  }
  Ex = pow(ValsProps(iex,iex),(1/3.0));
  Ey = pow(ValsProps(iey,iey),(1/3.0));
  OrientEy = atan2(VecsProps(1,iey),VecsProps(2,iey));
//  SimuOdo(R_state,OdoEstime,distLeft,distRight);
  ext1=abs(Ex*cos(OrientEy-M_PI/2));
  eyt1=abs(Ex*sin(OrientEy-M_PI/2));
  ext2=abs(Ey*cos(OrientEy));
  eyt2=abs(Ey*sin(OrientEy));

  PEShort.x=ext1;
  PEShort.y=ext1;
//  PEShort.theta=R_state.dir.theta+dt-pow(ValsProps(iet,iet),(1/3.0));
  PELong.x=ext2;
  PELong.y=ext2;
//  PELong.theta=R_state.dir.theta+dt+pow(ValsProps(iet,iet),(1/3.0));

}


double Compute_ThetaCompas(std::vector<polar_coordinate> Telemetrie, euclid_position Odom){
  std::vector<polar_coordinate> Tmp_telemetries;
  std::vector<int> iRans;
  std::vector<double> As, difAs;
  std::vector<double> WAs, WdifAs;

  double Coridor_orient[][5]={{4.88,26.38,-1,2.5,0},{23,30,2.88,50.88,M_PI/2}};
  int Coridor_Num=2;
  int RoboDir, PresentCoridor;
  int ifind;
  double awall, bwall, Angletmp;
  double Angle_Compas;
  struct polar_coordinate P1,P2;
  iRans.clear();
  As.clear();
  WAs.clear();
  awall=2000;
  Angle_Compas=999;

  //Retrais des points pouvant être trop eloignee
  for(int i=0; i<Telemetrie.size(); i++){
    if(Telemetrie[i].dist<3.5 && Telemetrie[i].dist>0.05){
      Tmp_telemetries.push_back(Telemetrie[i]);
    }
  }
  //Recuperation de l'orientation principale a cet endroit dans le reper monde.
  PresentCoridor=-1;
  RoboDir=-1;
  for(int i=0; i<Coridor_Num; i++){
    //Si l'odometrie se pense dans une zone d'un couloir
    if(Odom.pos.x>Coridor_orient[i][0] && Odom.pos.x<Coridor_orient[i][1] && Odom.pos.y>Coridor_orient[i][2] && Odom.pos.y<Coridor_orient[i][3]){
      PresentCoridor=i;
      break;
    }
  }
/*
  std::cout << "Present Coridor = "<< PresentCoridor  << '\n';
  std::cout << "Info Coridor = "<< Coridor_orient[PresentCoridor][4] << " - Odo theta = "<< Odom.dir.theta << '\n';
  std::cout << " - Fabs observe => "<< fabs(Coridor_orient[PresentCoridor][4]-Odom.dir.theta) << '\n';/**/
  //On determine le sens de deplacement du robot.
  if(PresentCoridor!=-1){
    if(fabs(Coridor_orient[PresentCoridor][4]-Odom.dir.theta)<4*M_PI/12){//Retour
      RoboDir=1;
    }
    if(fabs(Coridor_orient[PresentCoridor][4]-Odom.dir.theta)>8*M_PI/12){//Aller
      RoboDir=2;
    }
  }
//  std::cout << "ROBODIR = "<< RoboDir << '\n';
  if(Telemetrie.size()!=0 && RoboDir!=-1){
    while(As.size()<5 && Tmp_telemetries.size()>Telemetrie.size()*0.1){
      for(int i=0; i<iRans.size(); i++){
        ifind=iRans.size()-1-i;
        Tmp_telemetries.erase(Tmp_telemetries.begin()+iRans[ifind]);
      }
      iRans=ransaclignefrompole(Tmp_telemetries, awall, bwall, 0.01, P1, P2);
      As.push_back(awall);
      WAs.push_back(iRans.size());
//      std::cout << "New A = "<< awall << " - Poid = "<< iRans.size() << '\n';
    }
    difAs.clear();
    WdifAs.clear();
    //On met la plus grosse ligne en premier parmis les possibilités.
    difAs.push_back(As[0]);
    WdifAs.push_back(WAs[0]);
    for(int i=1; i<As.size(); i++){
      awall=0;
      for(int j=0; i<difAs.size(); j++){
        Angletmp=fabs(atan2(difAs[j],1)-atan2(As[i],1));
        while(Angletmp>=M_PI){
          Angletmp=Angletmp-M_PI;
        }
        if(Angletmp<M_PI/6 && Angletmp>5*M_PI/6){//Le mur est paralèle a +/-pi/4
          WdifAs[j]=WdifAs[j]+WAs[i];
          awall=1;
          break;
        }
      }
      if(awall!=1){//Pas de droite "paralèle" trouvé. on cré un nouveau groupe. avec la droite ayant le plus de "membres"
        difAs.push_back(As[i]);
        WdifAs.push_back(WAs[i]);
      }
    }
    //Recuperation de l'orientation dont le poid des paralèle est le plus important
    bwall=0;
    for(int i=0; i<WdifAs.size(); i++){
      if(WdifAs[i]>bwall){
        bwall=WdifAs[i];
        awall=difAs[i];
      }
    }
//    std::cout << "awall = "<< awall<< " - Atan awall = "<< atan2(awall,1) << '\n';
    if(RoboDir==1){ //retour
      Angle_Compas=-1*(atan2(awall,1))+Coridor_orient[PresentCoridor][4];
    }
    if(RoboDir==2){ //aller
      Angle_Compas=-1*(atan2(awall,1))+(Coridor_orient[PresentCoridor][4]-M_PI);
    }
  }
  return Angle_Compas;
}

std::vector<double> Iadd(std::vector<double> ia, std::vector<double>ib){
  std::vector<double> res;
  if (IisEmpty(ia)){
    return res;
  }
  if (IisEmpty(ib)){
    return res;
  }
  double mina, maxa, minb, maxb;
  mina=*min_element(ia.begin(), ia.end());
  maxa=*max_element(ia.begin(), ia.end());
  minb=*min_element(ib.begin(), ib.end());
  maxb=*max_element(ib.begin(), ib.end());
  res.reserve(2);
  res.push_back(mina+minb);
  res.push_back(maxa+maxb);
  return res;
}
std::vector<double> Isub(std::vector<double> ia, std::vector<double>ib){
    std::vector<double> res;
    if (IisEmpty(ia)){
      return res;
    }
    if (IisEmpty(ib)){
      return res;
    }
    double mina, maxa, minb, maxb;
    mina=*min_element(ia.begin(), ia.end());
    maxa=*max_element(ia.begin(), ia.end());
    minb=*min_element(ib.begin(), ib.end());
    maxb=*max_element(ib.begin(), ib.end());
    res.reserve(2);
    res.push_back(mina-maxb);
    res.push_back(maxa-minb);
    return res;
}
std::vector<double> Imul(std::vector<double> ia, std::vector<double>ib){
  std::vector<double> res;
  if (IisEmpty(ia)){
    return res;
  }
  if (IisEmpty(ib)){
    return res;
  }
  std::vector<double> tmp(4);
  double mina, maxa, minb, maxb;
  mina=*min_element(ia.begin(), ia.end());
  maxa=*max_element(ia.begin(), ia.end());
  minb=*min_element(ib.begin(), ib.end());
  maxb=*max_element(ib.begin(), ib.end());
  tmp[0]=mina*minb;
  tmp[1]=mina*maxb;
  tmp[2]=maxa*minb;
  tmp[3]=maxa*maxb;
  res.reserve(2);
  res.push_back(*min_element(tmp.begin(), tmp.end()));
  res.push_back(*max_element(tmp.begin(), tmp.end()));
  return res;
}
std::vector<double> Icos(std::vector<double> ia){
  std::vector<double> res, tmp(2);
  if (IisEmpty(ia)){
    return res;
  }
  double mina, maxa, minb, maxb;
  tmp[0]=*min_element(ia.begin(), ia.end())+M_PI/2;
  tmp[1]=*max_element(ia.begin(), ia.end())+M_PI/2;
  res=Isin(tmp);
  return res;
}
std::vector<double> Isin(std::vector<double> ia){
  std::vector<double> res,tmp(2),rtmp,Snum(1);
  if (IisEmpty(ia)){
    return res;
  }
  double mina, maxa, minb, maxb;
  tmp[0]=*min_element(ia.begin(), ia.end());
  tmp[1]=*max_element(ia.begin(), ia.end());
  if(Ilength(tmp)>2*M_PI){
    res.reserve(2);
    res.push_back(-1);
    res.push_back(1);
    return res;
  }
  while(tmp[1]>2*M_PI){
    tmp[0]=tmp[0]-2*M_PI;
    tmp[1]=tmp[1]-2*M_PI;
  }
  while(tmp[0]<0){
    tmp[0]=tmp[0]+2*M_PI;
    tmp[1]=tmp[1]+2*M_PI;
  }
  rtmp.push_back(sin(tmp[0]));
  rtmp.push_back(sin(tmp[1]));
  Snum[0]=M_PI/2;
  if(Iin(Snum,tmp)==true){
    rtmp.push_back(1);
  }
  Snum[0]=3*M_PI/2;
  if(Iin(Snum,tmp)==true){
    rtmp.push_back(-1);
  }
  res.reserve(2);
  res.push_back(*min_element(rtmp.begin(), rtmp.end()));
  res.push_back(*max_element(rtmp.begin(), rtmp.end()));
  return res;
}

double Imid(std::vector<double> ia){
    double res;
    double mina, maxa;
    mina=*min_element(ia.begin(), ia.end());
    maxa=*max_element(ia.begin(), ia.end());
    res=mina+(maxa-mina)/2;
    return res;
}

double Ilength(std::vector<double> ia){
  double res;
  double mina, maxa;
  if (IisEmpty(ia)){
    return -1;
  }
  mina=*min_element(ia.begin(), ia.end());
  maxa=*max_element(ia.begin(), ia.end());
  res=maxa-mina;
  return res;
}

bool Iin(std::vector<double> ia,std::vector<double> ib){
  if (IisEmpty(ia)||IisEmpty(ib)){
    return false;
  }
  double mina, maxa, minb, maxb;
  mina=*min_element(ia.begin(), ia.end());
  maxa=*max_element(ia.begin(), ia.end());
  minb=*min_element(ib.begin(), ib.end());
  maxb=*max_element(ib.begin(), ib.end());
  return ((mina>=minb)&&(maxa<=maxb));
}

void IVals(std::vector<double> ia,double &min, double &max){
  if (IisEmpty(ia)){
    return;
  }
  min=*min_element(ia.begin(), ia.end());
  max=*max_element(ia.begin(), ia.end());
  return;
}

std::vector<double> initInterval(double a,double b){
  std::vector<double> res={a,b};
  return res;
}


std::vector<double> Icentering(std::vector<double> ia){
  std::vector<double> res;
  if (IisEmpty(ia)){
    return res;
  }
//  double mina, maxa, minb, maxb;
  res.reserve(2);
  res.push_back(*min_element(ia.begin(), ia.end())-Imid(ia));
  res.push_back(*max_element(ia.begin(), ia.end())-Imid(ia));
  return res;
}

std::vector<double> Iintersec(std::vector<double> ia, std::vector<double>ib){
  std::vector<double> res;
  if (IisEmpty(ia)){
    return res;
  }
  if (IisEmpty(ib)){
    return res;
  }
  double mina, maxa, minb, maxb;
  mina=*min_element(ia.begin(), ia.end());
  maxa=*max_element(ia.begin(), ia.end());
  minb=*min_element(ib.begin(), ib.end());
  maxb=*max_element(ib.begin(), ib.end());
  if(!(maxa < minb || mina > maxb)){
    res.reserve(2);
    res.push_back(mina*(mina>minb)+minb*!(mina>minb));
    res.push_back(maxa*(maxa<maxb)+maxb*!(maxa<maxb));
  }
  return res;
}
std::vector<double> Iunion(std::vector<double> ia, std::vector<double>ib){
  std::vector<double> res;
  double mina, maxa, minb, maxb;
  if (IisEmpty(ia)){
    return ib;
  }
  if (IisEmpty(ib)){
    return ia;
  }
  mina=*min_element(ia.begin(), ia.end());
  maxa=*max_element(ia.begin(), ia.end());
  minb=*min_element(ib.begin(), ib.end());
  maxb=*max_element(ib.begin(), ib.end());
  res.reserve(2);
  res.push_back(mina*(mina<minb)+minb*!(mina<minb));
  res.push_back(maxa*(maxa>maxb)+maxb*!(maxa>maxb));
  return res;
}

bool IisEmpty(std::vector<double> ia){
  return ia.size()==0;
}
