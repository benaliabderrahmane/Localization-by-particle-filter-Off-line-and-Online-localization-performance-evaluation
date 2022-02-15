#include "Geometrie.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>
//#include <datas/sensor_data.hpp>
#include <datas/location_data.hpp>
#include <math.h>
#include <cmath>
#define _USE_MATH_DEFINES

/*renvois les coeeiscient ax+b de la droite corespondant au plus de points donnés en coordonnés polaires.*/
std::vector<int> ransaclignefrompole(std::vector<struct polar_coordinate> source, double &a, double &b, double dtol, struct polar_coordinate &pref1, struct polar_coordinate &pref2){
  double tmpa, tmpb, besta, bestb, tmpscore, bestscore, aperp, distpoint, obsa, obsb;
  int i1,i2,repet;
  bool vertical, Horizontal, valid;
  eucclid_coordinate p1, p2, pcomp, ptmp;
  std::vector<int> Ires, Itmps;
  repet=0;
  valid=false;
  bestscore=-1;
  if (source.size()==0){
    std::cout << "ERREUR RANSAC : Source vide." << '\n';
  }
  while(!valid){
    repet=repet+1;
    tmpscore=0;
    i1=rand() % source.size();
    i2=rand() % source.size();
    while(i1==i2){
      i2=rand() % source.size();
    }
    p1=ConvPolToEuclide(source[i1]);
    p2=ConvPolToEuclide(source[i2]);
    vertical=Computelignecoefs(p1,p2,obsa,obsb);
/*    Horizontal=false;
    if(obsa<1*10^(-9)){
      aperp=tan((atan(obsa)+M_PI/2));
    }else{
      Horizontal=true;
    }*/
    Itmps.clear();
    for(int i=0; i<source.size(); i++){
      pcomp=ConvPolToEuclide(source[i]);
      distpoint = GetDistToLine(pcomp, obsa, obsb, p1, p2);
      /*if(!Horizontal){
        if(!vertical){
          ptmp.x=pcomp.x+1;
          ptmp.y=pcomp.y+aperp;
          vertical=Computelignecoefs(pcomp,ptmp,tmpa,tmpb);
          ptmp.x=(obsb-tmpb)/(tmpa-obsa);
          ptmp.y=(ptmp.x*tmpa)+tmpb;
          distpoint=sqrt(pow((p1.x-ptmp.x),2)+pow((p1.y-ptmp.y),2));
        }else{
          distpoint=abs(pcomp.x-p1.x);
        }
      }else{
        distpoint=abs(pcomp.y-p1.y);
      }*/
      if(distpoint<dtol){
        tmpscore=tmpscore+1;
        Itmps.push_back(i);
      }else{
//        std::cout << "Distpoint : "<<distpoint<<" - " << std::flush ;
      }
    }
//    std::cout << "tmpscore="<<tmpscore<<" - Bestscore :"<<bestscore << '\n';
    if(tmpscore>bestscore){
      bestscore=tmpscore;
//      std::cout << "New Best scor : "<<bestscore<<"/"<< source.size() <<" A:"<<obsa<<" - B:"<<obsb << '\n';
      besta=obsa;
      bestb=obsb;
      pref1=source[i1];
      pref2=source[i2];
      Ires=Itmps;
    }
//    std::cout << "Nrepet : "<<repet << '\n';
    if(repet>100 || bestscore>(source.size()*0.95)){
//      std::cout << "Stop ransac with repet ="<< repet <<"and Best scor : "<<bestscore<<"/"<< source.size() << '\n';
      valid=true;
    }
  }
//  std::cout << "source size : "<< source.size() << '\n';
  a=besta;
  b=bestb;
  return Ires;
}

eucclid_coordinate ConvPolToEuclide(polar_coordinate pospol){
  eucclid_coordinate res;
  res.x=cos(pospol.dir.theta)*pospol.dist;
  res.y=sin(pospol.dir.theta)*pospol.dist;
  return res;
}

bool Computelignecoefs(eucclid_coordinate p1, eucclid_coordinate p2, double &a, double &b){
  bool non_vertical=true;
  if(p1.x==p2.x){
    a=tan(M_PI/2);
    b=0;
    non_vertical=false;
    return !non_vertical;
  }
/*  if(p1.x<p2.x){
    a=(p2.y-p1.y)/(p2.x-p1.x);
    b=p2.y-a*p2.x;
  }else{
    a=(p1.y-p2.y)/(p1.x-p2.x);
    b=p2.y-a*p2.x;
//  }*/
  a=(p1.y-p2.y)/(p1.x-p2.x);
  b=p2.y-a*p2.x;
  return !non_vertical;
}


double GetDistToLine(eucclid_coordinate p, double a, double b, eucclid_coordinate p1, eucclid_coordinate p2){
  eucclid_coordinate ptmp, pe1, pe2, plook;
  double aperp, bperp, tmpa, tmpb, distpoint;
  bool Horizontal = false;
  bool vertical = false;

//  if(a<10^(12)){
    aperp=tan((atan2(a,1)+M_PI/2));
//  }
//  std::cout << "diference angle A et Aperp ="<< (atan2(a,1)-atan2(aperp,1))<< '\n';
  if(fabs(a)<(pow(10,-6))){
    Horizontal=true;
  }
  if(fabs(a)>(pow(10,12))){
    vertical=true;
  }

  if(!Horizontal){
    if(!vertical){
      //un second point sur la droite suposé perpendiculaire ala courbe.
      ptmp.x=p.x+1;
      ptmp.y=p.y+aperp;
      vertical=Computelignecoefs(p,ptmp,tmpa,tmpb);
      if ((tmpa-a)==0){
        std::cout << "ERREUR DistToLigne : tmpa==a." << '\n';
      }
      ptmp.x=(b-tmpb)/(tmpa-a);
      ptmp.y=(ptmp.x*tmpa)+tmpb;
//      std::cout << "Intersection : X="<< ptmp.x << " Y="<<ptmp.y << " : "<<std::flush;
      distpoint=sqrt(pow((p.x-ptmp.x),2)+pow((p.y-ptmp.y),2));
    }else{
//      std::cout << "ligne verticale" << '\n';
      if (a==0){
        std::cout << "ERREUR DistToLigne : a=0 mais pas horyzontal?!" << '\n';
      }
      distpoint=fabs(p.x-(-b/a));
    }
  }else{
//    std::cout << "ligne Horizontal" << '\n';
    distpoint=fabs(p.y-(b));
  }
  return distpoint;
}


euclid_position TransphoInEuclide(euclid_position pos, double dx, double dy, double dtheta){
    euclid_position res;
    res.pos.x=cos(-dtheta)*pos.pos.x+sin(-dtheta)*pos.pos.y+dx;
    res.pos.y=-sin(-dtheta)*pos.pos.x+cos(-dtheta)*pos.pos.y+dy;
    res.pos.z=0;
    res.dir.theta=pos.dir.theta+dtheta;
    res.dir.phi=0;
    res.dir.psy=0;
    return res;
//    res.x=cos(pospol.dir.theta)*pospol.dist;
//    res.y=sin(pospol.dir.theta)*pospol.dist;
}

polar_coordinate ConvEuclideToPol(eucclid_coordinate poseuc){
  polar_coordinate res;
  res.dir.psy=0;
  res.dir.phi=0;
  res.dir.theta=atan2(poseuc.y,poseuc.x);
  res.dist=sqrt(pow(poseuc.x,2)+pow(poseuc.y,2));
//  std::cout << "Source X="<<poseuc.x<<" Y="<<poseuc.y<< " -- Res Dist="<<res.dist<<" - Theta="<<res.dir.theta << '\n';
  return res;
}
