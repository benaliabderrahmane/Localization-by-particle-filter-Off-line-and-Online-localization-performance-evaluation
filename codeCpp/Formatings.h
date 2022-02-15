#include <datas/sensor_data.hpp>
#include <datas/location_data.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <pfoa/path_finding.hpp>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#define ENTRE_ROUE 0.32;//0.328239;//0.3564514342;//0.33;//2*(0.2015-0.0474/2)/1.079;

float compute_value(laser_4m_record m1, int i, double limit);
std::vector<struct polar_coordinate> format_Telemetre(polar_coordinate Values[], int length, float offsetX, float OffsetY, float OffsetTheta);
std::vector<struct polar_coordinate> format_US(double US[]);
std::vector<pfoa::waypoint> format_Path(std::vector<struct eucclid_coordinate> Points);
void Recomput_Weeldist(euclid_position Oldstate,euclid_position R_state, double &ssg, double &ssd);
void SimuOdo(euclid_position StartPos,euclid_position &SimPos, double DistRG, double DistRD);
void compute_Fenetre_Error_Odo(euclid_position R_state, double distLeft, double distRight, Eigen::MatrixXd &P, double &x1, double &x2, double &y1, double &y2, double &t1, double &t2);
void compute_Fenetre_Error_Odo_Rob(euclid_position R_state, double distLeft, double distRight, Eigen::MatrixXd &P, eucclid_coordinate &PEShort,eucclid_coordinate &PELong);
//void RightLogs(std::vector<struct timespec> LogTime, std::vector<int> LogTime, std::vector<double> Logs, int lignelength, );
double Compute_ThetaCompas(std::vector<polar_coordinate> Telemetrie, euclid_position Odom);
std::vector<double> Iadd(std::vector<double> ia, std::vector<double>ib);
std::vector<double> Isub(std::vector<double> ia, std::vector<double>ib);
std::vector<double> Imul(std::vector<double> ia, std::vector<double>ib);
std::vector<double> Icos(std::vector<double> ia);
std::vector<double> Isin(std::vector<double> ia);
std::vector<double> Icentering(std::vector<double> ia);
std::vector<double> Iintersec(std::vector<double> ia, std::vector<double>ib);
std::vector<double> Iunion(std::vector<double> ia, std::vector<double>ib);
std::vector<double> initInterval(double a,double b);

double Imid(std::vector<double> ia);
double Ilength(std::vector<double> ia);
bool Iin(std::vector<double> ia,std::vector<double> ib);
void IVals(std::vector<double> ia,double &min, double &max);
bool IisEmpty(std::vector<double> ia);
