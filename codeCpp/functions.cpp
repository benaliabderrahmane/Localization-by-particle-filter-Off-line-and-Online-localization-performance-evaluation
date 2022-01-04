#include "functions.h"

vector<double> polyxpoly(double x,double x1,double y,double y1,vector<vector<double>> grandObstacle)
{
    vector<double> intersection(5,5);
    return intersection;
}


bool isInBoxMax(double x,double y,Obstacles obstacle)
{
    // set random engine
    default_random_engine generator{static_cast<long unsigned int>(time(0))};
    uniform_real_distribution<double> distribution(0,1);

    // flag to be returned
    bool flag = false;

    //vector<vector<double>> grandObstacle(217*3, std::vector<double>(2, 0));
    vector<vector<double>> grandObstacle;
    for (int i = 1;i<=217;i++)
    {
        grandObstacle.push_back({obstacle.posVertex[i][0][0],obstacle.posVertex[i][0][1]});
        grandObstacle.push_back({obstacle.posVertex[i][1][0],obstacle.posVertex[i][1][1]});
        grandObstacle.push_back({nan("1"),nan("1")});
    }
    if (!isnan(x) && !isnan(y))
    {
        double x1[7];
        double y1[7];
        int success = 0;
        for(int j=0; j<7; j++)
        {
            double h = distribution(generator)*2*pi-pi;
            x1[j] = x+99999+cos(h);
            y1[j] = y+99999+sin(h);


            for (int kk=0; kk<7; kk++)
            {
                vector<double> uu;
                uu = polyxpoly(x,x1[kk],y,y1[kk],grandObstacle);
                if (uu.empty())
                {
                    success = 0;
                    break;
                }
                if (fmod(uu.size(),2)==1)
                //we may be inside something
                success++;
            }
        }
        if (success>3)
        {
            flag = true;
        }
    }
    return flag;
}



vector<vector<double>> particleGenerator(double xMin,double xMax,double yMin, double yMax, double thetaMin, double thetaMax, int N,Obstacles obstacle)
{
    // set random engine
    default_random_engine generator{static_cast<long unsigned int>(time(0))};
    uniform_real_distribution<double> distribution(0,1);

    vector<vector<double>> particles;
    vector<double> p(3,0);
    int i = 1;
while(i <= N)
    p[0] = (xMax-xMin)*distribution(generator)+xMin;//particle i x coordinate
    p[1] = (yMax-yMin)*distribution(generator)+yMin;//particle i y coordinate
    p[2] = (thetaMax-thetaMin)*distribution(generator)+thetaMin;//particle i theta coordinate
    bool test = isInBoxMax(p[0],p[1],obstacle);
    if (test)
    {
        particles.push_back(p);
        i++;
    }
    return particles;
}

double wrapAngle(double a)
{	
	if (a > M_PI){
		a = a -2*M_PI;
	}
	else if(a < -M_PI){
		a = a + 2*M_PI;
	}
	return a;	
}

double likelihood(vector<double> rho, vector<double> rho_particles)
{
	double var = 0;
	double size = rho.size();
	vector<double> diff(size);
	vector<double> E(size),ee(size);

	for( int k = 0; k != size; k++){
		diff[k] = rho[k] - rho_particles[k];
	}
	double sum = accumulate(diff.begin(), diff.end(), 0.0);
	double mean = sum / diff.size();
	for( int k = 0; k != size; k++){
		var += (diff[k] - mean) * (diff[k] - mean); 
	}
	var /= diff.size();
	double sigma = sqrt(var);
	for( int k = 0; k != E.size(); k++){
		E[k] = (diff[k] - mean)/sigma;
		ee[k] = E[k] * E[k];
	}
	
	double EE = accumulate(ee.begin(), ee.end(), 0.0);
	double weight = 1/(sqrt(2*M_PI)*sigma)*exp(-0.5*EE);
	cout<<weight<<endl;
	return weight;
}