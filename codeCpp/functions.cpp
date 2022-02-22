#include "functions.h"


bool isInBoxMax(double x,double y,Obstacles Obstacles1,vector <vector<double>> GrandObstacle)
{
    // set random engine
    default_random_engine generator{static_cast<long unsigned int>(time(0))};
    uniform_real_distribution<double> distribution(0,1);

    // flag to be returned
    bool flag = false;


    if (!isnan(x) && !isnan(y))
    {
        double x1;
        double y1;
        int success = 0;
        for(int j=0; j<7; j++)
        {
            double h = distribution(generator)*2*M_PI-M_PI;
//            std::cout << "In box : H="<< h << '\n';
            x1 = x+99999*cos(h);
            y1 = y+99999*sin(h);


//            for (int kk=0; kk<7; kk++)
//            {
                auto uu = Intersection(x,x1,y,y1,GrandObstacle);

//                std::cout << "In box : NINTER="<< uu.size() << '\n';
                if (uu.empty())
                {
                    success = 0;
                    break;
                }
//                std::cout << "In box : modulo="<< fmod(uu.size(),2) << '\n';
                if (fmod(uu.size(),2)==1)
                //we may be inside something
                success++;
//            }
        }
        if (success>3)
        {
            flag = true;
        }
    }
    return flag;
}

vector<vector<double>> particleGenerator(double xMin,double xMax,double yMin, double yMax, double thetaMin, double thetaMax, int N,Obstacles obstacle,vector <vector<double>> GrandObstacle)
{
    // set random engine
    default_random_engine generator{static_cast<long unsigned int>(time(0))};
    uniform_real_distribution<double> distribution(0,1);

    vector<vector<double>> particles;

    int i = 1;
    while(i <= N)
    {
        vector<double> p(3,0);
        p[0] = (xMax-xMin)*distribution(generator)+xMin;//particle i x coordinate
        p[1] = (yMax-yMin)*distribution(generator)+yMin;//particle i y coordinate
        p[2] = (thetaMax-thetaMin)*distribution(generator)+thetaMin;//particle i theta coordinate
        bool test = isInBoxMax(p[0],p[1],obstacle,GrandObstacle);
        if (test)
        {
            particles.push_back(p);
//            std::cout << "PF: Cree une particule valide!! pos---" << '\n';
            i++;
        }else{
          std::cout << "Particule invalide a la position : "<< p[0]<< ", " << p[1]<<", " << p[2]<<", " << '\n';
        }
    }
    return particles;
}

double wrapAngle(double a)
{
	if (a > M_PI){
		a = a -2.0*M_PI;
	}
	else if(a < -M_PI){
		a = a + 2.0*M_PI;
	}
	return a;
}

double likelihood(vector<double> rho, vector<double> rho_particles)
{
	double var = 0.0;
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
	var /= (diff.size()-1);
	double sigma = sqrt(var);
	for( int k = 0; k != E.size(); k++){
		E[k] = (diff[k] - mean)/sigma;
		ee[k] = E[k] * E[k];
	}

	double EE = accumulate(ee.begin(), ee.end(), 0.0);
	double weight = 1/(sqrt(2*M_PI)*sigma)*exp(-0.5*EE);
	return weight;
}

int closest(vector<double> const& vec, double value)
{
    auto const it = lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }
    return distance(vec.begin(), it);
}

vector<double> selection(vector<double> weights,int N)
{
//SELECTION of next generation particles
//(used in main function ParticleFilter)
vector<double> iNextGeneration;

        //calculate the sum of the vector weights
        double sum;
        sum = std::accumulate(weights.begin(), weights.end(), 0.0);
        if(sum==0){
          return iNextGeneration;
        }

        //calculate the normalized Cumulative sum of the vector weights
        vector<double> NCS(N,0);
        int j = 0;
        for(vector<double>::iterator i = begin(weights); i != end(weights); ++i)
        {
            NCS[j]=accumulate(weights.begin(), i, 0.0)/sum;
//            NCS[j]=accumulate(weights.begin(), i, weights[0])/sum;
            j++;
        }

        //generate random number between 0 and 1 (generate random int between 0 and 10000 then divide by 10000 the distribution is uniform)
        default_random_engine generator{static_cast<long unsigned int>(time(0))};
        uniform_real_distribution<double> distribution(0,1);
        vector<double> iSelect;
        for (int i = 1; i<=N;i++)
            iSelect.push_back(distribution(generator));

        // interpolate to find the particle with nearest weight to the random generated weight and return its index

        int itmp;
        for (int i = 1; i<=N;i++){
//            iNextGeneration.push_back(closest(NCS, iSelect[i]));
            itmp=closest(NCS, iSelect[i]);
            while(itmp>=0&&weights[itmp]==0)
              itmp--;
            if(itmp<0)
              while(itmp<weights.size()&&weights[itmp]==0)
                itmp++;
            iNextGeneration.push_back(itmp);
        }

            /*
        //calculate the sum of the vector weights
        double sum;
        sum = std::accumulate(weights.begin(), weights.end(), 0);
        //calculate the normalized Cumulative sum of the vector weights
        vector<double> NCS(N,0);
        int j = 0;
        for(vector<int>::iterator i = begin(weights); i != end(weights); ++i)
        {
            NCS[j]=accumulate(weights.begin(), i, weights[0])/sum;
            j++;
        }
        //generate random number between 0 and 1 (generate random int between 0 and 10000 then divide by 10000 the distribution is uniform)
        default_random_engine generator{static_cast<long unsigned int>(time(0))};
        uniform_real_distribution<double> distribution(0,1);
        double randValue = distribution(generator);
        vector<double> iSelect;
        for (int i = 0; i<N;i++)
            iSelect.push_back(fmod (1.0/(N)*i+randValue,1));
        // interpolate to find the particle with nearest weight to the random generated weight and return its index
        vector<int> iNextGeneration;
        for (int i = 1; i<=N;i++)
            iNextGeneration.push_back(closest(NCS, iSelect[i]));*/

return iNextGeneration;
}


double var(vector<double> v){
    double variance = 0;
    double sum = accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    for( int k = 0; k != v.size(); k++){
        variance += (v[k] - mean) * (v[k] - mean);
        }
    variance /= v.size();
    return variance;
}
double coefficients(double sd[], int N)
{
    double k;
    double mean = (sd[0]+sd[1]+sd[2])/3;
    if (N>50)
    {
        if (mean > 5)
        {
            k=1;
        }
        else
        {
            k=0.8;
        }
    }
    else
    {
        k=0.8;
    }
    return k;
}

vector<vector<double>> testInext(vector<double> iNextGeneration, vector<vector<double>> Particles,  Obstacles Obstacles1,vector<vector<double>> GrandObstacle)
{
    vector<vector<double>>  Particles_new;
    vector<vector<double>>  Particles_IN;
    sort(iNextGeneration.begin(), iNextGeneration.end());
//    int N = 0;
    int cmpt=1;
    vector<vector<double>> Tab;
    vector<double> temp;
    double xmin, xmax, ymin, ymax, thetamin, thetamax;

    std::cout << "testInext : Inits" << '\n';
    for (int i=0;i<(iNextGeneration.size()-1);i++)
    {
        if (iNextGeneration[i]==iNextGeneration[i+1])
        {
            cmpt++;
        }
        else
        {
            temp.push_back(iNextGeneration[i]);
            temp.push_back(cmpt);
            Tab.push_back(temp);
            temp.clear();
            cmpt = 1;
        }
    }
    std::cout << "testInext : Inits2" << '\n';
    temp.push_back(iNextGeneration[iNextGeneration.size()-1]);
    temp.push_back(cmpt);
    Tab.push_back(temp);
    std::cout << "testInext : Inits3" << '\n';
    for (int i=0;i<Tab.size();i++)
    {
        xmin = Particles[Tab[i][0]][0]-0.25;
        xmax = Particles[Tab[i][0]][0]+0.25;
        ymin = Particles[Tab[i][0]][1]-0.25;
        ymax = Particles[Tab[i][0]][1]+0.25;
        thetamin = Particles[Tab[i][0]][2]-M_PI/8;
        thetamax = Particles[Tab[i][0]][2]+M_PI/8;
//        std::cout << "testInext : Will generate" << '\n';
        Particles_IN =particleGenerator(xmin,xmax,ymin,ymax,thetamin,thetamax,Tab[i][1],Obstacles1,GrandObstacle);
//        std::cout << "testInext : generate ok" << '\n';
        Particles_new.insert( Particles_new.end(), Particles_IN.begin(), Particles_IN.end() );
//        std::cout << "testInext : Particules added" << '\n';
//        N += Tab[i][1];
    }
    std::cout << "Particles_new size" << Particles_new.size() << '\n';
    return Particles_new;
}
/*
N_new=round(coefficients(Particles)*Tab(i,2));
Particles_new=[Particles_new,Particles_generator(xmin,xmax,ymin,ymax,theta_min,theta_max,N_new,Obstacles)];
NN=NN+N_new;
end
P_new.x=Particles_new(1,:);
P_new.y=Particles_new(2,:);
P_new.theta=Particles_new(3,:);
end
*/

void check_redistribution(double PoseEstime[], double OldParticles[], double Robot[], double OldRobot[], double SdX, double SdY, double SdTheta, bool &flag1, bool &flag2)
{
	double Distance_Particles = sqrt(pow( (PoseEstime[0] - OldParticles[0]) , 2) + pow( (PoseEstime[1] - OldParticles[1]) , 2));
	double Distance_robot = sqrt(pow( (Robot[0] - OldRobot[0]) , 2) + pow( (Robot[1] - OldRobot[1]) , 2));

	if((abs(Distance_Particles - Distance_robot) > 2.5)){
		flag1 = true;
		cout<<"flag 1:\t"<<"true"<<endl;
	}
	else{
		flag1 = false;
		cout<<"flag 1:\t"<<"false"<<endl;
	}
	if(SdX < 0.5 && SdY < 0.5 && SdTheta < 0.2){
		flag2 = true;
		cout<<"flag 2:\t"<<"true"<<endl;
	}
	else{
		flag2 = false;
		cout<<"flag 2:\t"<<"false"<<endl;
	}
}
void displayPoint(pdd P)
{
    cout << "(" << P.first << ", " << P.second << ")" << endl;
}

pdd Intersection_seg(pdd A, pdd B, pdd C, pdd D)
{
    //displayPoint(C);
    //displayPoint(D);
    // Line AB represented as a1x + b1y = c1
    double a1 = B.second - A.second;
    double b1 = A.first - B.first;
    double c1 = a1*(A.first) + b1*(A.second);
    // Line CD represented as a2x + b2y = c2
    double a2 = D.second - C.second;
    double b2 = C.first - D.first;
    double c2 = a2*(C.first)+ b2*(C.second);
    double determinant = a1*b2 - a2*b1;
    if (determinant == 0)
    {
        // The lines are parallel. This is simplified by returning a pair of FLT_MAX
        return make_pair(NAN, NAN);
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        if ( min(A.first, B.first)<=(x+0.001) && max(A.first, B.first)>=(x-0.001) &&
             min(C.first, D.first)<=(x+0.001) && max(C.first, D.first)>=(x-0.001) &&
             min(A.second, B.second)<=(y+0.001) && max(A.second, B.second)>=(y-0.001) &&
             min(C.second, D.second)<=(y+0.001) && max(C.second, D.second)>=(y-0.001) )
        {
            return make_pair(x, y);
        }
        else{
            return make_pair(NAN, NAN);
        }
    }
}

vector<pdd> Intersection(double x1, double x2, double y1, double y2, vector<vector<double>> map)
{
    //cout << x1 << "\t" << x2 << "\t" << y1 << "\t" << y2 << endl;
    int i,j;
	  pdd s1 = make_pair(x1, y1);
    pdd s2 = make_pair(x2, y2);
    //cout << "seg points:\t";
    //cout << "(" << s1.first << ", " << s1.second << ")" << "\t" << "(" << s2.first << ", " << s2.second << ")" << endl;
    //cout<<"------------------------------------"<<endl;
    //cout<<"------------------------------------"<<endl;
    pdd m1, m2;
    vector<pdd> v;
    for (i = 0; i< map.size()/2-1; i++)
    {
		m1 = make_pair(map[i*2][0], map[i*2][1]);
		m2 = make_pair(map[i*2+1][0], map[i*2+1][1]);
        //cout << "map points :\t"<<endl;
        //cout << "(" << m1.first << ", " << m1.second << ")" << "\t" << "(" << m2.first << ", " << m2.second << ")" << endl;
        //cout << "(" << s1.first << ", " << s1.second << ")" << "\t" << "(" << s2.first << ", " << s2.second << ")" << endl;
        pdd intersection;
        intersection = Intersection_seg(s1, s2, m1, m2);
        //cout << "intersection :\t" << endl;
        //cout<<"------------------------------------"<<endl;
        if(!isnan(intersection.first))
        v.push_back(intersection);
        //cout << v[i].first << "\t" << v[i].second << endl;
	}
    return v;
}

void uspatch_act(double Portee, vector<double> angles, vector<vector<double>> obstacle,
vector <double> distDetect, double x, double y, double theta,
vector<double> &DIST, vector<double> &ximp, vector<double> &yimp)
{
	vector<pdd> v;
	vector<double> h(angles.size());
	for(int i=0; i!=angles.size(); i++){
		h[i] = angles[i] + theta;
		if(h[i]<-M_PI){ h[i] += 2*M_PI;}
		else if(h[i]>M_PI){ h[i] -= 2*M_PI;}}

	vector <double> x1(angles.size()), y1(angles.size());
	for(int i=0; i!=h.size(); i++){
		x1[i]= x + Portee*cos(h[i]);
		y1[i]= y + Portee*sin(h[i]); }


	for(int kk=0; kk!= x1.size(); kk++){
  		DIST[kk] = Portee;
      ximp[kk] = x1[kk];
      yimp[kk] = y1[kk];

		v = Intersection(x, x1[kk], y, y1[kk], obstacle);
		//cout << v[0].first;
		//cout << endl << v[0].second;
		vector <double> d;
		for(int i=0; i!= v.size(); i++){
			if (!isnan(v[i].first) && !isnan(v[i].second))
			{d.push_back(pow((v[i].first-x), 2) + pow((v[i].second-y), 2));}
			else
			{d.push_back(Portee*Portee);}
    }
		if(d.size()!= 0)
		{
			double  a = *std::min_element(d.begin(), d.end()); //min
			int b = std::min_element(d.begin(),d.end()) - d.begin(); //min index
			DIST[kk] = sqrt(a);
			//cout << DIST[kk] << endl;
			/*if(sqrt(a) <= distDetect[b])
			{
				DIST[kk] = sqrt((pow((v[b].first-x),2)+pow((v[b].second-y),2)));
				//cout << DIST[kk] << endl;
			}*/
			if(!isnan(v[b].first) && !isnan(v[b].second))
			{	ximp[kk] = v[b].first;
				yimp[kk] = v[b].second;
      }
		}
	}
}


void Mesure_act(double Portee, vector<double> theta, vector<vector<double>> obstacle,
vector <double> distDetect, double xROB,double yROB,double thetaROB,
vector<double> &rho, vector<double> &ximp, vector<double> &yimp)
{
  /*
	for(int i=0; i!=theta.size(); i++)
	{
		theta[i] = round(theta[i]*(1022/(2*M_PI)))*(2*M_PI/1022);
	}
  */
	if(obstacle.size()==0){
		for(int i; i!=theta.size();i++){
			rho[i] = NAN;
			ximp[i] = NAN;
			yimp[i] = NAN;
		}
	}
	else{
		 uspatch_act(Portee, theta, obstacle, distDetect, xROB, yROB, thetaROB, rho, ximp, yimp);
	}

	for(int i=0; i<rho.size();i++){
		rho[i] = min(rho[i], Portee);
	}
}


std::vector<double> linspace(double start, double end, int num)
{

  std::vector<double> linspaced;


  if (num == 0) { return linspaced; }
  if (num == 1)
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
                            // in case start + delta * num is different then end
  return linspaced;
}
