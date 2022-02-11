#include <iostream>
#include <random>
#include <time.h>
#include <numeric>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>
#include <algorithm>

using namespace std;


int closest(vector<double> const& vec, double value)
{
    auto const it = lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }
    return distance(vec.begin(), it);
}

vector<int> selection(int choice,vector<double> weights,int N)
{
//SELECTION of next generation particles
//(used in main function ParticleFilter)


        //calculate the sum of the vector weights
        double sum;
        sum = std::accumulate(weights.begin(), weights.end(), 0);

        //calculate the normalized Cumulative sum of the vector weights
        vector<double> NCS(N,0);
        int j = 0;
        for(vector<double>::iterator i = begin(weights); i != end(weights); ++i)
        {
            NCS[j]=accumulate(weights.begin(), i, weights[0])/sum;
            j++;
        }

        //generate random number between 0 and 1 (generate random int between 0 and 10000 then divide by 10000 the distribution is uniform)
        default_random_engine generator{static_cast<long unsigned int>(time(0))};
        uniform_real_distribution<double> distribution(0,1);
        vector<double> iSelect;
        for (int i = 1; i<=N;i++)
            iSelect.push_back(distribution(generator));

        // interpolate to find the particle with nearest weight to the random generated weight and return its index
        vector<int> iNextGeneration;
        for (int i = 1; i<=N;i++)
            iNextGeneration.push_back(closest(NCS, iSelect[i]));


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

int main()
{
    return 0;
}


