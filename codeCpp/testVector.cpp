
// C++ program to illustrate the
// iterators in vector
#include <iostream>
#include <vector>
#include <numeric>
#include <time.h>
#include <random>


using namespace std;


int main()
{

default_random_engine generator{static_cast<long unsigned int>(time(0))};
uniform_real_distribution<double> distribution(0,1);


    double s = 0;
    double e = 1;
    int n = 9;

    vector<double> v;
    double randd = distribution(generator);
    cout << "rand " << randd << endl ;
    for (int i=0;i<n;i++)
        v.push_back(fmod(1.0/(n)*i+randd,1));

    for (auto i = v.begin(); i != v.end(); ++i)
        cout << *i << " ";



    /*
default_random_engine generator;
uniform_int_distribution<int> distribution(0,10000);
vector<double> iSelect;
    for (int i = 1; i<=10;i++)
    {
                iSelect.push_back(distribution(generator)/10000.0);
    }


    for (auto i = iSelect.begin(); i != iSelect.end(); ++i)
        cout << *i << " ";


*/
/*
    for (int i = 1; i <= 5; i++)
        g1.push_back(i);

    cout << "Output of begin and end: ";
    for (auto i = g1.begin(); i != g1.end(); ++i)
        cout << *i << " ";

    cout << "\nOutput of cbegin and cend: ";
    for (auto i = g1.cbegin(); i != g1.cend(); ++i)
        cout << *i << " ";

    cout << "\nOutput of rbegin and rend: ";
    for (auto ir = g1.rbegin(); ir != g1.rend(); ++ir)
        cout << *ir << " ";

    cout << "\nOutput of crbegin and crend : ";
    for (auto ir = g1.crbegin(); ir != g1.crend(); ++ir)
        cout << *ir << " ";
*/
    return 0;
}
