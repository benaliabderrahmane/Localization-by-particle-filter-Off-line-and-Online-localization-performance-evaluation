#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

#include <cstdint>
#include <cstring>
#include <cmath>


using namespace std;

double closest(vector<double> const& vec, double value)
{
    auto const it = lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }
    return distance(vec.begin(), it);
}


int main()
{


double n = nan("1");

double a = 0;

double res = a*n;

cout << res ;
return 0;
}
