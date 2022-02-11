#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

int main()
{
	string fname = "obstacles.csv";


	vector<vector<string>> obstacles;
	vector<string> row;
	string line, data;

	fstream file (fname, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();
			stringstream str(line);

			while(getline(str, data, ','))
				row.push_back(data);
			obstacles.push_back(row);
		}
	}
	else
		cout<<"Could not open the file\n";

	for(int i=0;i<obstacles.size();i++)
	{
		for(int j=0;j<obstacles[i].size();j++)
		{
			cout<<obstacles[i][j]<<" ";
		}
		cout<<"\n";
	}

	return 0;
}

