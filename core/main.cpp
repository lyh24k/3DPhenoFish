#include "pre.h"

string getname(string path)
{
	string name;
	for (int i = path.size() - 5; i > 0; i--)
	{
		if (path[i] != '/')
		{
			name.push_back(path[i]);
		}
		else
			break;
	}
	string temp;
	for (int i = name.size() - 1; i >= 0; i--)
	{
		temp.push_back(name[i]);
	}
	return temp;
}

int main()
{
	return 0;
}