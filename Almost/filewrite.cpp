#include <fstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <boost/lexical_cast.hpp>

using namespace std;
int main(int argc, char* argv[])
{	
	float x = 0.0f, y = 0.0f, z = 1.5f;
	unsigned int t1;

	if(argc > 1)
		t1 = boost::lexical_cast<unsigned int>(argv[1]);
	else
		t1 = 3000000;

	ofstream os("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os.write((const char*)&x, sizeof(float));
    os.write((const char*)&y, sizeof(float));
    os.write((const char*)&z, sizeof(float));
	os.close();
	usleep(3000000);
	
	x = 1.0f; y = -1.0f; z = 1.5f;
	os.open("input.dat", ofstream::binary | ofstream::trunc);
	//char* d1 = argv[3];
	os.write((const char*)&x, sizeof(float));
    os.write((const char*)&y, sizeof(float));
    os.write((const char*)&z, sizeof(float));
	os.close();
	cout << "1" << "\n";
	usleep(t1);

	x = -1.0f; y = 2.0f ; z = 1.5f;
	os.open("input.dat", ofstream::binary | ofstream::trunc);
	//char* d1 = argv[3];
	os.write((const char*)&x, sizeof(float));
    os.write((const char*)&y, sizeof(float));
    os.write((const char*)&z, sizeof(float));
	os.close();
	cout << "2" << "\n";
	usleep(t1);
	
	x = 0.0f; y = 0.0f; z = 0.0f;
	os.open("input.dat", ofstream::binary | ofstream::trunc);
	//char* d1 = argv[3];
	os.write((const char*)&x, sizeof(float));
    os.write((const char*)&y, sizeof(float));
    os.write((const char*)&z, sizeof(float));
	os.close();
	cout << "0" << "\n";
}
