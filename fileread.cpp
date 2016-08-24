#include <fstream>
#include <vector>
#include <iostream>
using namespace std;
int main()
{
	ifstream is("input.dat", ifstream::binary);
    is.seekg (0, is.beg);
	// Create a vector to store the data
	char buffer[2];
    while(!is.eof())
    {	
        // Load the data
	    is.read((char *)&buffer, 3);
        cout << buffer[0] << "\n";
        cout << buffer[1] << "\n";
        cout << buffer[2] << "\n";
    }
	// Close the file
	is.close();
}
