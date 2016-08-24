#include <fstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <boost/lexical_cast.hpp>

using namespace std;
int main(int argc, char* argv[])
{
	
	ofstream os6("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os6.write((char*)"002", 3);
	os6.close();
	usleep(3000000);

	ofstream os("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
	//char* d1 = argv[3];
    os.write((char*)"202", 3);
	os.close();
	cout << "1" << "\n";
	unsigned int t1 = boost::lexical_cast<unsigned int>(argv[1]);
	usleep(t1);
	
	// Close the file

	ofstream os3("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
	//char* d2 = argv[4];
    os3.write((char*)"022", 3);
	os3.close();
	cout << "3" << "\n";
	unsigned int t2 = boost::lexical_cast<unsigned int>(argv[2]);
	usleep(t2);
	// Close the file

	ofstream os2("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os2.write((char*)"002", 3);
	os2.close();
	usleep(2000000);
	//cout << "2" << "\n";
	// Close the file
	//os2.close();
/*	
	ofstream os4("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os4.write((char*)"012", 3);
	os4.close();
	usleep(2500000);
	cout << "4" << "\n";
	// Close the file
	//os4.close();

	ofstream os7("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os7.write((char*)"102", 3);
	os7.close();
	usleep(3000000);
	cout << "1" << "\n";
	// Close the file

	ofstream os8("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os8.write((char*)"202", 3);
	os8.close();
	usleep(2500000);
	cout << "3" << "\n";
	// Close the file
	//os3.close();

	ofstream os9("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os9.write((char*)"012", 3);
	os9.close();
	usleep(3000000);
	cout << "2" << "\n";
	// Close the file
	//os2.close();
	
	ofstream os10("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os10.write((char*)"022", 3);
	os10.close();
	usleep(2500000);
	cout << "4" << "\n";
*/
	ofstream os5("input.dat", ofstream::binary | ofstream::trunc);
	// Create a vector to store the data
    os5.write((char*)"999", 3);
	os5.close();
	cout << "5" << "\n";
	// Close the file
	//os5.close();
}
