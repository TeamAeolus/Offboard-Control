#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <typeinfo>

int main() {
    float a, b, c;
    std::ifstream f("input.dat", std::ios::binary);
    f.read((char*)&a, sizeof(float));
    f.read((char*)&b, sizeof(float));
    f.read((char*)&c, sizeof(float));
    std::cout << a << ", " << b << ", " << c << std::endl;
	
}
