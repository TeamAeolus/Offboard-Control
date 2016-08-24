#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <typeinfo>
void writeTheFile()
{
    float x = 0.0f, y = 40.1f, z = -1.2f;
    std::ofstream f("test.bin", std::ios::trunc | std::ios::binary | std::ios::out);

    f.write((const char*)&x, sizeof(float));
    f.write((const char*)&y, sizeof(float));
    f.write((const char*)&z, sizeof(float));
}

void readTheFile() {
    float a, b, c;
    std::ifstream f("test.bin", std::ios::binary);

    f.read((char*)&a, sizeof(float));
    f.read((char*)&b, sizeof(float));
    f.read((char*)&c, sizeof(float));
	/*std::cout << typeid(a).name() << "\n";
	a = boost::lexical_cast<float>(a);
	b = boost::lexical_cast<float>(b);
	c = boost::lexical_cast<float>(c);
	std::cout << typeid(a).name() << "\n";*/
    std::cout << a << ", " << b << ", " << c << std::endl;
}

int main()
{
    writeTheFile();
    readTheFile();
}
