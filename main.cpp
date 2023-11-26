#include <boost/python.hpp>
#include <iostream>

int main() {
    Py_Initialize();
    std::cout << "Boost is compiled against Python version: " 
              << Py_GetVersion() << std::endl;
    Py_Finalize();
    return 0;
}
