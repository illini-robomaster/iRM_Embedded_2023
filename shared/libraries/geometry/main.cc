#include "geometry.h"
#include <iostream>
int main(){
    Rotation3d r1(0.01,0,0);
    Rotation3d r2(0.02,0,0);
    Rotation3d r3 = r1*r2;
    std::cout << r1.getYaw() << " " << r1.getPitch() << " " << r1.getRoll() << std::endl;
    std::cout << r2.getYaw() << " " << r2.getPitch() << " " << r2.getRoll() << std::endl;
    std::cout << r3.getYaw() << " " << r3.getPitch() << " " << r3.getRoll() << std::endl;
}