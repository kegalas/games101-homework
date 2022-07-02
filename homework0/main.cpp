#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

const double PI = acos(-1);

int main(){
    Eigen::Vector3d v(2.0,1.0,1.0);
    Eigen::Matrix3d i,j;
    double deg = PI*45.0/180.0;
    i<<std::cos(deg),-std::sin(deg),0.0,std::sin(deg),std::cos(deg),0.0,0.0,0.0,1.0;
    j<<1.0,0,1.0,0,1.0,2.0,0,0,1.0;

    v = i*v;
    v = j*v;

    std::cout<<v<<std::endl;

    return 0;
}

