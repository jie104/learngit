//
// Created by zxj on 2022/12/15.
//
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
int main()
{
    Eigen::Isometry2d T1 = Eigen::Isometry2d::Identity();
    T1.rotate(0.0*M_PI/180.0);
    T1.translate(Eigen::Vector2d(0,0));
    Eigen::Isometry2d T2 = Eigen::Isometry2d::Identity();
    T2.rotate(45.0*M_PI/180.0);
    T2.translate(Eigen::Vector2d(1,2));
    Eigen::Isometry2d dT = T2 * T1.inverse();
    std::cout << dT.rotation() << std::endl;
    std::cout << dT.translation() << std::endl;


}