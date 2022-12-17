#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <cmath>
Eigen::Isometry2f EuclTf(const Eigen::Vector3f& src_point,const Eigen::Vector3f& tar_point){
    Eigen::Matrix2f R;
    float theta=tar_point[2]-src_point[2];
    R << cos(theta),-sin(theta),
            sin(theta),cos(theta);
    Eigen::Vector2f t=tar_point.head(2)-R*src_point.head(2);
    Eigen::Isometry2f T=Eigen::Isometry2f::Identity();
    T.rotate(R);
    T.pretranslate(t);

    return T;

}
Eigen::Isometry2f CoordinateTf(const Eigen::Vector3f& src_point){
    Eigen::Matrix2f R;
    float theta=src_point[2];
    R << cos(theta),-sin(theta),
            sin(theta),cos(theta);
    Eigen::Vector2f t=Eigen::Vector2f(src_point[0],src_point[1]);
//        LOG(INFO) << "R: " << R << "determina " << R.determinant();
//        LOG(INFO) << "t: " << t;
    Eigen::Isometry2f T=Eigen::Isometry2f::Identity();
    T.pretranslate(t);
    T.rotate(R);

    Eigen::Isometry2d T_temp = Eigen::Isometry2d::Identity();
    T_temp.rotate(src_point[2]);
    T_temp.pretranslate(Eigen::Vector2d(src_point[0], src_point[1]));

    std::cout << "----T---" << std::endl;
    std::cout << T.matrix() << std::endl;

    std::cout << "----T_temp---" << std::endl;
    std::cout << T_temp.inverse().matrix() << std::endl;



    return T;
}

Eigen::Affine2f CoordinateTf1(const Eigen::Vector3f& src_point){
    Eigen::Affine2f T =
             Eigen::Rotation2Df(src_point[2])*Eigen::Translation2f(src_point[0], src_point[1]) ;

    Eigen::Isometry2d T_temp = Eigen::Isometry2d::Identity();
    T_temp.rotate(src_point[2]);
    T_temp.pretranslate(Eigen::Vector2d(src_point[0], src_point[1]));

    std::cout << "----T---" << std::endl;
    std::cout << T.matrix() << std::endl;

    std::cout << "----T_temp---" << std::endl;
    std::cout << T_temp.inverse().matrix() << std::endl;

//    Eigen::Matrix2f R;
//    float theta=src_point[2];
//    R << cos(theta),-sin(theta),
//            sin(theta),cos(theta);
//    Eigen::Vector2f t=Eigen::Vector2f(src_point[0],src_point[1]);
////        LOG(INFO) << "R: " << R << "determina " << R.determinant();
////        LOG(INFO) << "t: " << t;
//    Eigen::Affine2f T=Eigen::Affine2f::Identity();
//    T.rotate(R);
//    T.pretranslate(t);


    return T;
}



int main()
{
//    Eigen::Vector2d a(0,0);
//    std::cout << a.transpose() << a[0] << a[1] << std::endl;
//    Eigen::Vector2d b(1,2);
//    Eigen::Vector2d c(2,6);
//    Eigen::Matrix<double,2,2> v=b*c.transpose();
//    std::cout << v(1,1) << std::endl;

//    Eigen::Vector3f a(1,2,M_PI/4);
//    Eigen::Vector3f b(3,6,3*M_PI/4);
//    std::cout << "b: " << b << std::endl;
//    double theta=b[2]-a[2];
//    Eigen::Matrix<float,2,2> R;
//    R << cos(theta),-sin(theta),
//        sin(theta),cos(theta);
//    Eigen::Vector2f t=b.head(2)-R*a.head(2);
//
//    Eigen::Vector2f c=R*a.head(2)+t;
//    std::cout << "c: " << c << std::endl;
//
    Eigen::Vector3f O2(3,3,M_PI/4);
    Eigen::Vector3f O3(0,0,0);
    Eigen::Vector2f P_O2(2,0);

//    Eigen::Isometry2f T2=CoordinateTf(O2);
//    Eigen::Isometry2f T3=CoordinateTf(O3);
//    Eigen::Vector2f P_03=T2*P_O2;
//    std::cout << "P_03: " << P_03 << std::endl;

    Eigen::Affine2f T4= CoordinateTf(O2);
//    Eigen::Vector2f P_04=T4*P_O2;
//    std::cout << "P_04: " << P_04 << std::endl;
    std::cout << "T inverse: " << std::endl;
    std::cout << T4.inverse()*Eigen::Vector2f(0,0) << std::endl;






//
//    double theta=O2[2]-O3[2];
////    std::cout << "theta: " << theta*180/M_PI << std::endl;
//    Eigen::Matrix<float,2,2> R32;
//    R32 << cos(theta),-sin(theta),
//            sin(theta),cos(theta);
//    std::cout << "R32: " << R32 << std::endl;
//
//
//    Eigen::Vector2f t32(O2[0]-O3[0],O2[1]-O3[1]);
////    std::cout << "t32: " << t32 << std::endl;
////    Eigen::Vector2f P_03=R32*P_O2.head(2)+t32;
////    std::cout << "P_O3: " << P_03 << std::endl;
//
//    Eigen::Isometry2f T=Eigen::Isometry2f::Identity();
//    T.pretranslate(t32);
//    T.rotate(R32);
//    Eigen::Vector2f P_03=T*P_O2;
//    std::cout << "T*P_O2: " << P_03;
//

//    std::cout << T.matrix() << std::endl;


//    Eigen::Vector3f OA(7,5,M_PI/6);
//    Eigen::Vector3f OB(2,2,M_PI/4);
//    Eigen::Vector2f OD=OA.head(2);
//    Eigen::Isometry2f T1= EuclTf(OA,OB);
//    Eigen::Vector2f OC=T1*OD;
//
//    std::cout << OC << std::endl;


}