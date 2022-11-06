#include <iostream>
#include <cmath>

struct Point{
    Point():x(0),y(0) {}
    Point(double a,double b):x(a),y(b) {}

    Point operator -(Point P){
        return Point(x-P.x,y-P.y);
    }
    Point operator +(Point P){
        return Point(x+P.x,y+P.y);
    }

    double operator*(Point P){
        return x*P.x+y*P.y;
    }


    double Mod(){
        if (x==0 && y==0){
            std::cout << "the Mod is zero!!!  it is wrong";
            return 0;
        }
        return sqrt(x*x+y*y);
    }

    void unitizate(){
        double mod=sqrt(x*x+y*y);
        if (mod==0){
        }
        x=x/mod;
        y=y/mod;
    }


    double x;
    double y;
};


int main()
{
    Point point=Point(2,3);
    point.unitizate();
    std::cout << "x: " << point.x << " y: " << point.y << std::endl;
    

}


