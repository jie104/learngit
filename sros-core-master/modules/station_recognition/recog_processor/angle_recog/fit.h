//#pragma once
#ifndef OPENRADAR_H
#define OPENRADAR_H

#include <iostream>
#include <cmath>
#include <map>
//~ #include "opencv2/highgui/highgui.hpp"
//~ #include "opencv2/imgproc/imgproc.hpp"
//~ #include"Qsin.h"


using namespace std;

#include <vector>
#include "WeightedFit.h"
//~ #include"Coordinate.h"
#include <fstream>
//~ #include"urg_parameter_t.h"
#define CLOCKWISE
//
//static int RadarImageWdith = 720;
//static int RadarImageHeight = 720;
////usual color
///*Colour      Red      Green      Blue      值
//    白色   White    255    255    255    16777215
//    红色    Red    255    0    0    255
//    深红色    Dark    Red    128    0    0    128
//    绿色    Green    0    255    0    65280
//    深绿色    Dark    Green    0    128    0    32768
//    蓝色    Blue    0    0    255    16711680
//    紫红色    Magenta    255    0    255    16711935
//    深紫    Dark    Cyan    0    128    128    8421376
//    黄色    Yellow    255    255    0    65535
//    棕色    Brown    128    128    0    32896
// */
//static int usualColor[15] = {16777215, 255, 128, 65280, 32768,
//                             16711680, 16711935, 8421376, 65535, 32896};
///*<usual color*/
//static int RadarRho[1082] = {0};
//static float RadarTheta[1082] = {0};
////sin cos
//static float RadarSinTheta[1082];
//static float RadarCosTheta[1082];
//
//static int RadarDataCnt = 0;
//static int RadarX[1082] = {0};
//static int RadarY[1082] = {0};


class OpenRadar {
public:
    OpenRadar(void);

    ~OpenRadar(void);

    int BreakRadar(int *RadarRho, float *RadarTheta, int Cnt);

//	void MedFilter(int *RadarRho,float *RadarTheta,int Cnt,int halfWindowSize);
    void ConvertRho2XY(int *rho, float *sinTheta, float *cosTheta, int Cnt, int *X, int *Y);

    int BreakPolyLine(int *X, int *Y, int Cnt, vector<iPoint> &Corners);

    int FitLine(vector<LinePara> &FittedLine, int *X, int *Y, int Cnt);

    bool decodeCorner(int *X, int *Y, LinePara tmpline, int endindex, int startindex);


    int PolyContourFit(int *X, int *Y, int n, float Eps);

    void myStrcpy(string s, char *array);

    int FindCorners(vector<int> &Corner, int *X, int *Y, int start, int Cnt, float Eps);

    bool getCornerLine(int index, LinePara &tmp_line);

    bool getLineVector(int* X, int* Y, int Cnt);

public:
    vector<LinePara> FittedLine;    //Fitted line
    vector<iPoint> RadarCorner;    //Corners
    int RadarState;
    //0x01: 找直线；
    vector<vector<iPoint> > line_point;
    map<int, vector<LinePara> > corner_line;
    map<int, iPoint> corner_point;
    int pointcnt_thresh;
    float linear_rthresh;
    float high_thresh;
    float linear_RMSEthresh;
};

#endif
