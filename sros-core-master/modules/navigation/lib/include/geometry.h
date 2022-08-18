#ifndef  __included_geometry_h
#define __included_geometry_h

#include <eigen3/Eigen/Dense>
#include "jps_grid.h"
#include "core/map/NavigationMap.hpp"

const double EPSINON = 0.000001;

extern double ARCSTEPLENGTH; //圆弧像素点搜索步长
extern double LINESTEPLENGTH; //直线搜索步长
extern double MAX_CURVATURE_RADIUS; //最大曲率


typedef struct point {
    point() = default;
    point(double x, double y): x(x), y(y) {}
    double x, y;
} Geometry_Point;

using Polygon = std::vector<point>;
using Polygons = std::vector<Polygon>; // 多边形列表

/* 四舍五入 */
int dci(double x);

/*
 * @brief 计算两向量的夹角
 * 夹角范围[-PI,PI],逆时针为正。起始向量为(startx,starty),终止向量为(endx,endy).
 */
double calAngle(double startx, double starty, double endx, double endy);

/*
 * @brief 计算与x轴正方向的夹角
 * 夹角范围[-PI,PI],逆时针为正。起始向量为(1, 0),终止向量为(endx, endy).
 */
double calAngleWithX_axis(double endx, double endy);

double clipDouble(double value, double min,double max);

/* 判断三点共线 */
bool pointInSameLine(int x1, int y1, int x2, int y2, int x3, int y3);

/*
 * @brief 判断直线上是否有障碍
 * @param gd为网格地图指针，直线为(x1,y1)->(x2,y2)
 * @param draw若为true则画出直线，否则单纯判断直线段上是否有障碍.
 * @param isRePlan若为true表检测动态障碍是否可走，为false表路径优化是否可走.
 */
bool bresenhamLine(struct grid *gd, int x1, int y1, int x2, int y2, int *o_x, int *o_y, int w, int h, bool draw,
                   bool isRePlan);

/*
 * @brief 利用对称性画圆
 * gd为网格地图指针，center坐标(xc,yc), 圆半径r，地图范围w*h,draw若为true则画出圆，否则判断圆区域是否有障碍
 */
bool draw_circle(struct grid *gd, int xc, int yc, int r, int w, int h, bool draw);

/*
 * @brief 判断弧线是否可走
 * gd为网格地图指针，center是圆心，radius是圆半径，start是弧线起始点坐标，end是弧线终点坐标， draw若为true则画出弧线，
 * 否则单纯判断弧线上是否有障碍
 */
bool arcWalkable(struct grid *gd, Geometry_Point center, double radius, Geometry_Point start,
                 Geometry_Point end, int *o_x, int *o_y, int w, int h, bool draw);

bool arcWalkable2(struct grid *gd, Geometry_Point center, double radius, Geometry_Point start,
                  Geometry_Point end, int *o_x, int *o_y, int w, int h, bool draw);

/**
 * @brief 获取两直线的交点坐标
 * 直线为(x1,y1)->(x2,y2)和直线(x3,y3)->(x4,y4),函数返回交点坐标
 **/
Geometry_Point getCross(double x1, double y1, double x2, double y2, double x3, double y3,
                        double x4, double y4);

/*
 * @brief 两条交线确定圆心
 * 已知两线为(x1,y1)->(x2,y3)和(x2,y2)->(x3,y3), 其中(x2,y2)为两线交点，圆半径为r
 * 函数返回圆心坐标
 * */
Geometry_Point getCicleCenter(int x1, int y1, int x2, int y2, int x3, int y3, double r);

/*
 * @brief 获取垂足坐标
 * 已知圆心坐标center，直线为(x1,y1)->(x2,y2)
 * 函数返回圆心到直线的垂线段垂足坐标
 */
Geometry_Point getVerticalCross(const Geometry_Point &center, int x1, int y1, int x2, int y2);

/*
 * @brief 点到点的距离
 * 返回(x1,y1)到(x2,y2)的距离
 */
double pointDistance(double x1, double y1, double x2, double y2);

/*
 * @brief 点到线的距离
 * center点到直线(x1,y1)->(x2,y2)的距离
 */
double pointToLineDistance(const Geometry_Point &center, int x1, int y1, int x2, int y2);

/*
 * @brief 判断线段与圆是否相交
 * 判断线段(x1,y1)->(x2,y2)与半径为r，圆心为center的圆是否相交
 */
bool isLineCircleCross(const Geometry_Point &center, int radius, int x1, int y1, int x2, int y2);

/*
 * @brief 返回线段和圆交点
 * 线段(x1,y1)->(x2,y2),圆半径radius,圆心center
 */
Geometry_Point getLineCircleCross(const Geometry_Point &center, int radius, int x1, int y1, int x2, int y2);

/*
 * @brief 返回圆弧与圆交点
 * 圆半径radius,圆心center.圆弧半径arc_radius,圆弧圆心arc_center,圆弧开始点start,终点end
 */
Geometry_Point getArcCircleCross(const Geometry_Point &center, int radius, const Geometry_Point &arc_center,
                                 Geometry_Point start, Geometry_Point end, int arc_radius);

/*
 * @brief 计算贝塞尔斜率
 * @param p0,p1,p2,p3 控制点
 * @param t 插值步长
 */
double calSplineAngle(Geometry_Point p0, Geometry_Point p1, Geometry_Point p2, Geometry_Point p3, double t);

/*
 * @brief 计算贝塞尔曲线曲率
 * @param t 插值点
 * @param p0,p1,p2,p3 控制点
 * @return 曲率
 */
double calCurvature(Geometry_Point p0, Geometry_Point p1, Geometry_Point p2, Geometry_Point p3, double t);

/*
 * @brief 两点间的线性插值
 */
void lerp(Geometry_Point &dest, Geometry_Point a, Geometry_Point b, double t);

/*
 * @brief 获取曲线单个像素
 */
void bezier(Geometry_Point &dest, Geometry_Point a, Geometry_Point b,
            Geometry_Point c, Geometry_Point d, double t);

/*
 * @brief 获取贝塞尔线段上点的坐标
 * @param start,control_1,control_2,end为控制点
 */
double _bezier_point(double t, double start, double control_1,
                     double control_2, double end);

/*
 * @brief 弧微分计算贝塞尔长度
 */
double bezier_length(Geometry_Point p0, Geometry_Point p1, Geometry_Point p2, Geometry_Point p3);

/*
 * @brief 判断贝塞尔曲线是否可走
 */
bool bezierWalkable(struct grid *gd, Geometry_Point ps1, Geometry_Point pA, Geometry_Point pB, Geometry_Point ps2,
                    int &obs_x, int &obs_y, double &radius, bool isCheck, int w, int h);

/*
 * @brief 计算点到线段的距离
 * @param p点到(x1,y1)->(x2,y2)
 */
double pointToLineSegmentDistance(const Geometry_Point &p, int x1, int y1, int x2, int y2);

#endif