#include <math.h>
#include "../include/geometry.h"

double ARCSTEPLENGTH = 0.005;
double LINESTEPLENGTH = 0.005;
double MAX_CURVATURE_RADIUS = 10000.0;

int dci(double x) {
    return (int) (x + 0.5);
};

double calAngle(double startx, double starty, double endx, double endy) {
    double sins = startx * endy - starty * endx;
    double coss = startx * endx + starty * endy;
    return atan2(sins, coss) * (180.0 / M_PI);
}

double calAngleWithX_axis(double endx, double endy) {
    return atan2(endy, endx);
}


double clipDouble(double value, double min,double max) {
    if(value<min)
    {
        return  min;
    }
    else if(value>max)
    {
        return max;
    } else{
        return  value;
    }
}


bool pointInSameLine(int x1, int y1, int x2, int y2, int x3, int y3) {
    if ((x1 == x2) && (x1 == x3))
        return true;
    int dr = (y2 - y1) * (x3 - x1);
    int dl = (y3 - y1) * (x2 - x1);
    if (dr == dl)
        return true;
    return false;
}

/*
 * 直线参数方程:(x-x1)/(x2-x1) = (y-y1)(y2-y1) = t  ===> x = x1 + t(x2-x1), y = y1 + t(y2-y1). 其中t属于[0,1];
 */
bool bresenhamLine(struct grid *gd, int x1, int y1, int x2, int y2, int *o_x, int *o_y, int w, int h, bool draw,
                   bool isRePlan) {
    double t = 0.0, Line_Length = 0.0;
    int x, y, dx, dy;
    dx = x2 - x1;
    dy = y2 - y1;
    Line_Length = pointDistance(x1, y1, x2, y2);
    LINESTEPLENGTH = 1.0 / (Line_Length);
//    std::cout << "LINESTEP: " << LINESTEPLENGTH << std::endl;
    for (t; t - 1.0 <= EPSINON; t += LINESTEPLENGTH) {
        x = dci(x1 + t * dx);
        y = dci(y1 + t * dy);
        //std::cout << "x, y:" << x << " " << y << std::endl;
        if (x < 0 || y < 0 || x >= w || y >= h)
            return false;
        if (draw) {
            setWalkableAt(gd, x, y, false);
//            SETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, WALKABLE_FALSE, false);
        } else {
            if (isRePlan) {
                if (!GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, VIEW_WALKABLE_TRUE)) {
                    *o_x = x;
                    *o_y = y;
                    return false;
                }
            } else {
                if (!GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, STATIC_WALKABLE_TRUE) ||
                    !GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, DYNAMIC_WALKABLE_TRUE)) {
                    *o_x = x;
                    *o_y = y;
                    return false;
                }
            }
        }
    }
    return true;
}

//圆边界判断
void boundaryHandle(struct grid *gd, int x, int y, int w, int h) {
    if (x >= h || x < 0 || y >= w || y < 0) {
        return;
    } else {
        setWalkableAt(gd, y, x, false);
//        SETWALKABLE(gd->nodes[x][y].grid_layered_walkable_, WALKABLE_FALSE, false);
    }
}

//圆具有八对称性
bool _draw_circle_8(struct grid *gd, int xc, int yc, int x, int y, int w, int h, bool draw) {
    if (draw) {
        boundaryHandle(gd, xc + x, yc + y, w, h);
        boundaryHandle(gd, xc - x, yc + y, w, h);
        boundaryHandle(gd, xc + x, yc - y, w, h);
        boundaryHandle(gd, xc - x, yc - y, w, h);
        boundaryHandle(gd, xc + y, yc + x, w, h);
        boundaryHandle(gd, xc - y, yc + x, w, h);
        boundaryHandle(gd, xc + y, yc - x, w, h);
        boundaryHandle(gd, xc - y, yc - x, w, h);

    } else {
        if (!isWalkableAt(gd, yc + y, xc + x) || !isWalkableAt(gd, yc + y, xc - x) ||
            !isWalkableAt(gd, yc - y, xc + x) || !isWalkableAt(gd, yc - y, xc - x) ||
            !isWalkableAt(gd, yc + x, xc + y) || !isWalkableAt(gd, yc + x, xc - y) ||
            !isWalkableAt(gd, yc - x, xc + y) || !isWalkableAt(gd, yc - x, xc - y))
            return false;
    }
    return true;
}

bool draw_circle(struct grid *gd, int xc, int yc, int r, int w, int h, bool draw) {
    int x = 0, y = r, d;
    d = 3 - 2 * r;
    while (x <= y) {

        if (!_draw_circle_8(gd, xc, yc, x, y, w, h, draw)) return false;

        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
    return true;
}

Geometry_Point getCross(double x1, double y1, double x2, double y2, double x3, double y3,
                        double x4, double y4) {
    Geometry_Point CrossP;
    double a1 = 1.0 * (y1 - y2) / (x1 - x2);
    double b1 = y1 - a1 * (x1);

    double a2 = 1.0 * (y3 - y4) / (x3 - x4);
    double b2 = y3 - a1 * (x3);

    CrossP.x = (b1 - b2) / (a2 - a1);
    CrossP.y = a1 * CrossP.x + b1;
    return CrossP;
}

bool arcWalkable2(struct grid *gd, Geometry_Point center, double radius, Geometry_Point start,
                  Geometry_Point end, int *o_x, int *o_y, int w, int h, bool draw) {
    double start_angle, end_angle;
    double i, arcx, arcy;
    start_angle = calAngle(start.x - center.x, start.y - center.y, 0, 1) * M_PI / 180.0;
    end_angle = calAngle(end.x - center.x, end.y - center.y, 0, 1) * M_PI / 180.0;

    if (start_angle - end_angle > EPSINON) {
        double temp = start_angle;
        start_angle = end_angle;
        end_angle = temp;
    }
    //起始角和终止角相差PI，走另外一半圆弧
    if (end_angle - start_angle > M_PI) {
        double temp = start_angle;
        start_angle = end_angle;
        end_angle = 2 * M_PI - fabs(temp);
    }

    for (i = start_angle; end_angle - i > EPSINON; i += ARCSTEPLENGTH) {
        arcx = center.x + radius * sin(i);
        arcy = center.y + radius * cos(i);
        int x = (int) (arcx + 0.5);
        int y = (int) (arcy + 0.5);
        if (draw) {
            setWalkableAt(gd, x, y, false);
        } else {
            if (!GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, STATIC_WALKABLE_TRUE) ||
                !GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, DYNAMIC_WALKABLE_TRUE)) {
                *o_x = x;
                *o_y = y;
                return false;
            }
        }
    }
    return true;
}

bool arcWalkable(struct grid *gd, Geometry_Point center, double radius, Geometry_Point start,
                 Geometry_Point end, int *o_x, int *o_y, int w, int h, bool draw) {
    double start_angle, turn_angle, end_angle, delta_theta;
    start_angle = fabs(calAngle(1, 0, start.x - center.x, start.y - center.y) * M_PI / 180.0);
    if (start.y - center.y > EPSINON) { // 保证start_angle为起点与x轴正方向的夹角
        start_angle = 2 * M_PI - start_angle;
    }

    turn_angle = fabs(calAngle(start.x - center.x, start.y - center.y,
                               end.x - center.x, end.y - center.y) * M_PI / 180.0);
    if (radius > EPSINON) {
        // 逆时针
        delta_theta = ARCSTEPLENGTH;
        end_angle = start_angle + turn_angle;
    } else {
        // 顺时针
        delta_theta = -ARCSTEPLENGTH;
        end_angle = start_angle - turn_angle;
    }

    double abs_radius = fabs(radius);
    for (double theta = start_angle; fabs(theta - end_angle) > 3 * ARCSTEPLENGTH; theta += delta_theta) {
        int x = (int) (center.x + abs_radius * cos(theta) + 0.5);
        int y = (int) (center.y - abs_radius * sin(theta) + 0.5);
        if (draw) {
            setWalkableAt(gd, x, y, false);
        } else {
            if (!GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, VIEW_WALKABLE_TRUE)) {
                *o_x = x;
                *o_y = y;
                return false;
            }
        }
    }
    return true;
}

Geometry_Point getCicleCenter(int x1, int y1, int x2, int y2, int x3, int y3, double r) {
    /*
     * (dx,dy)为向量(x1-x2,y1-y2)和向量(x3-x2,y3-y2)的角平分线向量
     * 利用公式向量a是向量b和c的角平分线向量，则 a = b/|b| + c/|c|.
     */
    double dx, dy, x4, y4, k;
    Geometry_Point center, p1, p2;
    double absolute1 = sqrt(1.0 * (x1 - x2) * (x1 - x2) + 1.0 * (y1 - y2) * (y1 - y2));
    double absolute2 = sqrt(1.0 * (x3 - x2) * (x3 - x2) + 1.0 * (y3 - y2) * (y3 - y2));
    dx = (x1 - x2) / absolute1 + (x3 - x2) / absolute2;
    dy = (y1 - y2) / absolute1 + (y3 - y2) / absolute2;
    //(x2,y2)->(x4,y4)即为角平分线所在直线
    x4 = dx + x2;
    y4 = dy + y2;

    if (x2 - x1) {
        k = 1.0 * (y2 - y1) / (x2 - x1);
    } else {
        k = 1.0 * (y3 - y2) / (x3 - x2);
    }

    /*
     * 求直线p1->p2和直线(x2,y2)->(x4,y4)的交点，这点即为圆的圆心点
     * 直线p1->p2是与直线(x1,y1)->(x2,y2)或直线(x2,y2)->(x3,y3)平行且距离为r的直线
     * 利用公式：与直线y=kx+b平行且距离为r的直线方程为y=kx+b+sqrt((kr)^2+r^2) 或 y=kx+b-sqrt((kr)^2+r^2)
     */
    p1.x = 0.0, p1.y = y2 - k * x2 + sqrt(k * k * r * r + r * r);
    p2.x = 1, p2.y = k + y2 - k * x2 + sqrt(k * k * r * r + r * r);
    center = getCross(x2, y2, x4, y4, p1.x, p1.y, p2.x, p2.y);

    //圆心在角平方线上，向量(center.x-x2,center.y-y2)与角平分线向量夹角为0
    double angle = calAngle(x4 - x2, y4 - y2, center.x - x2, center.y - y2);
    if (fabs(angle) > 0.1) {
        p1.y = y2 - k * x2 - sqrt(k * k * r * r + r * r);
        p2.y = k + y2 - k * x2 - sqrt(k * k * r * r + r * r);
        center = getCross(x2, y2, x4, y4, p1.x, p1.y, p2.x, p2.y);
    }

    //printf("center = %lf %lf\n", center.x, center.y);
    return center;
}

Geometry_Point getVerticalCross(const Geometry_Point &center, int x1, int y1, int x2, int y2) {
    Geometry_Point VerCross;
    if (x1 == x2) {
        VerCross.x = x1;
        VerCross.y = center.y;
    } else {
        double A = 1.0 * (y1 - y2) / (x1 - x2);
        double B = (y1 - A * x1);
        //0 = ax +b -y; 对应垂线方程为 -x -ay + m = 0;(m为系数)
        //A = a; B = b;
        double m = center.x + A * center.y;

        //求两直线交点坐标
        VerCross.x = ((m - A * B) / (A * A + 1));
        VerCross.y = (A * VerCross.x + B);
    }

    return VerCross;
}

double pointDistance(double x1, double y1, double x2, double y2) {
    double dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    return dis;
}

double pointToLineDistance(const Geometry_Point &center, int x1, int y1, int x2, int y2) {
    /*
     * 若直线平行与y轴，即斜率不存在时特判；否则运用公式(x0,y0)到y=kx+b距离为 |kx0-y+b|/sqrt(k^2+1)
     */
    if (x1 == x2) {
        return fabs(center.x - x1);
    } else {
        double k = 1.0 * (y1 - y2) / (x1 - x2);
        double b = y1 - k * x1;
        return fabs(k * center.x - center.y + b) / sqrt(k * k + 1);
    }
}

bool isLineCircleCross(const Geometry_Point &center, int radius, int x1, int y1, int x2, int y2) {
    /*
     * 线段AB与圆(圆心O)相交情况:
     * 1).满足圆心到线段所在直线距离小于或等于半径
     * 2).点Ａ或Ｂ在圆上
     * 3).线段两点到圆心的距离一个大于半径，一个小于半径
     * 4).线段两点到圆心距离均大于半径，且角OAB和角OBA均为锐角.
     */
    if (radius >= pointToLineDistance(center, x1, y1, x2, y2)) {
        double dis_1 = pointDistance(center.x, center.y, x1, y1);
        double dis_2 = pointDistance(center.x, center.y, x2, y2);
        if (fabs(dis_1 - radius) < 0.01 || fabs(dis_2 - radius) < 0.01) {
            return true;
        }
        if (dis_1 < radius && dis_2 < radius) {
            return false;
        } else if ((dis_1 < radius && dis_2 > radius)) {
            return true;
        } else if (dis_1 > radius && dis_2 > radius) {
            double angle_1 = calAngle(center.x - x1, center.y - y1, x2 - x1, y2 - y1);
            double angle_2 = calAngle(center.x - x2, center.y - y2, x1 - x2, y1 - y2);
            if (fabs(angle_1) - 90.0 <= EPSINON && fabs(angle_2) - 90.0 <= EPSINON)
                return true;
        }

    }
    return false;
}

Geometry_Point getLineCircleCross(const Geometry_Point &center, int radius, int x1,
                                  int y1, int x2, int y2) {
    /*
     * 已知前提：直线与圆必有交点
     * 1).若直线斜率不存在
     * 2).斜率存在，y=kx+b代入(x-cx)^2+(y-cy)^2=r^2,求解二次函数如下：
     * (1+k^2)*x^2 + (2kb-2k*cy-2*cx)*x + (cx^2+cy^2+b^2-2b*cy-r^2)=0. ---> Ax^2+Bx+C=0.
     * 解为:x=(-B +/- sqrt(B^2-4AC))/2A;
     */
    Geometry_Point cross;
    if (x1 == x2) {
        cross.x = x2;
        double y_1 = sqrt(1.0 * radius * radius - (x1 - center.x) * (x1 - center.x)) + center.y;
        double y_2 = -1 * sqrt(1.0 * radius * radius - (x1 - center.x) * (x1 - center.x)) + center.y;
        if (fabs(y_2 - y2) - fabs(y_1 - y2) <= EPSINON) {
            cross.y = y_2;
        } else {
            cross.y = y_1;
        }

    } else {
        double k, b, A, B, C, x_1, x_2, dis_1, dis_2;
        k = 1.0 * (y1 - y2) / (x1 - x2);
        b = y1 - k * x1;
        A = 1 + k * k;
        B = 2 * k * b - 2 * k * center.y - 2 * center.x;
        C = 1.0 * center.x * center.x + center.y * center.y + b * b - 2 * b * center.y - radius * radius;
        x_1 = (-1 * B + sqrt(B * B - 4 * A * C)) / (2 * A);
        x_2 = (-1 * B - sqrt(B * B - 4 * A * C)) / (2 * A);
        dis_1 = pointDistance(x_1, k * x_1 + b, x2, y2);
        dis_2 = pointDistance(x_2, k * x_2 + b, x2, y2);
        //靠近(x2,y2)点
        if (dis_1 - dis_2 <= EPSINON) {
            cross.x = x_1;
        } else {
            cross.x = x_2;
        }
        cross.y = k * cross.x + b;
    }
    return cross;
}

Geometry_Point getArcCircleCross(const Geometry_Point &center, int radius,
                                 const Geometry_Point &arc_center, Geometry_Point start,
                                 Geometry_Point end, int arc_radius) {
    /*
     * 根据余弦函数计算转角d_angle
     */
    double cos_A, A, full_angle, d_angle, start_angle, end_angle;
    Geometry_Point cross;
    cos_A = 1.0 - 1.0 * arc_radius * arc_radius / (2 * radius * radius);
    A = acos(cos_A);

    full_angle = calAngle(center.x - arc_center.x, center.y - arc_center.y,
                          end.x - arc_center.x, end.y - arc_center.y) * M_PI / 180.0;
    start_angle = calAngle(start.x - center.x, start.y - center.y, 0, 1) * M_PI / 180.0;
    end_angle = calAngle(end.x - center.x, end.y - center.y, 0, 1) * M_PI / 180.0;

    if (start_angle - end_angle > EPSINON) {
        double temp = start_angle;
        start_angle = end_angle;
        end_angle = temp;
    }
    //起始角和终止角相差PI，走另外一半圆弧
    if (end_angle - start_angle > M_PI) {
        double temp = start_angle;
        start_angle = end_angle;
        end_angle = 2 * M_PI - fabs(temp);
    }

    d_angle = fabs(fabs(full_angle) - A) + start_angle;
    cross.x = arc_center.x + arc_radius * sin(d_angle);
    cross.y = arc_center.y + arc_radius * cos(d_angle);
    return cross;
}

double calSplineAngle(Geometry_Point p0, Geometry_Point p1, Geometry_Point p2, Geometry_Point p3, double t) {
    Geometry_Point bk;
    bk.x = -3 * (1 - t) * (1 - t) * p0.x + (9 * t * t - 12 * t + 3) * p1.x +
           (6 * t - 9 * t * t) * p2.x + 3 * t * t * p3.x;
    bk.y = -3 * (1 - t) * (1 - t) * p0.y + (9 * t * t - 12 * t + 3) * p1.y +
           (6 * t - 9 * t * t) * p2.y + 3 * t * t * p3.y;
    return calAngleWithX_axis(bk.x, bk.y);
}

double calCurvature(Geometry_Point p0, Geometry_Point p1, Geometry_Point p2, Geometry_Point p3, double t) {
    double dx, dy, d2x, d2y;
    double SQUARET = t * t;
    double curvature = 0.0;
    dx = (-3 * SQUARET + 6 * t - 3) * p0.x + (9 * SQUARET - 12 * t + 3) * p1.x +
         (-9 * SQUARET + 6 * t) * p2.x + (3 * SQUARET) * p3.x;
    d2x = (-6 * t + 6) * p0.x + (18 * t - 12) * p1.x + (-18 * t + 6) * p2.x + 6 * t * p3.x;
    dy = (-3 * SQUARET + 6 * t - 3) * p0.y + (9 * SQUARET - 12 * t + 3) * p1.y +
         (-9 * SQUARET + 6 * t) * p2.y + (3 * SQUARET) * p3.y;
    d2y = (-6 * t + 6) * p0.y + (18 * t - 12) * p1.y + (-18 * t + 6) * p2.y + 6 * t * p3.y;
    curvature = (dx * d2y - d2x * dy) / sqrt(pow(dx * dx + dy * dy, 3));
    return curvature;
}

void lerp(Geometry_Point &dest, Geometry_Point a, Geometry_Point b, double t) {
    dest.x = a.x + (b.x - a.x) * t;
    dest.y = a.y + (b.y - a.y) * t;
}

void bezier(Geometry_Point &dest, Geometry_Point a, Geometry_Point b,
            Geometry_Point c, Geometry_Point d, double t) {
    Geometry_Point ab, bc, cd, abbc, bccd;
    lerp(ab, a, b, t); //point between a and b
    lerp(bc, b, c, t);
    lerp(cd, c, d, t);
    lerp(abbc, ab, bc, t);
    lerp(bccd, bc, cd, t);
    lerp(dest, abbc, bccd, t); //point on bezier-curve
}

double _bezier_point(double t, double start, double control_1,
                     double control_2, double end) {
    /* Formula from Wikipedia article on Bezier curves. */
    return start * (1.0 - t) * (1.0 - t) * (1.0 - t)
           + 3.0 * control_1 * (1.0 - t) * (1.0 - t) * t
           + 3.0 * control_2 * (1.0 - t) * t * t
           + end * t * t * t;
}

/*
   Approximate length of the Bezier curve which starts at "start" and
   is defined by p1, p2, p3. According to Computing the Arc Length of Cubic Bezier Curves
   there is no closed form integral for it.
*/
double bezier_length(Geometry_Point p0, Geometry_Point p1, Geometry_Point p2, Geometry_Point p3) {
    double t;
    int i, steps;
    Geometry_Point dot, previous_dot;
    double length = 0.0;
    steps = 20;
    for (i = 0; i <= steps; i++) {
        t = (double) i / (double) steps;
        dot.x = _bezier_point(t, p0.x, p1.x, p2.x, p3.x);
        dot.y = _bezier_point(t, p0.y, p1.y, p2.y, p3.y);
        if (i > 0) {
            double x_diff = dot.x - previous_dot.x;
            double y_diff = dot.y - previous_dot.y;
            length += sqrt(x_diff * x_diff + y_diff * y_diff);
        }
        previous_dot = dot;
    }
    return length;
}

bool bezierWalkable(struct grid *gd, Geometry_Point ps1, Geometry_Point pA, Geometry_Point pB, Geometry_Point ps2,
                    int &obs_x, int &obs_y, double &radius, bool isCheck, int w, int h) {
    int i, x, y;
    double t, max_curvature = -10000.0, t_cur;
    Geometry_Point dest;
    int BEZIERPOINT = (int) (2 * bezier_length(ps1, pA, pB, ps2));
//    std::cout << "Bezier_Length: " << BEZIERPOINT << std::endl;

    for (i = 0; i <= BEZIERPOINT; i++) {
        t = i * 1.0 / BEZIERPOINT;
        bezier(dest, ps1, pA, pB, ps2, t);
        x = dci(dest.x);
        y = dci(dest.y);
        if (y < 0 || x < 0 || x >= w || y >= h)
            return false;
        if (isCheck) {
            if (!GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, VIEW_WALKABLE_TRUE)) {
                obs_x = x;
                obs_y = y;
                return false;
            }
        } else {
            if (!GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, STATIC_WALKABLE_TRUE) ||
                !GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, DYNAMIC_WALKABLE_TRUE)) {
                obs_x = x;
                obs_y = y;
                return false;
            }
            t_cur = calCurvature(ps1, pA, pB, ps2, t);
            if (fabs(t_cur) > max_curvature) {
                max_curvature = fabs(t_cur);
            }
        }
    }
    if (!isCheck) {
        if (max_curvature < 1.0 / MAX_CURVATURE_RADIUS)
            radius = MAX_CURVATURE_RADIUS;
        else
            radius = 1.0 / max_curvature;
    }
    return true;
}

//void drawBezier(struct grid *gd, Geometry_Point ps1, Geometry_Point pA, Geometry_Point pB, Geometry_Point ps2, int w,
//                int h) {
//    int i;
//    double t;
//    Geometry_Point dest;
//    double max_curvature = -10000.0, t_cur, t_max;
//    for (i = 0; i <= BEZIERPOINT; i++) {
//        t = i * 1.0 / BEZIERPOINT;
//        bezier(dest, ps1, pA, pB, ps2, t);
//        t_cur = calCurvature(ps1, pA, pB, ps2, t);
//        if (fabs(t_cur) > max_curvature) {
//            t_max = t;
//            max_curvature = fabs(t_cur);
//        }
//        if (i == 0 || i == BEZIERPOINT)
//            std::cout << "t: " << t << "  curvature: " << t_cur << std::endl;
//        if (dci(dest.x) < 0 || dci(dest.y) < 0 || dci(dest.x) >= w || dci(dest.y) >= h)
//            continue;
//        gd->nodes[dci(dest.y)][dci(dest.x)].walkable = false;
//    }
//    std::cout << "max_t: " << t_max << " max_curature: " << max_curvature << std::endl;
//    return;
//}

double pointToLineSegmentDistance(const Geometry_Point &p, int x1, int y1, int x2, int y2) {
    double A, B, xx, yy, dx, dy, dot, param = -1;
    int len_sq, C, D;
    A = p.x - x1;
    B = p.y - y1;
    C = x2 - x1;
    D = y2 - y1;
    dot = A * C + B * D;
    len_sq = C * C + D * D;
    if (len_sq != 0) //in case of 0 length line
        param = dot;
    if (param < EPSINON) {
        xx = x1;
        yy = y1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    dx = p.x - xx;
    dy = p.y - yy;
    return sqrt(dx * dx + dy * dy);
}
