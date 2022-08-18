#include <cstdio>
#include <cmath>
#include <chrono>
#include "../include/navigation.h"
#include "../include/path.h"
#include "../include/display.h"
#include "core/src.h"
#include "glog/logging.h"
#include "core/settings.h"
#include "core/util/utils.h"


using namespace sros::core;
using namespace sros::map;
using namespace std;

int LENGTH_PER_GRID = 2;

int STOP_DISTANCE = 20 / LENGTH_PER_GRID;
int SLOW_DISTANCE = 70 / LENGTH_PER_GRID;

double OBSTACLE_MEMORIZE_TIME = 2.0;

double IGNORE_ANGLE = 2.0;
double IGNORE_LINE = 2.0;

StructuringElementType INFLATION_TYPE = StructuringElementCircle;

int BUFFER_LENGTH = 8;
int DYNAMIC_BUFFER_LENGTH;

double ROUND_OFF_ANGLE;
double LONGEST_DIVIDED_LENGTH = 100.0 / LENGTH_PER_GRID;

const float SIMULATE_WALK_STEP_LENGTH = 2.0;             //when checking the obstacle, the robot moves the  length of the step ,simulate  unit:cm
float SIMULATE_WALK_TOTAL_LENGTH = 200.0;          //when checking the obstacle, only check the total length that the robot hasnot reached
float INFLATION_FOR_SAFETY = 0.0;                 //reserve 20 centimeter for safety distance
const float RESOLUATION_IN_ROTATE = 1.0 * M_PI / 180    ;                        //
const int PATH_ROTATE_LEAST_NUM_TO_BE_CUT = 50;                          //　
//腐蚀系数为车体长度的一半
#define INFLATION_FACTOR_WIDTH (0) / (2*LENGTH_PER_GRID)

const float OBSTACLE_FILTER_DISTANCE = 4;

static verticesOfRobots fourVerticesWithTwoMiddlePos;
const double REVERSE_ANGLE = 10 / 180.0 * M_PI;//车体当前方向和路径前进方向的反方向 在这个范围内 车体不进行旋转逆向行驶
Navigation::Navigation() {
    navigation_map_.reset(new sros::map::NavigationMap());
    _radar_obstacle.clear();
    _last_clear_time = std::chrono::steady_clock::now().time_since_epoch().count();
    grid_map_.nodes = nullptr;
}

Navigation::~Navigation() {
    int i, height;
    height = navigation_map_->getMapSize().y;
    for (i = 0; i < height; i++) {
        delete[] grid_map_.nodes[i];
       // delete[] localgrid_map_.nodes[i];
        malloc_count--;
    }
    delete[] grid_map_.nodes;
   // delete[] localgrid_map_.nodes;
    malloc_count--;
}

/*
 * 实际地图坐标转化为导航坐标
 */
void Navigation::convertMapCoords(int offset_x, int offset_y, double x, double y, int *cx, int *cy) {
    *cx = dci(offset_x + x * METER_TO_CM / LENGTH_PER_GRID);
    *cy = dci(offset_y - y * METER_TO_CM / LENGTH_PER_GRID);
    return;
}

/*
 * 导航坐标转换为实际地图坐标
 */
void Navigation::reverseMapCoords(int offset_x, int offset_y, double cx, double cy, double *x, double *y) {
    *x = (cx - offset_x) * LENGTH_PER_GRID / METER_TO_CM;
    *y = (offset_y - cy) * LENGTH_PER_GRID / METER_TO_CM;
    return;
}

void convertMapRightCoords(double x, double y, int *cx, int *cy) {
    *cx = dci(x * METER_TO_CM / LENGTH_PER_GRID);
    *cy = dci(y * METER_TO_CM / LENGTH_PER_GRID);
    return;
}

sros::core::NavigationPath_vector Navigation::changeToNavigationPath(sros::core::NavigationPathi_vector &paths) {
    int offset_x, offset_y;
    double sx, sy, ex, ey, cx, cy, dx, dy, radius;
    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;
    sros::core::NavigationPath_vector nav_paths;

    sros::core::NavigationPathi_vector::iterator it;
    for (it = paths.begin(); it != paths.end(); it++) {
        sros::core::NavigationPath<int> p = *(it);
        if (p.type_ == sros::core::PATH_LINE) {
            reverseMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            reverseMapCoords(offset_x, offset_y, p.ex_, p.ey_, &ex, &ey);
            if (p.direction_ == 0x02)
                nav_paths.push_back(sros::core::LinePath(sx, sy, ex, ey, sros::core::PATH_BACKWARD));
            else
                nav_paths.push_back(sros::core::LinePath(sx, sy, ex, ey, sros::core::PATH_FORWARD));

        } else if (p.type_ == sros::core::PATH_ROTATE) {
            reverseMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);

            nav_paths.push_back(sros::core::RotatePath(sx, sy, p.rotate_angle_));
        } else if (p.type_ == sros::core::PATH_ARC) {
            reverseMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            reverseMapCoords(offset_x, offset_y, p.ex_, p.ey_, &ex, &ey);
            reverseMapCoords(offset_x, offset_y, p.cx_, p.cy_, &cx, &cy);

            nav_paths.push_back(sros::core::CirclePath(sx, sy, ex, ey, cx, cy, (p.radius_ < EPSINON)));
        } else if (p.type_ == sros::core::PATH_BEZIER) {
            reverseMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            reverseMapCoords(offset_x, offset_y, p.ex_, p.ey_, &ex, &ey);
            reverseMapCoords(offset_x, offset_y, p.cx_, p.cy_, &cx, &cy);
            reverseMapCoords(offset_x, offset_y, p.dx_, p.dy_, &dx, &dy);
            radius = p.radius_ * LENGTH_PER_GRID / METER_TO_CM;
            if (p.direction_ == 0x02)
                nav_paths.push_back(
                        sros::core::BezierPath(sx, sy, cx, cy, dx, dy, ex, ey, radius, sros::core::PATH_BACKWARD));
            else
                nav_paths.push_back(
                        sros::core::BezierPath(sx, sy, cx, cy, dx, dy, ex, ey, radius, sros::core::PATH_FORWARD));
        }
    }
    return nav_paths;
}

sros::core::NavigationPathi_vector Navigation::changeToMapPath(sros::core::NavigationPath_vector &paths) {
    int offset_x, offset_y, sx, sy, ex, ey, cx, cy, dx, dy;
    double radius;

    sros::core::NavigationPathi_vector nav_paths;
    if (paths.empty()) {
        return nav_paths;
    }
    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;

    sros::core::NavigationPath_vector::iterator it;
    sros::core::NavigationPath<double> p;
    for (it = paths.begin(); it != paths.end(); it++) {
        p = *(it);
        if (p.type_ == sros::core::PATH_LINE) {
            convertMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            convertMapCoords(offset_x, offset_y, p.ex_, p.ey_, &ex, &ey);
            sros::core::LinePathi np(sx, sy, ex, ey);
            nav_paths.push_back(np);

        } else if (p.type_ == sros::core::PATH_ROTATE) {
            convertMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            sros::core::RotatePathi ro(sx, sy, p.rotate_angle_);
            nav_paths.push_back(ro);
        } else if (p.type_ == sros::core::PATH_ARC) {
            convertMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            convertMapCoords(offset_x, offset_y, p.ex_, p.ey_, &ex, &ey);
            convertMapCoords(offset_x, offset_y, p.cx_, p.cy_, &cx, &cy);
            sros::core::CirclePathi ar(sx, sy, ex, ey, cx, cy, (p.radius_ < EPSINON));
            nav_paths.push_back(ar);
        } else if (p.type_ == sros::core::PATH_BEZIER) {
            convertMapCoords(offset_x, offset_y, p.sx_, p.sy_, &sx, &sy);
            convertMapCoords(offset_x, offset_y, p.ex_, p.ey_, &ex, &ey);
            convertMapCoords(offset_x, offset_y, p.cx_, p.cy_, &cx, &cy);
            convertMapCoords(offset_x, offset_y, p.dx_, p.dy_, &dx, &dy);
            radius = p.radius_ * METER_TO_CM / LENGTH_PER_GRID;
            sros::core::BezierPathi sp(sx, sy, cx, cy, dx, dy, ex, ey, radius);
            nav_paths.push_back(sp);
        }
    }
    return nav_paths;
}

/*
 * @brief 将搜索得到的路径点进行处理，产生直线路径，减少路径条数
 * path_head数据结构如下(环链表, 点之间左边为left箭头,右边为right箭头):
 *    -->起点---
 *   |   | |   |
 *   |   点１   |
 *   |   | |   |
 *   |   点２   |
 *   |   | |   |
 *   |   ...   |
 *   |   | |   |
 *   |   终点   |
 *   |   | |   |
 *   |---nul<--|  --->path_head
 */
sros::core::NavigationPathi_vector Navigation::getLinePaths(struct grid *gd,
                                                            struct neighbor_xy_list *path_head,
                                                            bool isFirstNavigate) {
    struct neighbor_xy_list *path_pos = NULL, *begin_pos = NULL;
    sros::core::NavigationPathi_vector paths_with_line;
    int obs_x, obs_y, width, height;
    bool isEnd = false, isBegin = true;

    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;
    path_pos = path_head;

    //以下begin_pos,path_pos,path_pos->left分别表示为点1,点2,点3
    while (path_head != NULL && (path_head != (path_pos = path_pos->left))) {
        if (!isBegin) {
            isEnd = false;
            begin_pos = path_pos->right;
            bool isLongPath = (pointDistance(begin_pos->x, begin_pos->y, path_pos->x, path_pos->y) > SHORTEST_PATH);
            while (1) {
                if (path_pos->left == path_head) { //点2已到达终点,此时点3指向内容为path_head.
                    isEnd = true;
                    break;
                }

                //点1,2,3在同一直线上
                if (pointInSameLine(begin_pos->x, begin_pos->y, path_pos->x, path_pos->y,
                                    path_pos->left->x, path_pos->left->y)) {
                    path_pos = path_pos->left;
                    continue;
                }

                /*
                 * 两种策略.isFirstNavigate为True时无视线段长短进行合并.False时则长线段和长线段合并，短线段和短线段合并
                 * 区分的原因在于之后的路径优化效果。无视路径长短进行合并产生理论上最短路径，其缺点是可能过于靠近障碍，此时雷达点跳动时会
                 * 导致重新规划; 而根据路径长短进行合并好处在于能在拐角处略远离障碍，缺点在于可能有不必要的拐角
                 */
                if (isFirstNavigate) {
                    if (bresenhamLine(gd, begin_pos->x, begin_pos->y,
                                      path_pos->left->x, path_pos->left->y,
                                      &obs_x, &obs_y,
                                      width, height, false, false)) { //点1到点3直线无障碍,点2向后偏移继续查找
                        path_pos = path_pos->left;
                    } else {
                        break;
                    }
                } else {
                    if ((path_pos == (path_head->left)->left || path_pos == (path_head->right)->right)
                        && bresenhamLine(gd, begin_pos->x, begin_pos->y, path_pos->left->x,
                                         path_pos->left->y, &obs_x, &obs_y, width, height, false, false)) {
                        path_pos = path_pos->left;
                        continue;
                    }

                    bool isOk = (pointDistance(path_pos->x, path_pos->y,
                                               path_pos->left->x, path_pos->left->y) > SHORTEST_PATH);

                    if (isLongPath == isOk &&
                        bresenhamLine(gd, begin_pos->x, begin_pos->y,
                                      path_pos->left->x, path_pos->left->y,
                                      &obs_x, &obs_y,
                                      width, height, false,
                                      false)) { //点1到点3直线无障碍且点1点2距离与点2点3距离均为长(或短)路径
                        path_pos = path_pos->left;
                    } else {
                        break;
                    }
                }
            }
            sros::core::LinePathi node(begin_pos->x, begin_pos->y, path_pos->x, path_pos->y); //点1点2连成的直线加入直线路径
            paths_with_line.push_back(node);
            if (isEnd) break;
        }
        isBegin = false;
    }

    return paths_with_line;
}

sros::core::NavigationPathi_vector Navigation::dealPaths(struct grid *gd,
                                                         sros::core::NavigationPathi_vector &paths_with_line) {
    int obs_x, obs_y;
    int width = navigation_map_->getMapSize().x;
    int height = navigation_map_->getMapSize().y;
    sros::core::NavigationPathi_vector path_with_arc;

    //直线路径转角处理为圆弧或原地旋转
    for (sros::core::NavigationPathi_vector::iterator i = paths_with_line.begin(); i != paths_with_line.end(); ++i) {
        sros::core::NavigationPath<int> cur = *(i);
        //printf("cur: %d %d\n", cur.sx_, cur.sy_);
        if ((i + 1) != paths_with_line.end()) {
            sros::core::NavigationPath<int> next = *(i + 1);

            //获取圆心和垂足
            Geometry_Point center = getCicleCenter(cur.sx_, cur.sy_, cur.ex_, cur.ey_,
                                                   next.ex_, next.ey_, ARC_RADIUS);
            Geometry_Point vertical1 = getVerticalCross(center, cur.sx_, cur.sy_, cur.ex_, cur.ey_);
            Geometry_Point vertical2 = getVerticalCross(center, next.sx_, next.sy_, next.ex_, next.ey_);

            //计算直线两端点与垂足的夹角.
            double angle_1 = calAngle(cur.sx_ - vertical1.x, cur.sy_ - vertical1.y,
                                      cur.ex_ - vertical1.x, cur.ey_ - vertical1.y);
            double angle_2 = calAngle(next.sx_ - vertical2.x, next.sy_ - vertical2.y,
                                      next.ex_ - vertical2.x, next.ey_ - vertical2.y);
            //计算两条直线间的夹角
            double angle_between_line = fabs(calAngle(cur.ex_ - cur.sx_, cur.ey_ - cur.sy_,
                                                      next.ex_ - next.sx_, next.ey_ - next.sy_));

            //满足两条直线间夹角大于ROUND_OFF_ANGLE,两个垂足中每个垂足都位于直线上(因为垂足不可位于直线外)且圆弧上无障碍
            if (angle_between_line > ROUND_OFF_ANGLE && fabs(fabs(angle_1) - 180.0) < 0.1 &&
                fabs(fabs(angle_2) - 180.0) < 0.1 &&
                arcWalkable2(gd, center, ARC_RADIUS, vertical1, vertical2, &obs_x, &obs_y, width, height, false)) {
                (i + 1)->sx_ = dci(vertical2.x);
                (i + 1)->sy_ = dci(vertical2.y);

                //直线距离大于可忽略最小距离,加入端点到垂足的直线段
                if (pointDistance(cur.sx_, cur.sy_, dci(vertical1.x), dci(vertical1.y)) - IGNORE_LINE > EPSINON) {
                    sros::core::LinePathi line(cur.sx_, cur.sy_, dci(vertical1.x), dci(vertical1.y));
                    path_with_arc.push_back(line);
                }
                //加入圆弧
                bool isClockWise = (calAngle(vertical1.x - center.x, vertical1.y - center.y,
                                             vertical2.x - center.x, vertical2.y - center.y) > EPSINON);
                sros::core::CirclePathi arc(dci(vertical1.x), dci(vertical1.y), dci(vertical2.x), dci(vertical2.y),
                                            dci(center.x), dci(center.y), isClockWise);
                path_with_arc.push_back(arc);

            } else { //不可圆弧化，加入直线和旋转
                if (pointDistance(cur.sx_, cur.sy_, cur.ex_, cur.ey_) - IGNORE_LINE > EPSINON) {
                    sros::core::LinePathi line(cur.sx_, cur.sy_, cur.ex_, cur.ey_);
                    path_with_arc.push_back(line);
                }
                double angle_3 = -1 * calAngleWithX_axis(next.ex_ - next.sx_, next.ey_ - next.sy_);
                sros::core::RotatePathi rotate(next.sx_, next.sy_, angle_3);
                path_with_arc.push_back(rotate);
            }

        } else { //加入最后一段线段
            if (pointDistance(cur.sx_, cur.sy_, cur.ex_, cur.ey_) - IGNORE_LINE > EPSINON) {
                sros::core::LinePathi node(cur.sx_, cur.sy_, cur.ex_, cur.ey_);
                path_with_arc.push_back(node);
            }
        }
    }

    if (!path_with_arc.empty()) {
        sros::core::NavigationPath<int> p = path_with_arc[0];
        double first_turn = -1 * calAngleWithX_axis(p.ex_ - p.sx_, p.ey_ - p.sy_);
        sros::core::RotatePathi temp(p.sx_, p.sy_, first_turn);
        path_with_arc.insert(path_with_arc.begin(), temp);
    }

    return path_with_arc;
}

void Navigation::deleteRotateFromPaths(sros::core::NavigationPathi_vector &paths) {
    sros::core::NavigationPathi_vector::iterator it;
    sros::core::NavigationPath<int> p, pre_path, next_path;
    double pre_direct, next_direct;
    for (it = paths.begin(); it != paths.end(); ++it) {
        p = *it;
        if (it != paths.begin() && it != (paths.end() - 1) && p.type_ == sros::core::PATH_ROTATE) {
            pre_path = *(it - 1);
            next_path = *(it + 1);
            if (pre_path.type_ == sros::core::PATH_LINE) {
                pre_direct = -1 * calAngleWithX_axis(pre_path.ex_ - pre_path.sx_,
                                                     pre_path.ey_ - pre_path.sy_);
            } else if (pre_path.type_ == sros::core::PATH_BEZIER) {
                pre_direct = -1 * calAngleWithX_axis(pre_path.ex_ - pre_path.dx_,
                                                     pre_path.ey_ - pre_path.dy_);
            }
            if (next_path.type_ == sros::core::PATH_LINE) {
                next_direct = -1 * calAngleWithX_axis(next_path.ex_ - next_path.sx_,
                                                      next_path.ey_ - next_path.sy_);
            } else if (next_path.type_ == sros::core::PATH_BEZIER) {
                next_direct = -1 * calAngleWithX_axis(next_path.cx_ - next_path.sx_,
                                                      next_path.cy_ - next_path.sy_);
            }
            //若原地旋转的前一段路径和后一段路径几近与相切,则去除原地旋转
            if (fabs(pre_direct - next_direct) - IGNORE_ANGLE * M_PI / 180.0 < EPSINON)
                paths.erase(it);
        }
    }
}

sros::core::NavigationPathi_vector Navigation::dividePathIntoPieces(sros::core::NavigationPathi_vector &path) {
    sros::core::NavigationPathi_vector::iterator it;
    sros::core::NavigationPathi_vector path_new;
    int middle_x, middle_y;
    for (it = path.begin(); it != path.end(); ++it) {
        sros::core::NavigationPath<int> p = *it;
        if (pointDistance(p.sx_, p.sy_, p.ex_, p.ey_) > LONGEST_DIVIDED_LENGTH) {
            middle_x = dci((p.sx_ + p.ex_) / 2.0);
            middle_y = dci((p.sy_ + p.ey_) / 2.0);
            sros::core::LinePathi lineleft(p.sx_, p.sy_, middle_x, middle_y);
            sros::core::LinePathi lineright(middle_x, middle_y, p.ex_, p.ey_);
            path_new.push_back(lineleft);
            path_new.push_back(lineright);
        } else {
            sros::core::LinePathi line(p.sx_, p.sy_, p.ex_, p.ey_);
            path_new.push_back(line);
        }
    }
    return path_new;
}

//Paths Navigation::combineLinePaths(struct grid *gd, Paths new_path, Paths last_path, int segment) {
//    int size_path = static_cast<int>(_last_paths_line.size());
//    int exchange = segment / 2;
//    int i;
//    struct neighbor_xy_list *new_path_head = neighbor_xy_new();
//    struct neighbor_xy_list *current = new_path_head;
//    //有时路径不合并
//    printf("next_path:%d exchange:%d size_path:%d\n", segment, exchange, size_path);
//    for (i = size_path - 1; i > exchange; i--) {
//        if (i == size_path - 1)
//            current = neighbor_xy_insert_right(current, _last_paths_line[i].ex_, _last_paths_line[i].ey_);
//        current = neighbor_xy_insert_right(current, _last_paths_line[i].sx_, _last_paths_line[i].sy_);
//    }
//    size_path = (int) new_path.size();
//    for (i = size_path - 1; i >= 0; i--) {
//        if (i == size_path - 1)
//            current = neighbor_xy_insert_right(current, new_path[i].ex_, new_path[i].ey_);
//        current = neighbor_xy_insert_right(current, new_path[i].sx_, new_path[i].sy_);
//    }
//    new_path = getLinePaths(gd, new_path_head);
//    neighbor_xy_clean(new_path_head);
//
//    _last_paths_line = new_path;
//    printf("new_line_path:%d last_paths_line:%d\n", (int) new_path.size(), (int) _last_paths_line.size());
//    return new_path;
//}

bool Navigation::updateBezier(Geometry_Point *p0, Geometry_Point *p1,
                              Geometry_Point *p2, Geometry_Point *p3,
                              long i, long startOrEnd, long restLength, long n,
                              sros::core::NavigationPathi_vector &line_path) {
    if (1 == restLength) //若只有一段路径,则直接返回
        return false;
    /*
     * B_0, B_1...B_n为直线端点.求得每一段直线的贝塞尔四个控制点分别为:
     * S_{i-1}, 2/3*B_{i-1}+1/3*B_i, 1/3*B_{i-1}+2/3*B_i, S_i
     * 其中S_i = 1/6*B_{i-1} + 2/3*B_i + 1/6*B_{i+1}, i=1,2,...n-1.
     * 对于i=0时,S_0 = B_0
     * 对于i=n时,S_n = B_n.
     */
    if (0 == startOrEnd) {
        p0[i].x = line_path[i].sx_, p0[i].y = line_path[i].sy_;
        p3[i].x = 1.0 / 6 * line_path[i].sx_ + 2.0 / 3 * line_path[i].ex_ + 1.0 / 6 * line_path[i + 1].ex_;
        p3[i].y = 1.0 / 6 * line_path[i].sy_ + 2.0 / 3 * line_path[i].ey_ + 1.0 / 6 * line_path[i + 1].ey_;
    } else if (restLength - 1 == startOrEnd) {
        p0[i].x = 1.0 / 6 * line_path[n - 2].sx_ + 2.0 / 3 * line_path[n - 1].sx_ + 1.0 / 6 * line_path[n - 1].ex_;
        p0[i].y = 1.0 / 6 * line_path[n - 2].sy_ + 2.0 / 3 * line_path[n - 1].sy_ + 1.0 / 6 * line_path[n - 1].ey_;
        p3[i].x = line_path[i].ex_, p3[i].y = line_path[i].ey_;
    } else {
        p0[i].x = 1.0 / 6 * line_path[i - 1].sx_ + 2.0 / 3 * line_path[i].sx_ + 1.0 / 6 * line_path[i].ex_;
        p0[i].y = 1.0 / 6 * line_path[i - 1].sy_ + 2.0 / 3 * line_path[i].sy_ + 1.0 / 6 * line_path[i].ey_;
        p3[i].x = 1.0 / 6 * line_path[i].sx_ + 2.0 / 3 * line_path[i].ex_ + 1.0 / 6 * line_path[i + 1].ex_;
        p3[i].y = 1.0 / 6 * line_path[i].sy_ + 2.0 / 3 * line_path[i].ey_ + 1.0 / 6 * line_path[i + 1].ey_;
    }
    p1[i].x = 2.0 / 3 * line_path[i].sx_ + 1.0 / 3 * line_path[i].ex_;
    p1[i].y = 2.0 / 3 * line_path[i].sy_ + 1.0 / 3 * line_path[i].ey_;
    p2[i].x = 1.0 / 3 * line_path[i].sx_ + 2.0 / 3 * line_path[i].ex_;
    p2[i].y = 1.0 / 3 * line_path[i].sy_ + 2.0 / 3 * line_path[i].ey_;
    return true;
}

sros::core::NavigationPathi_vector
Navigation::bezierFitting(sros::core::NavigationPathi_vector &line_path, bool isAllowBackward) {
    sros::core::NavigationPathi_vector path_bezier;
    long i, n = line_path.size(), index = 0, restLength = n;
    if (1 == n) {
        double turn = -1 * calAngleWithX_axis(line_path[0].ex_ - line_path[0].sx_,
                                              line_path[0].ey_ - line_path[0].sy_);
        if (isAllowBackward) {
            sros::core::Pose curPose = src_sdk->getCurPose();
            double angleToLine = curPose.yaw() - turn + M_PI;
            normalizeAngle(angleToLine);

            if (fabs(angleToLine) < REVERSE_ANGLE) { //
                sros::core::LinePathi line(line_path[0].sx_, line_path[0].sy_, line_path[0].ex_, line_path[0].ey_,
                                           sros::core::PATH_BACKWARD);
                path_bezier.push_back(line);
            } else {
                sros::core::RotatePathi rotate(line_path[0].sx_, line_path[0].sy_, turn);
                sros::core::LinePathi line(line_path[0].sx_, line_path[0].sy_, line_path[0].ex_, line_path[0].ey_);
                path_bezier.push_back(rotate);
                path_bezier.push_back(line);
            }
            //displayPath(path_bezier);
        } else {
            sros::core::RotatePathi rotate(line_path[0].sx_, line_path[0].sy_, turn);
            sros::core::LinePathi line(line_path[0].sx_, line_path[0].sy_, line_path[0].ex_, line_path[0].ey_);
            path_bezier.push_back(rotate);
            path_bezier.push_back(line);
        }
        return path_bezier;
    }
    Geometry_Point *p0 = new Geometry_Point[n], *p1 = new Geometry_Point[n];
    Geometry_Point *p2 = new Geometry_Point[n], *p3 = new Geometry_Point[n];
    double radius;
    int width, height, obs_x, obs_y;
    width = navigation_map_->getMapSize().x, height = navigation_map_->getMapSize().y;


    for (i = 0; i < n; i++) {
        if (updateBezier(p0, p1, p2, p3, i, index, restLength, n, line_path)) {

            if (bezierWalkable(&grid_map_, p0[i], p1[i], p2[i], p3[i], obs_x, obs_y, radius, false, width, height)) {
                sros::core::BezierPathi bezier(dci(p0[i].x), dci(p0[i].y), dci(p1[i].x), dci(p1[i].y), dci(p2[i].x),
                                               dci(p2[i].y), dci(p3[i].x), dci(p3[i].y), radius);
//                std::cout << "i: " << i << " MaxCurvature: " << bezier.radius_ << std::endl;
                if (!path_bezier.empty() && (path_bezier.end() - 1)->type_ == sros::core::PATH_LINE) {
                    double turn = -1 * calSplineAngle(p0[i], p1[i], p2[i], p3[i], 0);
                    sros::core::RotatePathi rotate(line_path[i].sx_, line_path[i].sy_, turn);
                    path_bezier.push_back(rotate);
                }
                path_bezier.push_back(bezier);
                ++index;
            } else {
                //当拟合产生的贝塞尔路径上有障碍时,把上一段路径改为原地旋转且更新路径,当前路径为直线
                if (!path_bezier.empty()) {
                    (path_bezier.end() - 1)->ex_ = line_path[i].sx_;
                    (path_bezier.end() - 1)->ey_ = line_path[i].sy_;
                    double turn = -1 * calAngleWithX_axis(line_path[i].ex_ - line_path[i].sx_,
                                                          line_path[i].ey_ - line_path[i].sy_);
                    sros::core::RotatePathi rotate(line_path[i].sx_, line_path[i].sy_, turn);
                    path_bezier.push_back(rotate);
                }
                path_bezier.push_back(line_path[i]);
                index = 0;
                restLength = n - i - 1;
            }
        } else {
            //当无法产生贝塞尔控制点时(可能到了路径的首位两段),走原地旋转和直线
            if (!path_bezier.empty()) {
                (path_bezier.end() - 1)->ex_ = line_path[i].sx_;
                (path_bezier.end() - 1)->ey_ = line_path[i].sy_;
                double turn = -1 * calAngleWithX_axis(line_path[i].ex_ - line_path[i].sx_,
                                                      line_path[i].ey_ - line_path[i].sy_);
                sros::core::RotatePathi rotate(line_path[i].sx_, line_path[i].sy_, turn);
                path_bezier.push_back(rotate);
            }
            path_bezier.push_back(line_path[i]);
        }
    }

    if (!path_bezier.empty()) {
        double first_turn;
        sros::core::NavigationPath<int> p = path_bezier[0];
        if (p.type_ == sros::core::PATH_BEZIER)
            first_turn = -1 * calSplineAngle(p0[0], p1[0], p2[0], p3[0], 0);
        else if (p.type_ == sros::core::PATH_LINE)
            first_turn = -1 * calAngleWithX_axis(p.ex_ - p.sx_, p.ey_ - p.sy_);
        sros::core::Pose curPose = src_sdk->getCurPose();
        double angleToLine = curPose.yaw() - first_turn + M_PI;
        normalizeAngle(angleToLine);
        if (isAllowBackward) {
            if (fabs(angleToLine) < REVERSE_ANGLE) { //
                sros::core::NavigationPathi_vector::iterator it = path_bezier.begin();
                for (; it != path_bezier.end(); it++) {
                    if (it->type_ == sros::core::PATH_ROTATE)
                        break;
                    else
                        it->setPathDirection(sros::core::PATH_BACKWARD);
                }
            } else {
                sros::core::RotatePathi temp(p.sx_, p.sy_, first_turn);
                path_bezier.insert(path_bezier.begin(), temp);
            }
        } else {
            sros::core::RotatePathi temp(p.sx_, p.sy_, first_turn);
            path_bezier.insert(path_bezier.begin(), temp);
        }
    }


    delete[] p0;
    delete[] p1;
    delete[] p2;
    delete[] p3;
    //displayPath(path_bezier);
    return path_bezier;
}

void Navigation::calOffset(Point &up_left, Point &down_right, int cx, int cy, int width, int height, int radius) {
    int left_x, right_x, top_y, bottom_y;
    left_x = cx - radius;
    right_x = cx + radius;
    top_y = cy - radius;
    bottom_y = cy + radius;
    if (left_x < 0)
        left_x = 0;
    if (right_x >= width)
        right_x = width - 1;
    if (top_y < 0)
        top_y = 0;
    if (bottom_y >= height)
        bottom_y = height - 1;
    up_left.x = left_x;
    up_left.y = top_y;
    down_right.x = right_x - left_x;
    down_right.y = bottom_y - top_y;
}

Obstacle_vector Navigation::getInflationObstacle(Obstacle_vector &src, int inflation_factor,
                                                 int width, int height, int cx, int cy) {
    Obstacle_vector dst;
    Obstacle_vector::iterator it;
    Point lu, dr;
    calOffset(lu, dr, cx, cy, width, height, ERODE_RADIUS);
    for (it = src.begin(); it != src.end(); ++it) {
        *it = *it - lu;
    }
    //dr = dr - lu;
    dst = inflationObstacles(src, dr.x, dr.y, INFLATION_TYPE, inflation_factor);
    for (it = dst.begin(); it != dst.end(); ++it) {
        *it = *it + lu;
    }
    //Fixed: recovery _radar_obstacle.
    for (it = src.begin(); it != src.end(); ++it) {
        *it = *it + lu;
    }

    return dst;
}

bool Navigation::getViewPointFromRadar(sros::core::Location_Vector &radar_obstacle,
                                       int width, int height,
                                       int offset_x, int offset_y,
                                       int cx, int cy) {
    sros::core::Location p;
    sros::core::Location_Vector::iterator it;
    Obstacle_vector::iterator itt;
    int x, y;

    _radar_obstacle.clear(); // 放在外面避免自由导航多次规划时间隔时间不错过OBSTACLE_MEMORIZE_TIME 导致之前的障碍物不清除。

    int64_t now = std::chrono::steady_clock::now().time_since_epoch().count();
    if ((now - _last_clear_time) / 1000000000.0 - OBSTACLE_MEMORIZE_TIME > EPSINON) {
//        std::cout << "间隔时间: " << (now - _last_clear_time) / 1000000000.0 << std::endl;
        _last_clear_time = now;
        _radar_obstacle.clear();
    }

    for (it = radar_obstacle.begin(); it != radar_obstacle.end(); ++it) {
        p = *(it);
//        x = dci(p.x() * METER_TO_CM / LENGTH_PER_GRID);
//        y = dci(p.y() * METER_TO_CM / LENGTH_PER_GRID);
        convertMapCoords(offset_x, offset_y, p.x(), p.y(), &x, &y);
        if (x < 0 || y < 0 || x >= width || y >= height)
            continue;
        //视野中选出在规定范围内的点
        if (pointDistance(cx, cy, x, y) < ERODE_RADIUS) {
            sros::map::Point obs(x, y);
            bool isSame = false;
            for (itt = _radar_obstacle.begin(); itt != _radar_obstacle.end(); ++itt) {
                if (*itt == obs) {
                    isSame = true;
                    break;
                }
            }
            if (!isSame)
                _radar_obstacle.push_back(obs);
        }

    }

    if (_radar_obstacle.empty()) {
        return false;
    }
    return true;
}

Obstacle_vector Navigation::chooseObstacleFromRadar(sros::map::PyramidNavMap_ptr map_data,
                                                    Obstacle_vector &radar_point, int width, int height) {
    Obstacle_vector::iterator it;
    sros::map::Point p;
    Obstacle_vector choosed_obstacle_vector;
    choosed_obstacle_vector.clear();
    for (it = radar_point.begin(); it != radar_point.end(); ++it) {
        p = *it;
        if (map_data->isGridFree(p.x,p.y)) {
            choosed_obstacle_vector.push_back(p);
        }
    }
    return choosed_obstacle_vector;
}

void Navigation::addBufferToGrid(int width, int height, sros::map::NavMapData_ptr matrix) {
    struct node *tgrid;
    int i, j;
    for (i = 0; i < width; ++i) {
        for (j = 0; j < height; ++j) {
            tgrid = getNodeAt(&grid_map_, i, j);
            if (1 == matrix[j * width + i] &&
                GETWALKABLE(tgrid->grid_layered_walkable_, STATIC_WALKABLE_TRUE)) {
                SETWALKABLE(tgrid->grid_layered_walkable_, STATIC_BUFFER_TRUE, true);
                SETWALKABLE(tgrid->grid_layered_walkable_, WALKABLE_FALSE, false);
            }
        }
    }
}

void Navigation::addObstacleToGrid(struct grid *gd, Obstacle_vector &obs) {
    Point p;
    Obstacle_vector::iterator it;
    for (it = obs.begin(); it != obs.end(); ++it) {
        p = *it;
        SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, DYNAMIC_WALKABLE_FALSE, false);
        SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, WALKABLE_FALSE, false);
    }
    return;
}
void Navigation::addObstacleToGrid1(struct grid *gd, Obstacle_vector &obs) {
    Point p;
    Obstacle_vector::iterator it;
    for (it = obs.begin(); it != obs.end(); ++it) {
        p = *it;
        gd->nodes[p.y][p.x].grid_layered_walkable_=INIT_WALKABLE_FALSE;
    }
    return;
}

void Navigation::addObstacleBufferToGrid(struct grid *gd, Obstacle_vector &obs) {
    Point p;
    Obstacle_vector::iterator it;
    for (it = obs.begin(); it != obs.end(); ++it) {
        p = *it;
        if (GETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, DYNAMIC_WALKABLE_TRUE)) {
            SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, DYNAMIC_BUFFER_TRUE, true);
            SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, WALKABLE_FALSE, false);
        }
    }
    return;
}

void Navigation::delObstacleFromGrid(struct grid *gd, Obstacle_vector &obs) {
    bool walkable;
    Point p;
    Obstacle_vector::iterator it;
    for (it = obs.begin(); it != obs.end(); ++it) {
        p = *it;
        SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, DYNAMIC_WALKABLE_TRUE, true);
        SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, DYNAMIC_BUFFER_FALSE, false);
        walkable = GETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, STATIC_WALKABLE_TRUE) &
                   !GETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, STATIC_BUFFER_TRUE);
        setWalkableAt(gd, p.x, p.y, walkable);
    }
    return;
}

void Navigation::addViewObstacleToGrid(struct grid *gd, Obstacle_vector &obs) {
    Point p;
    Obstacle_vector::iterator it;
    for (it = obs.begin(); it != obs.end(); ++it) {
        p = *it;
        SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, VIEW_WALKABLE_FALSE, false);
    }
    return;
}

void Navigation::delViewObstacleFromGrid(struct grid *gd, Obstacle_vector &obs) {
    Point p;
    Obstacle_vector::iterator it;
    for (it = obs.begin(); it != obs.end(); ++it) {
        p = *it;
        SETWALKABLE(gd->nodes[p.y][p.x].grid_layered_walkable_, VIEW_WALKABLE_TRUE, true);
    }
    return;
}

void Navigation::mendStartPoint(struct grid *gd, bool isErode, int startx, int starty, int w, int h) {
    bool walkable;
    int x, y;
    for (int i = -1 * MENDING_RADIUS; i <= MENDING_RADIUS; ++i) {
        x = startx + i;
        if (x < 0 || x >= w)
            continue;
        for (int j = -1 * MENDING_RADIUS; j <= MENDING_RADIUS; ++j) {
            y = starty + j;
            if (y < 0 || y >= h)
                continue;
            if (isErode) {
                SETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, WALKABLE_TRUE, true);
            } else {
                //恢复网格信息
                walkable = GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, DYNAMIC_WALKABLE_TRUE) &
                           GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, STATIC_WALKABLE_TRUE) &
                           !GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, STATIC_BUFFER_TRUE) &
                           !GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, DYNAMIC_BUFFER_TRUE);
                setWalkableAt(gd, x, y, walkable);
            }
        }
    }
}

sros::core::NavigationPath_vector Navigation::navigate(int startx, int starty,
                                                       int endx, int endy, double yaw,
                                                       NAV_PATH_TYPE type, bool isFirstNavigate, bool isAllowBackward,bool localplan_or_not) {
    sros::core::NavigationPathi_vector paths;
    sros::core::NavigationPath_vector ans_path;
    struct neighbor_xy_list *path_head = NULL;

    if (startx == endx && starty == endy) {
        printf("起点与终点重合！\n");
        sros::core::RotatePath zero_rotate(yaw);
        ans_path.push_back(zero_rotate);
        return ans_path;
    }
    if (!GETWALKABLE(grid_map_.nodes[endy][endx].grid_layered_walkable_, WALKABLE_TRUE)) {
        //printf("无法到达目标点，目标点不可走.\n");
        LOG(INFO)<<"！！！！！！无法到达目标点，目标点不可走.";
        failed_code_ = ERROR_CODE_FREE_NAV_DST_POSE_UNWALKABLE;
        return ans_path;
    }

    //为防止机器在缓冲区附近，使起点周围正方形区域可走
    //mendStartPoint(&grid_map_, true, startx, starty, navigation_map_->getMapSize().x,
   //                navigation_map_->getMapSize().y);
    path_head = findPath(&grid_map_, startx, starty, endx, endy,localplan_or_not);
    //复原起点起点周围区域地图
   // mendStartPoint(&grid_map_, false, startx, starty, navigation_map_->getMapSize().x,
     //              navigation_map_->getMapSize().y);

    if (path_head == NULL) {
        LOG(INFO)<<"！！！！！！！！！找不到合适的路径.";
        failed_code_ = ERROR_CODE_FREE_NAV_NO_WAY;
        return ans_path;
    }

    paths = getLinePaths(&grid_map_, path_head, isFirstNavigate);

    if (LINE_ROTATE_TYPE == type || LINE_ARC_TYPE == type) {
        if (LINE_ROTATE_TYPE == type)
            ROUND_OFF_ANGLE = 180.0;
        else
            ROUND_OFF_ANGLE = 10.0;
        paths = dealPaths(&grid_map_, paths);
    } else if (BEZIER_TYPE == type) {
        paths = dividePathIntoPieces(paths);
        paths = bezierFitting(paths, isAllowBackward);
    }

    if (yaw != yaw) yaw = 0.0; //if yaw is -nan then yaw = 0.0;
    sros::core::RotatePathi rotate(endx, endy, yaw);
    paths.push_back(rotate);

    deleteRotateFromPaths(paths);

    neighbor_xy_clean(path_head);
    ans_path = changeToNavigationPath(paths);

    displayPath(paths);

    return ans_path;
}

bool Navigation::setParam(std::string name, std::string value) {
    if ("nav.length_per_grid" == name) {
        LENGTH_PER_GRID = std::stoi(value);

        STOP_DISTANCE /= LENGTH_PER_GRID;
        SLOW_DISTANCE /= LENGTH_PER_GRID;
        LONGEST_DIVIDED_LENGTH /= LENGTH_PER_GRID;
        std::cout << "地图分辨率设置为: " << LENGTH_PER_GRID << "cm" << std::endl;

    } else if ("nav.vehicle_width" == name) {
        car_width_ = (int) (std::stod(value) * METER_TO_CM);
        std::cout << "车体宽参数设置为: " << car_width_ << std::endl;

    } else if ("nav.vehicle_length" == name) {
        car_length_ = (int) (std::stod(value) * METER_TO_CM);
        std::cout << "车体长参数设置为: " << car_length_ << std::endl;

    } else if ("nav.stop_distance" == name) {
        STOP_DISTANCE = (int) (std::stod(value) * METER_TO_CM / LENGTH_PER_GRID);
        std::cout << "停止距离参数设置为: " << STOP_DISTANCE << std::endl;

    } else if ("nav.slow_distance" == name) {
        SLOW_DISTANCE = (int) (std::stod(value) * METER_TO_CM / LENGTH_PER_GRID);
        std::cout << "减速距离参数设置为: " << SLOW_DISTANCE << std::endl;

    } else if ("nav.obstacle_memorize_time" == name) {
        OBSTACLE_MEMORIZE_TIME = std::stod(value);
        std::cout << "记忆时间参数修改为: " << OBSTACLE_MEMORIZE_TIME << std::endl;

    } else if ("nav.ignore_angle" == name) {
        IGNORE_ANGLE = (std::stod(value));
        std::cout << "直线路径忽略角度设置为: " << IGNORE_ANGLE << std::endl;

    } else if ("nav.ignore_line" == name) {
        IGNORE_LINE = (int) (std::stod(value) / LENGTH_PER_GRID);
        std::cout << "忽略直线长度设置为: " << IGNORE_LINE << std::endl;

    } else if ("nav.buffer_length" == name) {
        BUFFER_LENGTH = std::stoi(value);
        std::cout << "缓冲区长度参数设置为: " << BUFFER_LENGTH << std::endl;

    } else if ("nav.longest_divide_path" == name) {
        LONGEST_DIVIDED_LENGTH = (int) (std::stod(value) / LENGTH_PER_GRID);
        std::cout << "最长路径长度设置为: " << LONGEST_DIVIDED_LENGTH << std::endl;

    } else if ("nav.width_delta_obstacle" == name) {
        INFLATION_FOR_SAFETY = (int) (std::stod(value));
        std::cout << "车的避障缓冲区宽度: " << INFLATION_FOR_SAFETY << std::endl;

    } else if ("nav.inflation_type" == name) {
        INFLATION_TYPE = StructuringElementCircle; // 默认值为Circle
        if ("Square" == value) {
            INFLATION_TYPE = StructuringElementSquare;
        } else if ("Circle" == value) {
            INFLATION_TYPE = StructuringElementCircle;
        } else if ("Diamond" == value) {
            INFLATION_TYPE = StructuringElementDiamond;
        } else if ("Line" == value) {
            INFLATION_TYPE = StructuringElementLine;
        }

    }
    auto &s = sros::core::Settings::getInstance();



//    else if ("nav.enable_local_planner" == name) {
//        enable_local_planner = (int) (std::stod(value));
//        std::cout << "是否允许开启局部路径规划: " << enable_local_planner << std::endl;
//
//    }

    return true;
}

bool Navigation::loadMapFile(const std::string file_path, bool enable_load_map_data) {
    LOG(INFO) << "Navigation Start Loading Map File: " << file_path << " enable_load_map_data: " << enable_load_map_data;
    if (!navigation_map_->loadFile(file_path, enable_load_map_data)) {
        return false;
    }
    std::cout << "Loading Origin File Complete" << std::endl;

    return true;
}

void Navigation::initNavConfig(NavConfig& cfg) {
    cfg_ = &cfg;
}


void Navigation::initMapData() {

    if(grid_map_.nodes  !=nullptr){
        LOG(INFO)<<"DDDDD"<<height;
        for (int i = 0; i < height; i++) {
            delete[] grid_map_.nodes[i];
        }
        delete[] grid_map_.nodes;
    }
//    cfg_->nav_resolution = 2;
    LENGTH_PER_GRID = cfg_->nav_resolution;
    //由于从navigation_map_->getMapData() 得到的栅格地图时2cm。根据规划需要可以通过配置参数
    // 设置为4cm 8cm等分辨率。 为了不占用更多的内存资源。在
    //inflation_map_data.reset(new sros::map::NavMapNode_t[width * height]);
    //初始化时，就采用新的分辨率cfg_->nav_resolution 表示新的分辨率。
    //技术方法，现将地图坐标转换为全局坐标，。然后在将全局坐标转换为新分辨率的地图坐标。搞定。

    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;
    float resolution = navigation_map_->getMapResolution();

    //////////////////////////////////自动调整栅格地图分辨率
    // 通过读取原始地图参数，计算地图面积，当地图面积超过阈值（sizeThresh）时，调整生成网格地图的分辨率，减小内存占用
    int size_thresh = 60000;
    const auto current_map_size = static_cast<int>(width * height * resolution * resolution * 1e-4);
    LOG(INFO) << "origin resolution: " << resolution << "、 " << cfg_->nav_resolution << " origin size: " << current_map_size;
    if(current_map_size < size_thresh) {
        cfg_->nav_resolution = 4;
    }
    else {
        cfg_->nav_resolution = 4 + int(current_map_size / size_thresh) * 4;
    }
    //    cfg_->nav_resolution = 2.85;    //当分辨率小于此值，路径搜索会出现问题
    LENGTH_PER_GRID = cfg_->nav_resolution;
    LOG(INFO) << "new resolution: " << cfg_->nav_resolution;
    //////////////////////////////////自动调整栅格地图分辨率

    int offset_x,offset_y;
    offset_x = navigation_map_->getMapZeroOffsetX();
    offset_y = navigation_map_->getMapZeroOffsetY();
    double world_x,world_y;
    int new_offset_x,new_offset_y;
    world_x = (0 - offset_x) * 2 / 100.0;
    world_y = (offset_y - 0) * 2 / 100.0; 
    new_offset_x  = dci(0-world_x * 100.0 / cfg_->nav_resolution);
    new_offset_y  = dci(world_y * 100.0 / cfg_->nav_resolution);

    int new_width,new_height;
    world_x = (width-1 - offset_x) * 2 / 100.0;
    world_y = (offset_y - height+1) * 2 / 100.0; 
    new_width = dci(new_offset_x + world_x * 100.0 / cfg_->nav_resolution)+1;
    new_height = dci(new_offset_y - world_y * 100.0 / cfg_->nav_resolution)+1;

    sros::map::PyramidNavMap_ptr inflation_map_data1;
    float resolution_f = cfg_->nav_resolution/100.0f;
    float origin_reso = navigation_map_->getMapResolution()/100.0f;
    // 膨胀尺寸不同，hybrid A※ 主要用没有膨胀的图
    inflation_map_data.reset(
        new sros::map::PyramidNavMap<NavMapNode_t>(new_width, new_height, offset_x, offset_y, resolution_f));
    //全局规划，通过A※生成路径需要
    inflation_map_data1.reset(
        new sros::map::PyramidNavMap<NavMapNode_t>(new_width, new_height, offset_x, offset_y, resolution_f));
    auto tmp_mapdata = navigation_map_->getMapData();
    LOG(INFO)<<"height:"<<height<<","<<width;
    int cx,cy;
    int index = 0;
    tmp_mapdata->cpToCoarseMap(inflation_map_data);
    LOG(INFO) << "old width: " << width << "; height: " << height << "," << index << "," << width * height;
    LOG(INFO) << "new width: " << new_width << "; height: " << new_height;

    LOG(INFO) << "old offsetx: " << offset_x << "; y: " << offset_y;
    LOG(INFO) << "new offsetx: " << new_offset_x << "; y: " << new_offset_y;

    // 将之前的地图数据全都覆盖掉
    Point offsetpoint(new_offset_x,new_offset_y);
    navigation_map_->setMapZeroOffset(offsetpoint);

    Point width_height(new_width,new_height);
    navigation_map_->setMapSize(width_height);

    navigation_map_->setMapResolution(cfg_->nav_resolution);

    navigation_map_->setMapData(inflation_map_data);

    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;

    LOG(INFO)<<"Inflation_map Initial Complete";
    float tmp_width = sros::core::Settings::getInstance().getValue<double>("nav.vehicle_width", 0.6);

    int inflation_factor = std::max(cfg_->map_expansion_size,(float)(tmp_width+0.20))*100.0f/cfg_->nav_resolution/2.0f;

    //原始地图膨胀定位误差的值，这样可使得定位产生的跳点不计入动态障碍
    //inflationStaticMap(navigation_map_, navigation_map_->getMapData(), width, height, 1, INFLATION_TYPE, 1);
    LOG(INFO)<<"Origin Inflation Complete" ;

    inflationStaticMap(navigation_map_->getMapData(), inflation_map_data1, width, height, inflation_factor, INFLATION_TYPE, 1);

    LOG(INFO)<<"Static Map Inflation Complete" ;

    LOG(INFO)<<"Start Create Grid" ;

    grid_map_ = createGrid(width, height, inflation_map_data1); /* Create a new grid */
    //localgrid_map,主要用于主动绕障中判断，膨胀没必要和grid_map_一样。
    int64_t time_first = sros::core::util::get_time_in_ms();
   // localgrid_map_=createGrid(width, height, navigation_map_->getMapData()); /* Create a new grid */

   // localgrid_map_=createGrid(width, height,navigation_map_->getMapData()); /* Create a new grid */
    int64_t time_second = sros::core::util::get_time_in_ms();
    LOG(INFO)<<"创建localgridmap地图耗时ms;"<<time_second-time_first;
    LOG(INFO)<< "Create Grid Complete" ;

//    inflationStaticMap(navigation_map_, inflation_map_data, width, height, BUFFER_LENGTH, INFLATION_TYPE, 2);
//    LOG(INFO)<< "Buffer Inflation Complete" ;

  //  addBufferToGrid(width, height, inflation_map_data);
  //  LOG(INFO)<< "Add Buffer Complete" ;
}

void Navigation::releaseMapData() {
    //更换地图之前，清空旧的地图数据，减少内存占用
    LOG(INFO) << "Enter release map data";
    navigation_map_.reset(new sros::map::NavigationMap());
    inflation_map_data.reset();
    if(grid_map_.nodes  != nullptr){
        LOG(INFO) << "更换地图之前，释放网格地图所有数据";
        for (int i = 0; i < height; i++) {
            delete[] grid_map_.nodes[i];
        }
        delete[] grid_map_.nodes;
    }
}

// 返回d2相对与d1需要旋转的角度(逆时针为正)
double Navigation::getTurnAngle(sros::core::Pose &d1, sros::core::Pose &d2) const {
    double d1_len = sqrt(d1.x() * d1.x() + d1.y() * d1.y());
    double d2_len = sqrt(d2.x() * d2.x() + d2.y() * d2.y());

    double cos_theta = (d1.x() * d2.x() + d1.y() * d2.y()) / (d1_len * d2_len);

    // 不能直接调用acos(cos_theta)获取theta值
    // 由于浮点运算的精度限制，acos(cos_theta)可能返回nan
    double theta = 0.0;
    const double LOCAL_EPSILON = 0.00001;
    if (fabs(1 - cos_theta) < LOCAL_EPSILON) {
        theta = 0.0;
    } else if (fabs(cos_theta) < LOCAL_EPSILON) {
        theta = M_PI_2;
    } else if (fabs(-1 - cos_theta) < LOCAL_EPSILON) {
        theta = M_PI;
    } else {
        theta = acos(cos_theta);
    }

    double d2_x_d1 = d2.x() * d1.y() + d1.y() * d2.x();

    return (d2_x_d1 > 0) ? theta : -1 * theta;
}

sros::core::Pose Navigation::getThePoseAfterSimulateMoveInLine(sros::core::Pose curPose,
                                                               const sros::core::NavigationPath<double> &navLinePath) {
    sros::core::Pose nextStepPathInline;
    if (navLinePath.ex_ == navLinePath.sx_) {
        int sign = (navLinePath.ey_ > navLinePath.sy_) ? 1 : -1;
        nextStepPathInline.x() = curPose.x();
        nextStepPathInline.y() = curPose.y() + sign * SIMULATE_WALK_STEP_LENGTH / METER_TO_CM;
        nextStepPathInline.yaw() = M_PI / 2 * sign;
        if ((sign > 0) && (nextStepPathInline.y() > navLinePath.ey_)) {
            nextStepPathInline.y() = navLinePath.ey_;
        } else if ((sign < 0) && (nextStepPathInline.y() < navLinePath.ey_)) {
            nextStepPathInline.y() = navLinePath.ey_;
        }
    } else if (navLinePath.ey_ == navLinePath.sy_) {
        int sign = (navLinePath.ex_ > navLinePath.sx_) ? 1 : -1;
        nextStepPathInline.x() = curPose.x() + sign * SIMULATE_WALK_STEP_LENGTH / METER_TO_CM;
        nextStepPathInline.y() = curPose.y();
        nextStepPathInline.yaw() = 0;
    } else {
        sros::core::Pose temp1_pose;
        sros::core::Pose temp2_pose;

        temp1_pose.x() = 1;
        temp1_pose.y() = 0;

        temp2_pose.x() = navLinePath.ex_ - curPose.x();
        temp2_pose.y() = navLinePath.ey_ - curPose.y();
        double slope = getTurnAngle(temp2_pose, temp1_pose);//计算车位置到目标位置向量　与　Ｘ轴的角度
        double arcWithRad = curPose.yaw();
        nextStepPathInline.x() = curPose.x() + SIMULATE_WALK_STEP_LENGTH * cos(slope) / METER_TO_CM;
        nextStepPathInline.y() = curPose.y() + SIMULATE_WALK_STEP_LENGTH * sin(slope) / METER_TO_CM;
        nextStepPathInline.yaw() = arcWithRad;
        // LOG(INFO)<< "!!!!!!!!!arcWithRad " <<arcWithRad << " slope " << slope;
    }
    /*在位置末端的时候  车体角度和路线的角度可能并不匹配*/
    if (std::hypot((nextStepPathInline.x() - navLinePath.ex_), (nextStepPathInline.y() - navLinePath.ey_)) < 0.05) {
        nextStepPathInline.x() = navLinePath.ex_;
        nextStepPathInline.y() = navLinePath.ey_;
    }
    if ((nextStepPathInline.x() >= navLinePath.ex_ || nextStepPathInline.y() >= navLinePath.ey_) &&
        (navLinePath.ex_ > navLinePath.sx_) && (navLinePath.ey_ > navLinePath.sy_)) {
        nextStepPathInline.x() = navLinePath.ex_;
        nextStepPathInline.y() = navLinePath.ey_;
    } else if ((nextStepPathInline.x() <= navLinePath.ex_ || nextStepPathInline.y() <= navLinePath.ey_) &&
               (navLinePath.ex_ < navLinePath.sx_) && (navLinePath.ey_ < navLinePath.sy_)) {
        nextStepPathInline.x() = navLinePath.ex_;
        nextStepPathInline.y() = navLinePath.ey_;
    } else if ((nextStepPathInline.x() <= navLinePath.ex_ || nextStepPathInline.y() >= navLinePath.ey_) &&
               (navLinePath.ex_ < navLinePath.sx_) && (navLinePath.ey_ > navLinePath.sy_)) {
        nextStepPathInline.x() = navLinePath.ex_;
        nextStepPathInline.y() = navLinePath.ey_;
    } else if ((nextStepPathInline.x() >= navLinePath.ex_ || nextStepPathInline.y() <= navLinePath.ey_) &&
               (navLinePath.ex_ > navLinePath.sx_) && (navLinePath.ey_ < navLinePath.sy_)) {
        nextStepPathInline.x() = navLinePath.ex_;
        nextStepPathInline.y() = navLinePath.ey_;
    }
    return nextStepPathInline;
}

sros::core::Pose Navigation::getThePoseAfterSimulateMoveInRotate(sros::core::Pose curPose, double rotate_angle) {
    sros::core::Pose pos;
    // LOG(INFO)<<"*******getThePoseAfterSimulateMoveInRotate********";
    pos.yaw() = curPose.yaw() + rotate_angle;
    pos.x() = curPose.x();
    pos.y() = curPose.y();
    return pos;
}

sros::core::Pose Navigation::getThePoseAfterSimulateMoveInCircle(sros::core::Pose curPose,
                                                                 sros::core::NavigationPath<double> &navCirclePath) {
    sros::core::Pose pos;
    double slopeCurPosToCircleCentre = 0;
    //printf("enter getThePoseAfterSimulateMoveInCircle\n");

    slopeCurPosToCircleCentre = atan((curPose.y() - navCirclePath.cy_) / (curPose.x() - navCirclePath.cx_));
    double radLength = fabs(navCirclePath.rotate_angle_ * navCirclePath.radius_);
    int countLimit = ceil(radLength / SIMULATE_WALK_STEP_LENGTH);
    static int count = 0;                       //need to be improved

    double radChangedInOneStep = SIMULATE_WALK_STEP_LENGTH / navCirclePath.radius_;
    count++;
    pos.yaw() = curPose.yaw() + radChangedInOneStep;
    pos.x() = (navCirclePath.radius_ * cos(slopeCurPosToCircleCentre + radChangedInOneStep) + navCirclePath.cx_);
    pos.y() = (navCirclePath.radius_ * cos(slopeCurPosToCircleCentre + radChangedInOneStep) + navCirclePath.cy_);

    if (count >= countLimit) {
        pos.x() = navCirclePath.ex_;
        pos.y() = navCirclePath.ey_;
        pos.yaw() = navCirclePath.e_facing_;
        count = 0;
    }
    //printf("exit getThePoseAfterSimulateMoveInCircle\n");

    return pos;
}

sros::core::Pose Navigation::getThePoseAfterSimulateMoveInBezier(sros::core::Pose curPose,
                                                                 sros::core::NavigationPath<double> &navBezierPath) {
    static double robotNearestT = 0;
    double robotNextStepT = 0;
    double bezier1X1, bezier1Y1, bezier2X1, bezier2Y1;//计算二阶贝塞尔曲线的临时变量
    Geometry_Point bezierStart, bezierEnd, bezierControl1, bezierControl2;
    float VecPlus = -1;
    double bezierLength;
    sros::core::Pose nearPoint, nexStepPos;
    sros::core::Pose vectorFromCurToNearPoint;
    sros::core::Pose vectorTangent;

    static int prePath_x = 0;
    if (prePath_x != navBezierPath.sx_) {
        robotNearestT = 0;
    }
    prePath_x = navBezierPath.sx_;

    bezierStart.x = navBezierPath.sx_;
    bezierStart.y = navBezierPath.sy_;
    bezierEnd.x = navBezierPath.ex_;
    bezierEnd.y = navBezierPath.ey_;
    bezierControl1.x = navBezierPath.cx_;
    bezierControl1.y = navBezierPath.cy_;
    bezierControl2.x = navBezierPath.dx_;
    bezierControl2.y = navBezierPath.dy_;
    robotNearestT = getTheNearest_T_inBezier(curPose, navBezierPath, robotNearestT);

//    LOG(INFO)<<"****** exit while loop in getThePoseAfterSimulateMoveInBezier *******";
    bezierLength = bezier_length(bezierStart, bezierControl1, bezierControl2, bezierEnd);

    robotNextStepT = robotNearestT + (double) SIMULATE_WALK_STEP_LENGTH / (bezierLength * 100);
    if (robotNextStepT > 1) {
        robotNextStepT = 1;
    } else if (robotNextStepT < 0) {
        robotNextStepT = 0;
    }
    bezier1X1 = (1 - robotNextStepT) * (1 - robotNextStepT) * bezierStart.x
                + 2 * (1 - robotNextStepT) * robotNextStepT * bezierControl1.x
                + robotNextStepT * robotNextStepT * bezierControl2.x;
    bezier1Y1 = (1 - robotNextStepT) * (1 - robotNextStepT) * bezierStart.y
                + 2 * (1 - robotNextStepT) * robotNextStepT * bezierControl1.y
                + robotNextStepT * robotNextStepT * bezierControl2.y;
    bezier2X1 = (1 - robotNextStepT) * (1 - robotNextStepT) * bezierControl1.x
                + 2 * (1 - robotNextStepT) * robotNextStepT * bezierControl2.x
                + robotNextStepT * robotNextStepT * bezierEnd.x;
    bezier2Y1 = (1 - robotNextStepT) * (1 - robotNextStepT) * bezierControl1.y
                + 2 * (1 - robotNextStepT) * robotNextStepT * bezierControl2.y
                + robotNextStepT * robotNextStepT * bezierEnd.y;

    nexStepPos.x() = (1 - robotNextStepT) * bezier1X1 + robotNextStepT * bezier2X1;
    nexStepPos.y() = (1 - robotNextStepT) * bezier1Y1 + robotNextStepT * bezier2Y1;

    nexStepPos.yaw() = atan((bezier2Y1 - bezier1Y1) / (bezier2X1 - bezier1X1));

    //printf("exit getThePoseAfterSimulateMoveInBezier\n");
    return nexStepPos;
}

double Navigation::getTheNearest_T_inBezier(sros::core::Pose curPose, sros::core::NavigationPath<double> &navBezierPath,
                                            double robotNearestT) {
    Geometry_Point bezierStart, bezierEnd, bezierControl1, bezierControl2;
    float VecPlus = -1;
    double bezier1X1, bezier1Y1, bezier2X1, bezier2Y1;//计算二阶贝塞尔曲线的临时变量
    sros::core::Pose vectorFromCurToNearPoint;
    sros::core::Pose vectorTangent;
    sros::core::Pose nearPoint;
    bezierStart.x = navBezierPath.sx_;
    bezierStart.y = navBezierPath.sy_;
    bezierEnd.x = navBezierPath.ex_;
    bezierEnd.y = navBezierPath.ey_;
    bezierControl1.x = navBezierPath.cx_;
    bezierControl1.y = navBezierPath.cy_;
    bezierControl2.x = navBezierPath.dx_;
    bezierControl2.y = navBezierPath.dy_;
    //printf("--------------------------------bezierStart.x %f bezierStart.y %f\n",bezierStart.x,bezierStart.y);
//    LOG(INFO)<<"enter getThePoseAfterSimulateMoveInBezier";
    while (VecPlus < 0) {

        robotNearestT = robotNearestT + 0.001f;
        if (robotNearestT > 1) {
            robotNearestT = 1;
            break;
        } else if (robotNearestT < 0) {
            robotNearestT = 0;
            break;
        }
        bezier1X1 = (1 - robotNearestT) * (1 - robotNearestT) * bezierStart.x
                    + 2 * (1 - robotNearestT) * robotNearestT * bezierControl1.x
                    + robotNearestT * robotNearestT * bezierControl2.x;
        bezier1Y1 = (1 - robotNearestT) * (1 - robotNearestT) * bezierStart.y
                    + 2 * (1 - robotNearestT) * robotNearestT * bezierControl1.y
                    + robotNearestT * robotNearestT * bezierControl2.y;
        bezier2X1 = (1 - robotNearestT) * (1 - robotNearestT) * bezierControl1.x
                    + 2 * (1 - robotNearestT) * robotNearestT * bezierControl2.x
                    + robotNearestT * robotNearestT * bezierEnd.x;
        bezier2Y1 = (1 - robotNearestT) * (1 - robotNearestT) * bezierControl1.y
                    + 2 * (1 - robotNearestT) * robotNearestT * bezierControl2.y
                    + robotNearestT * robotNearestT * bezierEnd.y;


        nearPoint.x() = (1 - robotNearestT) * bezier1X1 + robotNearestT * bezier2X1;
        nearPoint.y() = (1 - robotNearestT) * bezier1Y1 + robotNearestT * bezier2Y1;

        vectorFromCurToNearPoint.x() = nearPoint.x() - curPose.x();
        vectorFromCurToNearPoint.y() = nearPoint.y() - curPose.y();

        vectorTangent.x() = bezier2X1 - bezier1X1;
        vectorTangent.y() = bezier2Y1 - bezier1Y1;

        VecPlus = vectorFromCurToNearPoint.x() * vectorTangent.x() + vectorFromCurToNearPoint.y() * vectorTangent.y();
    }
    return robotNearestT;
}

/*move SIMULATE_WALK_STEP_LENGTH , and find the robot pose */
void Navigation::getThePoseAfterSimulateMove(sros::core::Pose curPose,
                                             sros::core::NavigationPath_vector &nav_paths,
                                             verticesOfRobots &fourVerticesWithTwoMiddlePos) {
    //sros::core::NavigationPathi_vector paths = changeToMapPath(nav_paths);
    sros::core::NavigationPath_vector paths = nav_paths;
    sros::core::NavigationPath_vector::iterator it;
    sros::core::NavigationPath<double> p;
    sros::core::Pose nextStepPath;
    int stepCount = 0;
    int lineNumber = 0;
    // NOTE： 旋转的时候，没有其实角度和结束角度，只有一个旋转到的角度，所以需要记录上一次角度，
    // 然后和这个比较，来确认旋转方向和旋转大小，至于旋转180°时，
    // 不需要考虑是顺时针旋转还是逆时针旋转，因为他扫过的地方都为一个圆
    double last_angle = 0; // 上一次pose的角度
    //printf("enter getThePoseAfterSimulateMove stepCount %d \n",stepCount);
    for (it = paths.begin(); it != paths.end(); ++it) {
        p = *it;
//        displayNPath(nav_paths);

        if (it == paths.begin()) {
            nextStepPath = curPose;
            last_angle = curPose.yaw();
        } else {
            last_angle = nextStepPath.yaw();

            nextStepPath.x() = p.sx_;
            nextStepPath.y() = p.sy_;
            nextStepPath.yaw() = p.s_facing_;
        }
//        LOG(INFO) << it - paths.begin() << ". path type: " << p.type_ <<
//                  " (" << nextStepPath.x() << ", " << nextStepPath.y() << ", " << nextStepPath.yaw() << ") stepCount: "
//                  << stepCount;

        switch (p.type_) {
            case sros::core::PATH_LINE: {             //sin and tan　had better only cal once for the path
                while (p.ex_ != nextStepPath.x() || p.ey_ != nextStepPath.y()) {
                    getVerticesOfRobots(nextStepPath, fourVerticesWithTwoMiddlePos, lineNumber);
                    nextStepPath = getThePoseAfterSimulateMoveInLine(nextStepPath, p);
                    stepCount++;
                    if (stepCount >= SIMULATE_WALK_TOTAL_LENGTH / SIMULATE_WALK_STEP_LENGTH) {
                        //LOG(INFO)<<"nextStepPath x "<<nextStepPath.x()<<"nextStepPath y "<<nextStepPath.y() << "lineNumber " <<lineNumber;
                        //LOG(INFO)<<"stepCount exceed\n";
                        return;
                    }

                }
                break;
            }
            case sros::core::PATH_ARC: {
                while (p.ex_ != nextStepPath.x()) {
                    nextStepPath = getThePoseAfterSimulateMoveInCircle(nextStepPath, p);
                    getVerticesOfRobots(nextStepPath, fourVerticesWithTwoMiddlePos, lineNumber);
                    stepCount++;
                    if (stepCount >= SIMULATE_WALK_TOTAL_LENGTH / SIMULATE_WALK_STEP_LENGTH) {
                        //LOG(INFO)<<"step count execeed\n";
                        return;
                    }
                }
                break;
            }
            case sros::core::PATH_BEZIER: {
                while (p.ex_ != nextStepPath.x()) {
                    //printf("p.ex_ %d nextStepPath.x() %f\n",p.ex_,nextStepPath.x());
                    nextStepPath = getThePoseAfterSimulateMoveInBezier(nextStepPath, p);
                    getVerticesOfRobots(nextStepPath, fourVerticesWithTwoMiddlePos, lineNumber);
                    stepCount++;
                    if (stepCount >= SIMULATE_WALK_TOTAL_LENGTH / SIMULATE_WALK_STEP_LENGTH) {
                        //LOG(INFO)<<"stepCount exceed\n";
                        return;
                    }
                }
                break;
            }
            case sros::core::PATH_ROTATE: {                                                 //divide the arc to  PATH_ROTATE_LEAST_NUM_TO_BE_CUT parts
                double need_rotate_angle = last_angle - p.rotate_angle_; // 需要旋转的角度
                if (need_rotate_angle >= M_PI){
                    need_rotate_angle -= M_PI * 2;
                }else if(need_rotate_angle <= -M_PI){
                    need_rotate_angle += M_PI * 2;
                }

                int dividePartNum = abs(need_rotate_angle / RESOLUATION_IN_ROTATE); // 需要旋转的次数
//                if (dividePartNum < PATH_ROTATE_LEAST_NUM_TO_BE_CUT)
//                    dividePartNum = PATH_ROTATE_LEAST_NUM_TO_BE_CUT;
                double resoluationInRad = need_rotate_angle / dividePartNum; // 每次旋转的角度（可以为正数可以为负数）
                //stepCount++;                                                                           //the distance doesnot increase  in PATH_ROTATE
//                LOG(INFO) << "p.rotate_angle: " << p.rotate_angle_;
//                LOG(INFO) << "last_angle: " << last_angle;
//                LOG(INFO) << "need_rotate_angle: " << need_rotate_angle;
//                LOG(INFO) << "dividePartNum: " << dividePartNum;
//                LOG(INFO) << "resoluationInRad: " << resoluationInRad;

                for (int i = 1; i < dividePartNum; i++) {
//                    stepCount++;
                    nextStepPath = getThePoseAfterSimulateMoveInRotate(nextStepPath, resoluationInRad * i);
                    getVerticesOfRobots(nextStepPath, fourVerticesWithTwoMiddlePos, lineNumber);
                }
                if (stepCount >= SIMULATE_WALK_TOTAL_LENGTH / SIMULATE_WALK_STEP_LENGTH) {
                    //LOG(INFO)<<"step count execeed\n";
                    return;
                }
                break;
            }
        }
        lineNumber++;
    }
    //printf("exit getThePoseAfterSimulateMove\n");
}
/*
        2*length+width--------------5---------------width+length
                      --------------------------------
                  <-----------------------------------
                      --------------------------------
                      0--------------4----------------length
*/
//in checkPointsValid , still need to dilate the obstacle 5cm ,safety distance
// 小车的当前位置（curPose）用间隔为1cm的点填充
void Navigation::getVerticesOfRobots(sros::core::Pose curPose, verticesOfRobots &fourVerticesWithTwoMiddlePos,
                                     int lineNum) { //get the points spacing 1cm
    PosNeedToCheck posNeedToCheck;             //FourVertices  and two points :the middle point in the long side
    double halfHypotenuse = sqrt(
            pow(((double) HALF_DIAGONAL_LINE), 2) + pow(((double) HALF_WIDTH_LINE), 2)); // 斜边的长度的一半
    double ARC_DIGAGOLINE_WITH_LENGTH = atan((double) car_width_ / car_length_); // 长边与斜边的夹角
//    LOG(ERROR) << "car_length_: "<< car_length_ << " car_width_: " << car_width_ << " halfHypotenuse: " << halfHypotenuse;

    //LOG(INFO)<<"------------------ARC_DIGAGOLINE_WITH_LENGTH ---------------" <<ARC_DIGAGOLINE_WITH_LENGTH;
    //printf("enter getVerticesOfRobots curPose.x() %f curPose.y() %f\n",curPose.x(),curPose.y());
    posNeedToCheck.geometryPoint = new Geometry_Point[2 * (car_length_ + car_width_)];
    // agv的右上点
    posNeedToCheck.geometryPoint[0].x =
            (halfHypotenuse) * cos(curPose.yaw() + ARC_DIGAGOLINE_WITH_LENGTH) / METER_TO_CM + curPose.x();
    posNeedToCheck.geometryPoint[0].y =
            (halfHypotenuse) * sin(curPose.yaw() + ARC_DIGAGOLINE_WITH_LENGTH) / METER_TO_CM + curPose.y();
//    if(fabs(posNeedToCheck.geometryPoint[0].x)>1000||fabs(posNeedToCheck.geometryPoint[0].y)>1000){
//
//        printf("posNeedToCheck.geometryPoint[0].y %f\n",posNeedToCheck.geometryPoint[0].y);
//        return ;
//    }
    //LOG(INFO)<<"halfHypotenuse "<< halfHypotenuse <<" ARC_DIGAGOLINE_WITH_LENGTH "<< ARC_DIGAGOLINE_WITH_LENGTH << " curPose.yaw() " << curPose.yaw();
    //LOG(INFO)<<"posNeedToCheck.geometryPoint[0].x "<<posNeedToCheck.geometryPoint[0].x <<"  posNeedToCheck.geometryPoint[0].y "<<posNeedToCheck.geometryPoint[0].y;
    // agv的左上点
    posNeedToCheck.geometryPoint[2 * car_length_ + car_width_].x =
            (halfHypotenuse) * cos(curPose.yaw() - ARC_DIGAGOLINE_WITH_LENGTH) / METER_TO_CM + curPose.x();
    posNeedToCheck.geometryPoint[2 * car_length_ + car_width_].y =
            (halfHypotenuse) * sin(curPose.yaw() - ARC_DIGAGOLINE_WITH_LENGTH) / METER_TO_CM + curPose.y();
    //LOG(INFO)<< "   posNeedToCheck.geometryPoint[2*car_length_+car_width_].x " <<posNeedToCheck.geometryPoint[2*car_length_+car_width_].x
    //         << "   posNeedToCheck.geometryPoint[2*car_length_+car_width_].y " <<posNeedToCheck.geometryPoint[2*car_length_+car_width_].y;

    // agv的左下
    posNeedToCheck.geometryPoint[car_width_ + car_length_].x = 2 * curPose.x() - posNeedToCheck.geometryPoint[0].x;
    posNeedToCheck.geometryPoint[car_width_ + car_length_].y = 2 * curPose.y() - posNeedToCheck.geometryPoint[0].y;

    // agv的右下点
    posNeedToCheck.geometryPoint[car_length_].x =
            2 * curPose.x() - posNeedToCheck.geometryPoint[2 * car_length_ + car_width_].x;
    posNeedToCheck.geometryPoint[car_length_].y =
            2 * curPose.y() - posNeedToCheck.geometryPoint[2 * car_length_ + car_width_].y;

    auto m_to_mm_fun = [](const Geometry_Point &m_point) { // 米转毫米
        Geometry_Point mm_point;
        mm_point.x = m_point.x * METER_TO_MM;
        mm_point.y = m_point.y * METER_TO_MM;
//        LOG(INFO) << "x: " << mm_point.x << "y: " << mm_point.y;
        return mm_point;
    };
    Polygon car_pose; // 单位毫米
    car_pose.push_back(m_to_mm_fun(posNeedToCheck.geometryPoint[0]));
    car_pose.push_back(m_to_mm_fun(posNeedToCheck.geometryPoint[car_length_]));
    car_pose.push_back(m_to_mm_fun(posNeedToCheck.geometryPoint[car_width_ + car_length_]));
    car_pose.push_back(m_to_mm_fun(posNeedToCheck.geometryPoint[2 * car_length_ + car_width_]));
    car_simulate_move_poses_.push_back(std::move(car_pose));

    for (int i = 1; i < car_length_; i++) {
        posNeedToCheck.geometryPoint[i].x =
                (posNeedToCheck.geometryPoint[car_length_].x - posNeedToCheck.geometryPoint[0].x) / car_length_ * i +
                posNeedToCheck.geometryPoint[0].x;
        posNeedToCheck.geometryPoint[2 * car_length_ + car_width_ - i].x =
                (posNeedToCheck.geometryPoint[car_length_].x - posNeedToCheck.geometryPoint[0].x) / car_length_ * i +
                posNeedToCheck.geometryPoint[2 * car_length_ + car_width_].x;

        posNeedToCheck.geometryPoint[i].y =
                (posNeedToCheck.geometryPoint[car_length_].y - posNeedToCheck.geometryPoint[0].y) / car_length_ * i +
                posNeedToCheck.geometryPoint[0].y;
        posNeedToCheck.geometryPoint[2 * car_length_ + car_width_ - i].y =
                (posNeedToCheck.geometryPoint[car_length_].y - posNeedToCheck.geometryPoint[0].y) / car_length_ * i +
                posNeedToCheck.geometryPoint[2 * car_length_ + car_width_].y;
    }

    for (int j = 1; j < car_width_; j++) {
        posNeedToCheck.geometryPoint[car_length_ + j].x = (posNeedToCheck.geometryPoint[car_length_ + car_width_].x -
                                                           posNeedToCheck.geometryPoint[car_length_].x) / car_width_ *
                                                          j + posNeedToCheck.geometryPoint[car_length_].x;
        posNeedToCheck.geometryPoint[2 * car_length_ + 2 * car_width_ - j].x =
                (posNeedToCheck.geometryPoint[car_length_ + car_width_].x -
                 posNeedToCheck.geometryPoint[car_length_].x) / car_width_ * j + posNeedToCheck.geometryPoint[0].x;

        posNeedToCheck.geometryPoint[car_length_ + j].y = (posNeedToCheck.geometryPoint[car_length_ + car_width_].y -
                                                           posNeedToCheck.geometryPoint[car_length_].y) / car_width_ *
                                                          j + posNeedToCheck.geometryPoint[car_length_].y;
        posNeedToCheck.geometryPoint[2 * car_length_ + 2 * car_width_ - j].y =
                (posNeedToCheck.geometryPoint[car_length_ + car_width_].y -
                 posNeedToCheck.geometryPoint[car_length_].y) / car_width_ * j + posNeedToCheck.geometryPoint[0].y;
    }
    posNeedToCheck.lineNum = lineNum;

    fourVerticesWithTwoMiddlePos.push_back(posNeedToCheck);
    //printf("exit getVerticesOfRobots\n");
}


//in checkPointsValid , still need to dilate the obtacle 5cm ,safety ditance
int Navigation::checkPointsValid(sros::core::Pose cur,
                                 sros::core::Location_Vector &radar_obstacle,
                                 sros::core::NavigationPath_vector &nav_paths,
                                 verticesOfRobots &fourVerticesWithTwoMiddlePos) {
    int enableDebugObstacle = sros::core::Settings::getInstance().getValue<int>("nav.enable_debug_obstacle", 0);

    int width, height, offset_x, offset_y, cx, cy, obs_x = -1, obs_y = -1;
    Obstacle_vector inflation_obs;
    //printf("enter checkPointsValid\n");
    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;
    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;

    convertMapCoords(offset_x, offset_y, cur.x(), cur.y(), &cx, &cy);
    //sros::core::NavigationPathi_vector paths = changeToMapPath(nav_paths);

    if (!getViewPointFromRadar(radar_obstacle, width, height, offset_x, offset_y, cx, cy)) {
        //printf("enter no obstacle\n");
        return 0;
    }
    inflation_obs.clear();
    //膨胀视野内的雷达点,加入到视野层中用于判断当前路径是否被障碍阻挡
    inflation_obs = getInflationObstacle(_radar_obstacle, (int) 1, width, height, cx, cy);
    addViewObstacleToGrid(&grid_map_, inflation_obs);
    verticesOfRobots::iterator it;
    PosNeedToCheck forecastPos;
    int forecastPosIndex = 0;
    int lengthLinePointsNum = 2 * (HALF_DIAGONAL_LINE);
    int widthLintPointsNum = 2 * (HALF_WIDTH_LINE);
    double halfHypotenuse = sqrt(pow((HALF_DIAGONAL_LINE), 2) + pow((HALF_WIDTH_LINE), 2));
    //LOG(INFO) << "fourVerticesWithTwoMiddlePos" << fourVerticesWithTwoMiddlePos.size();
    for (it = fourVerticesWithTwoMiddlePos.begin(); it != fourVerticesWithTwoMiddlePos.end(); it++) {
        forecastPos = *it;
        int obstacleX;
        int obstacleY;
        for (int i = 0; i < 2 * (lengthLinePointsNum + widthLintPointsNum); i++) {
            convertMapCoords(offset_x, offset_y, forecastPos.geometryPoint[i].x, forecastPos.geometryPoint[i].y,
                             &obstacleX, &obstacleY);
            if (!GETWALKABLE(grid_map_.nodes[obstacleY][obstacleX].grid_layered_walkable_, VIEW_WALKABLE_TRUE)) {


                double distance = sqrt(pow(((double) (cur.x() - forecastPos.geometryPoint[i].x)), 2) +
                                       pow(((double) (cur.y() - forecastPos.geometryPoint[i].y)), 2)) * METER_TO_CM;
                if (enableDebugObstacle == 1) {
                    LOG(INFO) << "!!!!!!!!!-----------------------!!!!!!!!!!!!!!!";
                    LOG(INFO) << "current pos x:" << cur.x() << " pos y :" << cur.y() << " obstacle point x :"
                              << forecastPos.geometryPoint[i].x
                              << " pos y " << forecastPos.geometryPoint[i].y << "i " << i;
                    LOG(INFO) << "distance " << distance;
                }
//                if((distance<(halfHypotenuse+OBSTACLE_FILTER_DISTANCE))&&(distance>(halfHypotenuse-OBSTACLE_FILTER_DISTANCE))&&(g_state.load_state == sros::core::LOAD_FULL))
                if ((distance < (halfHypotenuse + OBSTACLE_FILTER_DISTANCE)) &&
                    (distance > (halfHypotenuse - OBSTACLE_FILTER_DISTANCE))) {
                    //LOG(INFO)<<"No obstacle distance";
                    continue;
                }
                //if(distance*100 > halfHypotenuse + 2 )||(distance*100 < halfHypotenuse + 2 )

                delViewObstacleFromGrid(&grid_map_, inflation_obs);
                if (forecastPosIndex == 0) {
                    forecastPosIndex = 10;
                }
                return forecastPosIndex;
            }
        }
        forecastPosIndex++;
    }
    delViewObstacleFromGrid(&grid_map_, inflation_obs);
    //printf("exit checkPointsValid\n");
    return 0;

}

int Navigation::calTheObstacleDistance(sros::core::Pose cur,
                                       sros::core::Location_Vector &radar_obstacle,
                                       sros::core::NavigationPath_vector &nav_paths,
                                       int &block_path_number) {
    getThePoseAfterSimulateMove(cur, nav_paths, fourVerticesWithTwoMiddlePos);
    //LOG(WARNING) << "fourVerticesWithTwoMiddlePos" << fourVerticesWithTwoMiddlePos.size();
    int forecastPosIndex = checkPointsValid(cur, radar_obstacle, nav_paths, fourVerticesWithTwoMiddlePos);
    //LOG(INFO) << "-----------forecastPosIndex: " << forecastPosIndex;
    int distanceObstacle = forecastPosIndex * SIMULATE_WALK_STEP_LENGTH;
//    if(forecastPosIndex==0){
//        block_path_number= fourVerticesWithTwoMiddlePos.at(fourVerticesWithTwoMiddlePos.size()-1).lineNum;
//    }
//    else
//        block_path_number= fourVerticesWithTwoMiddlePos.at(forecastPosIndex).lineNum;
    block_path_number = 0;
    //LOG(INFO)<<"fourVerticesWithTwoMiddlePos.size() "<<fourVerticesWithTwoMiddlePos.size() <<" block_path_number "<<block_path_number;
    for (int i = 0; i < fourVerticesWithTwoMiddlePos.size(); i++) {
        delete[] fourVerticesWithTwoMiddlePos.at(i).geometryPoint;
    }
    fourVerticesWithTwoMiddlePos.clear();

    return distanceObstacle;
}

BLOCK_TYPE Navigation::checkObstaclePointsWithDistance(sros::core::Pose cur,
                                                       sros::core::Location_Vector &radar_obstacle,
                                                       sros::core::NavigationPath_vector &nav_paths,
                                                       int &block_path_number) {
    car_simulate_move_poses_.clear();

    if (SIMULATE_WALK_TOTAL_LENGTH < SLOW_DISTANCE * LENGTH_PER_GRID) {
        SIMULATE_WALK_TOTAL_LENGTH = SLOW_DISTANCE * LENGTH_PER_GRID;
    }
    int distanceObstacle = calTheObstacleDistance(cur, radar_obstacle, nav_paths, block_path_number);

    new_car_simulate_move_poses_callback_(std::move(car_simulate_move_poses_));

    distanceObstacle = (distanceObstacle + 1) / LENGTH_PER_GRID;
    //LOG(INFO) << "distanceObstacle: " << distanceObstacle << " STOP_DISTANCE " << STOP_DISTANCE << " SLOW_DISTANCE " << SLOW_DISTANCE;
    if (distanceObstacle <= STOP_DISTANCE &&
        distanceObstacle > 1) {                                  //distanceObstacle 0 means no obstacle
        return SEGMENT_STOP;
    } else if (distanceObstacle > STOP_DISTANCE && distanceObstacle < SLOW_DISTANCE) {
        return SEGMENT_SLOW;
    } else {
        return SEGMENT_FREE;
    }

}


BLOCK_TYPE Navigation::checkObstaclePointsWithS300(int obstacle_area, int &block_path_number) {
    block_path_number = 0;
    uint8_t backLaderState = obstacle_area & 0x01;
    obstacle_area = obstacle_area >> 2;
    static int obstacleCounntNum = 0;
    if (backLaderState == 0) {
        obstacleCounntNum++;
    } else {
        obstacleCounntNum = 0;
    }
    //LOG(INFO)<<"******************obstacle_area ****************" << obstacle_area;
    //if(obstacle_area == INTERNAL_AREA || obstacle_area == MIDDLE_AREA )
    //if(obstacle_area == INTERNAL_AREA || obstacle_area == MIDDLE_AREA || obstacleCounntNum > 1)
    if (obstacle_area == MIDDLE_AREA) {
        auto src_state = src_sdk->getSRCState();
        LOG(INFO) << "src_state.gpio_input  " << hex << (int) src_state.gpio_input;
        LOG(INFO) << "******************checkObstaclePointsWithS300**************************";
        LOG(INFO) << "obstacle_area " << hex << (int) obstacle_area << " backLaderState " << hex
                  << (int) backLaderState;
        return SEGMENT_STOP;
    } else if (obstacle_area == EXTERNAL_AREA) {
        //LOG(INFO)<<"******************checkObstaclePointsWithS300****************222222222222222222";
        return SEGMENT_SLOW;
    } else if (obstacle_area == NO_OBSTACLE) {
        //LOG(INFO)<<"******************checkObstaclePointsWithS300****************333333333333333333";
        return SEGMENT_FREE;
    }
    //add default return values
    return SEGMENT_FREE;
}

BLOCK_TYPE Navigation::checkObstaclePoints(sros::core::Pose cur,
                                           sros::core::Location_Vector &radar_obstacle,
                                           sros::core::NavigationPath_vector &nav_paths,
                                           int &block_path_number) {
    int width, height, offset_x, offset_y, cx, cy, obs_x = -1, obs_y = -1;
    Obstacle_vector inflation_obs;
    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;
    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;
    convertMapCoords(offset_x, offset_y, cur.x(), cur.y(), &cx, &cy);
    sros::core::NavigationPathi_vector paths = changeToMapPath(nav_paths);

    if (!getViewPointFromRadar(radar_obstacle, width, height, offset_x, offset_y, cx, cy)) {
        return SEGMENT_FREE;
    }

    inflation_obs.clear();
    //膨胀视野内的雷达点,加入到视野层中用于判断当前路径是否被障碍阻挡
    inflation_obs = getInflationObstacle(_radar_obstacle, INFLATION_FACTOR, width, height, cx, cy);
    addViewObstacleToGrid(&grid_map_, inflation_obs);

    bool collision = false;
    double distance = 0.0, radius;
    sros::core::NavigationPathi_vector::iterator it;
    sros::core::NavigationPath<int> p;
    block_path_number = 0; //记录被阻碍的路径编号
    for (it = paths.begin(); it != paths.end(); ++it) {
        p = *it;
        if (p.type_ != sros::core::PATH_ROTATE) {
            if (((it != paths.begin()) && pointDistance(cx, cy, p.sx_, p.sy_) > ERODE_RADIUS) ||
                distance - ERODE_RADIUS > EPSINON) {
                delViewObstacleFromGrid(&grid_map_, inflation_obs);
                return SEGMENT_FREE;
            }

            if (p.type_ == sros::core::PATH_LINE) {
                if (!bresenhamLine(&grid_map_, p.sx_, p.sy_, p.ex_, p.ey_, &obs_x, &obs_y, width, height, false,
                                   true)) {
//                    std::cout << "BLOCK_LINE: " << p.sx_ << " ," << p.sy_ << " ==> " << p.ex_ << " ," << p.ey_ << std::endl;
                    collision = true;
                }

                //直线路径被阻挡
                if (collision) {
                    distance += pointDistance(p.sx_, p.sy_, obs_x, obs_y);
                } else {
                    distance += pointDistance(p.sx_, p.sy_, p.ex_, p.ey_);
                }

            } else if (p.type_ == sros::core::PATH_ARC) {
                Geometry_Point start, end, arc_center;
                start.x = p.sx_, start.y = p.sy_;
                end.x = p.ex_, end.y = p.ey_;
                arc_center.x = p.cx_, arc_center.y = p.cy_;
                double angle = 0.0;
                if (!arcWalkable(&grid_map_, arc_center, p.radius_, start, end, &obs_x, &obs_y, width, height, false)) {
//                    std::cout << "BLOCK_ARC: " << p.sx_ << " ," << p.sy_ << " ==> " << p.ex_ << " ," << p.ey_ << std::endl;
                    collision = true;
                }

                //圆弧路径被阻挡
                if (collision) {
                    angle = calAngle(p.sx_ - arc_center.x, p.sy_ - arc_center.y,
                                     obs_x - arc_center.x, obs_y - arc_center.y);
                } else {
                    angle = calAngle(p.sx_ - arc_center.x, p.sy_ - arc_center.y,
                                     p.ex_ - arc_center.x, p.ey_ - arc_center.y);
                }
                if ((p.radius_ < EPSINON && angle < EPSINON) || (p.radius_ > EPSINON && angle > EPSINON))
                    angle = 360.0 - fabs(angle);
                distance += fabs(angle) / 360.0 * fabs(p.radius_) * 2 * M_PI;

            } else if (p.type_ == sros::core::PATH_BEZIER) {
                Geometry_Point start, end, pa, pb;
                start.x = p.sx_, start.y = p.sy_;
                pa.x = p.cx_, pa.y = p.cy_;
                pb.x = p.dx_, pb.y = p.dy_;
                end.x = p.ex_, end.y = p.ey_;
                if (!bezierWalkable(&grid_map_, start, pa, pb, end, obs_x, obs_y, radius, true, width, height)) {
//                    std::cout << "BLOCK_BSPLINE: " << p.sx_ << " ," << p.sy_ << " ==> " << p.cx_ << " ," << p.cy_ <<
//                            " ==> " << p.dx_ << " ," << p.dy_ << " ==> " << p.ex_ << " ," << p.ey_ << std::endl;
                    collision = true;
                }
                //贝塞尔路径被阻挡
                if (collision) {
                    if (it == paths.begin())
                        distance += pointDistance(cx, cy, obs_x, obs_y);
                    else
                        distance += pointDistance(p.sx_, p.sy_, obs_x, obs_y);
                } else {
                    if (it == paths.begin()) {
                        distance += pointDistance(cx, cy, p.ex_, p.ey_);
                    } else {
                        distance += pointDistance(p.sx_, p.sy_, p.ex_, p.ey_);
                    }
                }
            }

            if (collision) {
                delViewObstacleFromGrid(&grid_map_, inflation_obs);
                std::cout << "OBS_BLOCK: " << obs_x << " " << obs_y << ", START_POINT" << cx << " " << cy <<
                          ", Distance: " << distance << " BLOCK_PATH_NUMBER: " << block_path_number << std::endl;
//                std::cout << "BLOCK_GRID Static: " << grid_map_.nodes[obs_y][obs_x].static_walkable <<
//                " Dynaimc: " << grid_map_.nodes[obs_y][obs_x].dynamic_walkable << std::endl <<
//                " Static Buffer: " << grid_map_.nodes[obs_y][obs_x].static_buffer <<
//                " Dynamic Buffer: " << grid_map_.nodes[obs_y][obs_x].dynamic_buffer << std::endl;

                if (distance <= STOP_DISTANCE) {
                    std::cout << "SEGMENT STOP:" << "distance=" << distance << " < " << STOP_DISTANCE << std::endl;
                    return SEGMENT_STOP;
                } else if (distance > STOP_DISTANCE && distance <= SLOW_DISTANCE) {
                    std::cout << "SEGMENT SLOW:" << "distance=" << distance << " > " << STOP_DISTANCE << std::endl;
                    return SEGMENT_SLOW;
                } else {
                    return SEGMENT_FREE;
                }
            }
        }

        ++block_path_number;
    }

    delViewObstacleFromGrid(&grid_map_, inflation_obs);
    return SEGMENT_FREE;
}

sros::core::NavigationPath_vector Navigation::getNavigationPaths(sros::core::Pose src,
                                                                 sros::core::Pose dst,
                                                                 NAV_PATH_TYPE type,
                                                                 sros::core::Location_Vector &radar_obstacle,
                                                                 bool isAllowBackward) {
    failed_code_ = ERROR_CODE_NONE;
    int startx, starty, endx, endy, offset_x, offset_y, width, height;
    sros::core::NavigationPath_vector nav_path;

    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;
    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;

    convertMapCoords(offset_x, offset_y, src.x(), src.y(), &startx, &starty);
    convertMapCoords(offset_x, offset_y, dst.x(), dst.y(), &endx, &endy);

    DYNAMIC_BUFFER_LENGTH = (type == BEZIER_TYPE) ? BUFFER_LENGTH : (2 * BUFFER_LENGTH);

//    std::cout << "地图分辨率为: " << LENGTH_PER_GRID << "cm, 车体宽度: " << car_width_
//    << "cm, 车体长度: " << car_length_ << "cm, 停止距离: " << STOP_DISTANCE
//    << "cm, 减速距离: " << SLOW_DISTANCE << "cm, 记忆时间: " << OBSTACLE_MEMORIZE_TIME
//    << "s, 忽略角度: " << IGNORE_ANGLE << ", 忽略直线: " << IGNORE_LINE
//    << "cm, 最长距离: " << LONGEST_DIVIDED_LENGTH << "cm, 膨胀类型: " << INFLATION_TYPE << std::endl;
//
//    std::cout << "缓冲半径: " << DYNAMIC_BUFFER_LENGTH << " 视野半径： " << ERODE_RADIUS
//    << " 起点周围半径: " << MENDING_RADIUS << " 腐蚀系数: " << INFLATION_FACTOR
//    << " 平滑圆弧半径: " << ARC_RADIUS << " 最短路径长度: " << SHORTEST_PATH << std::endl;

    int64_t clock_begin, clock_end;
    clock_begin = std::chrono::steady_clock::now().time_since_epoch().count();
        nav_path = navigate(startx, starty, endx, endy, dst.yaw(), type, false, isAllowBackward);

        std::cout << "初次规划路径！" << std::endl;
//        display(&grid_map_, width, height);


    clock_end = std::chrono::steady_clock::now().time_since_epoch().count();
    LOG(INFO) << "导航路径搜索时间: " << (clock_end - clock_begin) / 1000000.0 << "毫秒" ;

    return nav_path;
}


sros::core::NavigationPath_vector Navigation::replanPath(sros::core::Pose src,
                                                                 sros::core::Pose dst,
                                                                 NAV_PATH_TYPE type,
                                                                 sros::core::Location_Vector &radar_obstacle,
                                                                 bool isAllowBackward,bool localplan_or_not) {
    failed_code_ = ERROR_CODE_NONE;
    int startx, starty, endx, endy, offset_x, offset_y, width, height;
    sros::core::NavigationPath_vector nav_path;

    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;
    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;

    convertMapCoords(offset_x, offset_y, src.x(), src.y(), &startx, &starty);
    convertMapCoords(offset_x, offset_y, dst.x(), dst.y(), &endx, &endy);

    DYNAMIC_BUFFER_LENGTH = (type == BEZIER_TYPE) ? BUFFER_LENGTH : (2 * BUFFER_LENGTH);

//    std::cout << "地图分辨率为: " << LENGTH_PER_GRID << "cm, 车体宽度: " << car_width_
//    << "cm, 车体长度: " << car_length_ << "cm, 停止距离: " << STOP_DISTANCE
//    << "cm, 减速距离: " << SLOW_DISTANCE << "cm, 记忆时间: " << OBSTACLE_MEMORIZE_TIME
//    << "s, 忽略角度: " << IGNORE_ANGLE << ", 忽略直线: " << IGNORE_LINE
//    << "cm, 最长距离: " << LONGEST_DIVIDED_LENGTH << "cm, 膨胀类型: " << INFLATION_TYPE << std::endl;
//
//    std::cout << "缓冲半径: " << DYNAMIC_BUFFER_LENGTH << " 视野半径： " << ERODE_RADIUS
//    << " 起点周围半径: " << MENDING_RADIUS << " 腐蚀系数: " << INFLATION_FACTOR
//    << " 平滑圆弧半径: " << ARC_RADIUS << " 最短路径长度: " << SHORTEST_PATH << std::endl;
    
    float tmp_width = sros::core::Settings::getInstance().getValue<double>("nav.vehicle_width", 0.6);
    int inflation_factor = std::max(cfg_->map_expansion_size,(float)(tmp_width+0.20))*100.0f/cfg_->nav_resolution/2.0f;

    int64_t clock_begin, clock_end;
    clock_begin = std::chrono::high_resolution_clock::now().time_since_epoch().count();

    convertToViewPointFromRadar(src, radar_obstacle);
    LOG(INFO) << "重新规划路径！" ;

    Obstacle_vector obstacle, inflation_obstacle, buffer_obstacle;

    //从雷达点中筛选出障碍
    obstacle = chooseObstacleFromRadar(navigation_map_->getMapData(), _radar_obstacle, width, height);

    //障碍点膨胀
    inflation_obstacle = getInflationObstacle(obstacle, inflation_factor, width, height, startx, starty);

    //障碍点加入到grid地图中区
    addObstacleToGrid(&grid_map_, inflation_obstacle);

    //障碍点缓冲区膨胀
//    buffer_obstacle = getInflationObstacle(inflation_obstacle, DYNAMIC_BUFFER_LENGTH,
//                                           width, height, startx, starty);
//
//    //加入障碍缓冲
//    addObstacleBufferToGrid(&grid_map_, buffer_obstacle);

//        display(&grid_map_, width, height);
    LOG(INFO) << "地图更新完毕！开始搜索路径" ;
    nav_path = navigate(startx, starty, endx, endy, dst.yaw(), type, false,false,localplan_or_not);

     LOG(INFO) << "路径搜索结束，删除临时障碍点" ;
    delObstacleFromGrid(&grid_map_, inflation_obstacle);

    clock_end = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    LOG(INFO) << "导航路径搜索时间: " << (clock_end - clock_begin) / 1000000.0 << "毫秒" ;

    return nav_path;
}

void Navigation::replanPath_del_obs(){
        //恢复地图，从地图中去除障碍信息
}



bool Navigation::convertToViewPointFromRadar(sros::core::Pose &curr_pose, sros::core::Location_Vector &oba_points) {
    int width, height, offset_x, offset_y, cx, cy;
    offset_x = navigation_map_->getMapZeroOffset().x;
    offset_y = navigation_map_->getMapZeroOffset().y;
    width = navigation_map_->getMapSize().x;
    height = navigation_map_->getMapSize().y;
    convertMapCoords(offset_x, offset_y, curr_pose.x(), curr_pose.y(), &cx, &cy);

    return getViewPointFromRadar(oba_points,width,height,offset_x,offset_y,cx,cy);

}
