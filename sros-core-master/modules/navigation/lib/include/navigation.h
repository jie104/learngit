#ifndef __included_navigation_h
#define __included_navigation_h

#include <vector>
#include "core/msg/common_poses_info_msg.hpp"
#include "core/navigation_path.h"
#include "core/pose.h"
#include "geometry.h"
#include "jps_grid.h"
#include "map_erode.h"
#include "core/msg/common_poses_info_msg.hpp"
#include "../../hybrid_astar/hybrid_a_star.h"
#include "../../nav_config.h"
#include "core/map/NavigationMap.hpp"

extern int LENGTH_PER_GRID; //每个网格的实际宽度,单位cm

extern int car_width_; //车体宽
extern int car_length_; //车体长
extern int STOP_DISTANCE; //停止距离
extern int SLOW_DISTANCE; //减速距离

extern double OBSTACLE_MEMORIZE_TIME; //记忆障碍障碍时间(s)

extern double IGNORE_ANGLE; //忽略夹角小于IGNORE_ANGLE的圆弧
extern double IGNORE_LINE; //忽略短于该距离的直线

extern StructuringElementType INFLATION_TYPE;// StructuringElementSquare表矩形内核，
// StructuringElementCircle表圆形状内核，
// StructuringElementDiamond表菱形内核，
// StructuringElementLine表直线形内核

extern int BUFFER_LENGTH; //腐蚀膨胀区半径
extern int DYNAMIC_BUFFER_LENGTH; //障碍缓冲半径;
extern double ROUND_OFF_ANGLE; //小于该值需要进行圆弧化操作
extern double LONGEST_DIVIDED_LENGTH; //最大相距距离,路径长度大于此数值时该路径会重新划分为短路径


#define HALF_DIAGONAL_LINE  (car_length_/2) // FIXME: HALF_LENGTH_LINE
#define HALF_WIDTH_LINE (car_width_/2 + INFLATION_FOR_SAFETY)
//腐蚀系数为车体长度的一半
#define  INFLATION_FACTOR std::max(car_width_, car_length_) / (2 * LENGTH_PER_GRID)
//检测障碍的视野范围
#define ERODE_RADIUS 2 * (SLOW_DISTANCE + INFLATION_FACTOR)

//为防止起点在缓冲区内起点周围的障碍清除方形半径
#define MENDING_RADIUS (int)(0.71 * DYNAMIC_BUFFER_LENGTH + 1)
/*
 * SHORTEST_PATH表示跳点生成直线时距离内核不可走区的最近距离为DYNAMEC_BUFFER_LENGTH*(a/b)时的弦长，亦即最短路径长度
 * 其公式为2 * sqrt( (INFLATION+DYNAMIC_BUFFER_LENGTH)^2 - (INFLATION_FACTOR+a/b*DYNAMIC_BUFFER_LENGTH)^2 )
 *      = 2 * sqrt( (2-2*a/b)*INFLATION_FACTOR*DYNAMIC_BUFFER_LENGTH - (a^2)/(b^2) * DYNAMIC_BUFFER_LENGTH^2 )
 * 其中a/b < 1
 */
#define SHORTEST_PATH (int)(2 * sqrt(1.0 / 2 * INFLATION_FACTOR * DYNAMIC_BUFFER_LENGTH + 9.0 / 16 * DYNAMIC_BUFFER_LENGTH * DYNAMIC_BUFFER_LENGTH))
//生成圆弧半径
#define ARC_RADIUS (int) (SHORTEST_PATH * 2)


typedef struct pointWithLineNum {
    Geometry_Point *geometryPoint;
    int lineNum;
} PosNeedToCheck;

typedef std::vector<PosNeedToCheck> verticesOfRobots;

enum BLOCK_TYPE {
    SEGMENT_FREE = 0x01,
    SEGMENT_SLOW = 0x02,
    SEGMENT_STOP = 0x03,
};
enum MOVEMENT_TRPE{
    GO_STRAIGHT_FORWARD_WITH_LOAD,
    GO_STRAIGHT_FORWARD_WITHOUT_LOAD,
    GO_STRAIGHT_BACKWARD_WITH_LOAD,
    GO_STRAIGHT_BACKWARD_WITHOUT_LOAD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_OFF
};
enum NAV_PATH_TYPE {
    LINE_ROTATE_TYPE = 0x01,
    LINE_ARC_TYPE = 0x02,
    BEZIER_TYPE = 0x03,
};
enum OBSTACLE_AREA_TYPE{
    MIDDLE_AREA=0x00,
    EXTERNAL_AREA=0x01,
    NO_OBSTACLE = 0x03
};
/*
 * @brief 导航类
 */
class Navigation {
public:
    Navigation();

    ~Navigation();

    sros::map::NavigationMap_ptr getNavigationMap() const {
        return navigation_map_;
    }

    /*
     * @brief 设定参数
     * @param name 参数名
     * @param value 参数值
     */
    bool setParam(std::string name, std::string value);

    /*
     * @brief 载入地图信息
     * @param file_path 文件路径
     * @param enable_load_map_data 是否加载灰度数据, 不加载可节省内存
     */
    bool loadMapFile(const std::string file_path, bool enable_load_map_data);

    void initNavConfig(NavConfig& cfg);

    /*
     * @brief 根据载入的地图进行数据初始化操作
     *
     * 操作包括构建grid数组, 腐蚀不可走区域等
     */
    void initMapData();

    /*
     * @brief 将载入的地图清除，以减小内存占用
     *
     * 释放grid数组里面的内容
     */
    void releaseMapData();

    /*
     * @brief 检测雷达障碍是否阻挡当前路径
     * @param cur 当前位姿
     * @param radar_obstacle 雷达障碍信息，为vector
     * @param nav_paths 上一次搜索的路径
     * @return SEGMENT_FREE 无路径阻碍
     *         SEGMENT_SLOW 与障碍距离小于SLOW_DISTANCE时进入减速段
     *         SEGMENT_STOP 与障碍距离小于STOP_DISTANCE时停车重新规划
     */
    BLOCK_TYPE checkObstaclePoints(sros::core::Pose cur,
                                   sros::core::Location_Vector &radar_obstacle,
                                   sros::core::NavigationPath_vector &nav_paths,
                                   int &block_path_number);
    BLOCK_TYPE checkObstaclePointsWithS300(int obstacle_area,int &block_path_number);
    /*
     * @brief 规划路径
     * @param src 起点位姿
     * @param dst 终点位姿
     * @param type 路径类型,有LINE_ARC_TYPE为直线圆弧, BEZIER_TYPE为贝塞尔曲线
     * @param radar_obstacle 雷达障碍点vector数组
     * @return 地图路径
     */
    sros::core::NavigationPath_vector getNavigationPaths(sros::core::Pose src,
                                                         sros::core::Pose dst,
                                                         NAV_PATH_TYPE type,
                                                         sros::core::Location_Vector &radar_obstacle,
                                                         bool isAllowBackward = false);

    sros::core::NavigationPath_vector replanPath(sros::core::Pose src,
                                                             sros::core::Pose dst,
                                                             NAV_PATH_TYPE type,
                                                             sros::core::Location_Vector &radar_obstacle,
                                                             bool isAllowBackward = false,
                                                 bool localplan_or_not = false);

    void replanPath_del_obs();


    std::vector<Node3D> replanHybridPath(sros::core::Pose src,
                                                     sros::core::Pose dst,
                                                     sros::core::Location_Vector &radar_obstacle);
    /*
    * @brief 检测雷达障碍是否阻挡当前路径
    * @param cur 当前位姿
    * @param radar_obstacle 雷达障碍信息，为vector
    * @param nav_paths 上一次搜索的路径
    * @return SEGMENT_FREE 无路径阻碍
    *         SEGMENT_SLOW 与障碍距离小于SLOW_DISTANCE时进入减速段
    *         SEGMENT_STOP 与障碍距离小于STOP_DISTANCE时停车重新规划
    */
    BLOCK_TYPE checkObstaclePointsWithDistance(sros::core::Pose cur,
                                               sros::core::Location_Vector &radar_obstacle,
                                               sros::core::NavigationPath_vector &nav_paths,
                                               int &block_path_number);
    double getTheNearest_T_inBezier(sros::core::Pose curPose,sros::core::NavigationPath<double> &navBezierPath,double robotNearestT);

    using newCarSimulateMovePosesCallback_t = std::function<void (Polygons &&car_poses)>;
    void setNewCarSimulateMovePosesCallback(newCarSimulateMovePosesCallback_t callback) { new_car_simulate_move_poses_callback_ = callback; }

    sros::core::ErrorCode getFiledCode() const { return failed_code_; }

    void convertMapCoords(int offset_x, int offset_y, double x, double y, int *cx, int *cy);

    void reverseMapCoords(int offset_x, int offset_y, double cx, double cy, double *x, double *y);

   // struct grid localgrid_map_;
   // Obstacle_vector  inflation_obstacle_;
private:
    /*
     * @brief 解析位姿偏移
     * @param
     */


    //获取直线路径,isFirstNavigate=false时缓冲区内取消短路径,长路径和长路径合并,短路径和短路径合并
    sros::core::NavigationPathi_vector getLinePaths(struct grid *gd,
                                                    struct neighbor_xy_list *path_head,
                                                    bool isFirstNavigate);

    //直线路径平滑，产生圆弧和转角
    sros::core::NavigationPathi_vector dealPaths(struct grid *gd,
                                                 sros::core::NavigationPathi_vector &paths_with_line);

    void deleteRotateFromPaths(sros::core::NavigationPathi_vector &paths);

    //局部膨胀障碍点地图
    Obstacle_vector getInflationObstacle(Obstacle_vector &src, int inflation_factor,
                                         int width, int height, int cx, int cy);

    //从雷达得到的集合中搜寻出距离范围内点集
    bool getViewPointFromRadar(sros::core::Location_Vector &radar_obstacle,
                               int width, int height,
                               int offset_x, int offset_y,
                               int cx, int cy);

    //从激光雷达点中筛选出障碍点信息
    Obstacle_vector chooseObstacleFromRadar(sros::map::PyramidNavMap_ptr map_data,
                                            Obstacle_vector &radar_point,
                                            int width, int height);

    //计算局部膨胀地图即偏移量
    void calOffset(sros::map::Point &up_left, sros::map::Point &down_right,
                   int cx, int cy, int width, int height, int radius);

    //重新规划路径与上一次路径结合
    //Paths combineLinePaths(struct grid *gd, Paths new_path, Paths last_path, int segment);

    //修改输出路径格式
    sros::core::NavigationPath_vector changeToNavigationPath(sros::core::NavigationPathi_vector &paths);

    sros::core::NavigationPathi_vector changeToMapPath(sros::core::NavigationPath_vector &paths);

    //大路径划分为小路径,用于路径优化
    sros::core::NavigationPathi_vector dividePathIntoPieces(sros::core::NavigationPathi_vector &path);

    //更新贝塞尔控制点
    bool updateBezier(Geometry_Point *p0, Geometry_Point *p1,
                      Geometry_Point *p2, Geometry_Point *p3,
                      long i, long startOrEnd, long restLength, long n,
                      sros::core::NavigationPathi_vector &line_path);

    //Bezier路径拟合
    sros::core::NavigationPathi_vector bezierFitting(sros::core::NavigationPathi_vector &line_path, bool isAllowBackward = false);
    double getTurnAngle(sros::core::Pose &d1, sros::core::Pose &d2) const;

    //修改Grid_Map
    void addBufferToGrid(int width, int height, sros::map::NavMapData_ptr matrix);

    //添加视野内障碍点到Grid->view_walkable
    void addViewObstacleToGrid(struct grid *gd, Obstacle_vector &obs);

    void delViewObstacleFromGrid(struct grid *gd, Obstacle_vector &obs);

    //添加膨胀后的障碍点到Grid
    void addObstacleToGrid(struct grid *gd, Obstacle_vector &obs);
    void addObstacleToGrid1(struct grid *gd, Obstacle_vector &obs);

    //添加障碍缓冲到grid
    void addObstacleBufferToGrid(struct grid *gd, Obstacle_vector &obs);

    //从grid中删除障碍和缓冲
    void delObstacleFromGrid(struct grid *gd, Obstacle_vector &obs);

    //起始点附件腐蚀
    void mendStartPoint(struct grid *gd, bool isErode,
                        int startx, int starty,
                        int w, int h);
    bool convertToViewPointFromRadar(sros::core::Pose& curr_pose,sros::core::Location_Vector& oba_points);
    /*
     * @brief 静态导航
     * @param 起始位置坐标(startx,starty),终点位置坐标(endx,endy);
     * @param type 路径类型，type=LINE_ARC_TYPE为直线和圆弧，type=BEZIER_TYPE为贝塞尔
     */
    sros::core::NavigationPath_vector navigate(int startx, int starty,
                                               int endx, int endy, double yaw,
                                               NAV_PATH_TYPE type, bool isFirstNavigate,bool isAllowBackward = false,
                                               bool localplan_or_not = false);

    void getThePoseAfterSimulateMove(sros::core::Pose curPose,sros::core::NavigationPath_vector &nav_paths,verticesOfRobots &fourVerticesWithTwoMiddlePos);
    sros::core::Pose getThePoseAfterSimulateMoveInLine(sros::core::Pose curPose, const sros::core::NavigationPath<double> &navLinePath);
    sros::core::Pose getThePoseAfterSimulateMoveInBezier(sros::core::Pose curPose,sros::core::NavigationPath<double> &navBezierPath);
    sros::core::Pose getThePoseAfterSimulateMoveInRotate(sros::core::Pose curPose,double rotate_angle);
    sros::core::Pose getThePoseAfterSimulateMoveInCircle(sros::core::Pose curPose,sros::core::NavigationPath<double > &navCirclePath);
    void getVerticesOfRobots(sros::core::Pose curPose,verticesOfRobots &fourVertices,int lineNum);
    int checkPointsValid(sros::core::Pose cur,sros::core::Location_Vector &radar_obstacle, sros::core::NavigationPath_vector &nav_paths,verticesOfRobots &fourVerticesWithTwoMiddlePos);
    int calTheObstacleDistance(sros::core::Pose cur, sros::core::Location_Vector &radar_obstacle, sros::core::NavigationPath_vector &nav_paths, int &block_path_number);
    sros::core::Pose calNextStepPos(sros::core::Pose curPose);

    //网格地图，用于判断障碍
    struct grid grid_map_;

    sros::map::PyramidNavMap_ptr inflation_map_data;
    

    NavConfig* cfg_;

    //存储膨胀的障碍点信息
    Obstacle_vector _radar_obstacle;

    //导航地图
    sros::map::NavigationMap_ptr navigation_map_;

    //上次清除雷达障碍时间戳
    int64_t _last_clear_time;  // (ns)

    Polygons car_simulate_move_poses_; // 小车模拟移动的位置
    newCarSimulateMovePosesCallback_t new_car_simulate_move_poses_callback_ = nullptr;

    int car_length_ = 1000; // 车的长度(cm) 此处初始值为默认值
    int car_width_ = 1000; // 车的宽度（cm) 此处初始值为默认值

    sros::core::ErrorCode failed_code_ = sros::core::ERROR_CODE_NONE;
    int width, height;
};

template<class T>
inline void normalizeAngle(T &angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle >= M_PI) {
        angle -= 2.0f * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
}

#endif
