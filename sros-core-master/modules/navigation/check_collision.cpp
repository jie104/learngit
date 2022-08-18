//
// Created by yj on 20-4-17.
//

#include "check_collision.h"


// 析构函数，需要释放地图资源。
CheckCollision::~CheckCollision() {
    for (int i = 0; i < map_width; i++) {
        delete[] collision_map[i];
    }
    delete[] collision_map;
}

// 需要初始化地图
void CheckCollision::init_check_collision(sros::map::NavigationMap_ptr nav_map,NavConfig* cfg) {

    cfg_ = cfg;
    init_or_not = 0;
    last_tmp_obstalces.clear();
    if(collision_map!=nullptr){
        LOG(INFO)<<"删除collision："<<map_width;
        for (int i = 0; i < map_width; i++) {
        delete[] collision_map[i];
        }
        delete[] collision_map;
    }

    // std::cout << "localmap.width: " << localmap.width << "localmap.height" << localmap.height << std::endl;
    map_width = nav_map->getMapSizeX();
    map_height = nav_map->getMapSizeY();

    sros::map::PyramidNavMap_ptr map_date = nav_map->getMapData();



    collision_map = (MAP_POINT **) malloc(map_width * sizeof(MAP_POINT *));
    int count1=0;
    int count2 = 0;
    int i, j;
    for (i = 0; i < map_width; i++) {
        collision_map[i] = (MAP_POINT *) malloc(map_height * sizeof(MAP_POINT));
        // malloc_count++; /* [ Malloc Count ] */
        for (j = 0; j < map_height; ++j) {
            collision_map[i][j].value = (map_date->isFreeInGridIndex(j * map_width + i)) ? 0 : 254;
            if(collision_map[i][j].value==254){
                collision_map[i][j].type = 1;
                count1++;
            }else{
                collision_map[i][j].type = 0;
                count2++;
            }
        }
    }
    LOG(INFO)<<"被占用的栅格点数："<<count1<<";未被占用的点数："<<count2;
    LOG(INFO)<<"完成避障地图的初始化！";
   // LOG(INFO)<<"7"<<(float)collision_map[1][1].type;
    init_or_not = 1;
}

bool CheckCollision::find_obstacle_type(uint8_t id,std::string& device_name){
    auto int_iter = int_to_str_map_.find(id);
    if (int_iter == int_to_str_map_.end()) {//如果map内不存在当前id，则说明还未创建，直接返回错误
      LOG(INFO) << "cannot find the device name by id:" << id<< ";will return false" ;
      return false;
    }
    device_name = int_iter->second;
    return true;
}



uint8_t CheckCollision::find_obstacle_no(std::string& device_name){
    auto str_iter = str_to_int_map_.find(device_name);
    if (str_iter == str_to_int_map_.end()) {//如果map内不存在当前device，就新建一个id赋值给当前device，然后返回
      str_to_int_map_[device_name] = curr_incre_;
      int_to_str_map_[curr_incre_] = device_name;
      curr_incre_++;
      return curr_incre_ - 1;
    }
    return str_iter->second;
}


// 根据每次进来的实时障碍点来更新地图。更新地图前需要删除掉之前的临时障碍点。
void CheckCollision::update_collision_map(Navigation &nav, std::map<std::string,std::vector<Eigen::Vector2d>> &cur_obstalces) {

    if (!init_or_not) {
        return;
    }

    int offset_x = nav.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = nav.getNavigationMap()->getMapZeroOffsetY();
    //LOG(INFO)<<"222:"<<last_tmp_obstalces.size()<<";"<<map_width<<";"<<map_height;
    // 每次进来一次障碍物，则将上次的障碍物清理掉。
    if (!last_tmp_obstalces.empty()) {
        for (auto point:last_tmp_obstalces) {
            int map_x;
            int map_y;
            map_x = std::round(point.x());
            map_y = std::round(point.y());
            // nav.convertMapCoords(offset_x, offset_y, point.x(), point.y(), &map_x, &map_y);
            //LOG(INFO)<<"计算的障碍点的坐标："<<map_x<<":"<<map_y;
            if(map_x>=0&&map_x<map_width&&map_y>=0&&map_y<map_height){
                if (collision_map[map_x][map_y].value < 201&&collision_map[map_x][map_y].value >0) {
                    collision_map[map_x][map_y].value = collision_map[map_x][map_y].value -  (uint8_t)1;
                    collision_map[map_x][map_y].type = 0;
                }
                else{
                    LOG(INFO)<<"删除临时障碍点，该点值超过200为："<< (int16_t)collision_map[map_x][map_y].value;
                }
            }
        }
    }

    last_tmp_obstalces.clear();

    //LOG(INFO)<<"1111111"<<map_width<<";"<<map_height<<";"<<cur_obstalces.size();
    //添加最近的障碍物到地图中。
    for (auto p:cur_obstalces) {
        auto tmp_p = p.second;
        auto string = p.first;
        //LOG(INFO)<<"tmp_p:"<<tmp_p.size();
        for(auto point:tmp_p)
        {
            int map_x;
            int map_y;
            nav.convertMapCoords(offset_x, offset_y, point[0], point[1], &map_x, &map_y);
            // 取地图信息时都需要判断 是否超过边界。
            if(map_x>=0&&map_x<map_width&&map_y>=0&&map_y<map_height){
                //LOG(INFO)<<"计算的障碍点的坐标："<<map_x<<":"<<map_y;
                if (collision_map[map_x][map_y].value < 200) {
                     //LOG(INFO)<<"11111111";
                    collision_map[map_x][map_y].value = collision_map[map_x][map_y].value +  (uint8_t)1;
                    //LOG(INFO)<<"22222222";
                    collision_map[map_x][map_y].type = find_obstacle_no(string);
                    Eigen::Vector2d map_obs_point(map_x,map_y);
                    last_tmp_obstalces.push_back(map_obs_point);
                }

            }
            else {
               // LOG(INFO)<<"ERROR:2实时障碍点超过地图边界！！！";
            }
        }
    }

    //LOG(INFO)<<"3333!";
}

// 判断矩形区域内是否有障碍物。有 返回true 没有返回 false。
bool CheckCollision::check_point_collision(Navigation &nav, sros::core::Pose cur_pose, float robot_length,
                                           float robot_width,COLLISION_OBS& obs_pose) {

    if (!init_or_not) {
        return true;
    }

    int check_length;
    check_length = std::sqrt(robot_length * robot_length + robot_width * robot_width) / cfg_->nav_resolution*100.0 + 10;
    int offset_x = nav.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = nav.getNavigationMap()->getMapZeroOffsetY();
    int map_x, map_y;
    nav.convertMapCoords(offset_x, offset_y, cur_pose.x(), cur_pose.y(), &map_x, &map_y);
    sros::core::Pose cur_map_pose;
    cur_map_pose.x() = map_x;
    cur_map_pose.y() = map_y;
    cur_map_pose.yaw() = cur_pose.yaw();

    // 根据局部矩形的边长 计算四个角度的坐标
    sros::core::Pose up_left, up_right, down_left, down_right;

    up_left.x() = cur_map_pose.x() - check_length / 2;
    up_left.y() = cur_map_pose.y() - check_length / 2;

    if (up_left.x() < 0) { up_left.x() = 0.0; }
    if (up_left.y() < 0) { up_left.y() = 0.0; }

    up_right.y() = up_left.y();
    up_right.x() = cur_map_pose.x() + check_length / 2;

    if (up_right.x() >= map_width) { up_right.x() = map_width - 1; }

    down_left.x() = up_left.x();
    down_left.y() = cur_map_pose.y() + check_length / 2;

    if (down_left.y() >= map_height) { down_left.y() = map_height - 1; }

    down_right.x() = up_right.x();
    down_right.y() = down_right.y();


    int margin = -1.0;
    // 然后将局部矩形内的障碍点保存在 local_obstacles.
    for (int i = (int) (up_left.x()); i <= up_right.x(); i++) {
        for (int j = (int) (up_left.y()); j <= down_left.y(); j++) {
            if (collision_map[i][j].value > 0) {

                // 将坐标点i,j 转换为机器人坐标系下。
                float dx = i - cur_map_pose.x();
                float dy = j - cur_map_pose.y();

                float yaw = -1.0f * cur_map_pose.yaw();

                // 进行坐标转换。 将障碍点转换为agv坐标系下的坐标
                double new_x = dx * cos(yaw) + dy * sin(yaw);
                double new_y = -dx * sin(yaw) + dy * cos(yaw);

                if ((fabs(new_x) < (robot_length / cfg_->nav_resolution*100.0/2.0 + margin)) && (fabs(new_y) < (robot_width /cfg_->nav_resolution*100.0/2.0 + margin))) {
                    //LOG(INFO)<<"11agv_pose.x(): "<<cur_pose.x()<<" agv_pose.y():"<<cur_pose.y()<<"agv_pose.yaw():"<<cur_pose.yaw();
                    double obs_x, obs_y;
                    int16_t grid_value = collision_map[i][j].value;
                  //  uint8_t type = collision_map[i][j].type;
                    nav.reverseMapCoords(offset_x, offset_y, i, j, &obs_x, &obs_y);
//                   obs_pose.x() = obs_x;
//                   obs_pose.y() = obs_y;
                    find_obstacle_type(collision_map[i][j].type,obs_pose.obs_name);
                    LOG(INFO) << "避障系统检测到障碍物，此时的障碍点坐标为：" << obs_x << ":" << obs_y<<";此时检测点为x："<<cur_pose.x()<<";y"<<cur_pose.y()<<";yaw:"<<cur_pose.yaw();
                    LOG(INFO) << "检测到障碍物的值："<< grid_value<<";障碍物的类型为：" << obs_pose.obs_name ;
                    LOG(INFO) << "new_x：" << new_x << ";new_y:" << new_y<<"此时AGV的的长和宽 为："<< robot_length<<";"<<robot_width<<";lenlimit:"<<(robot_length / cfg_->nav_resolution*100.0/2.0 + margin)<<";widlimit:"<< (robot_width /cfg_->nav_resolution*100.0/2.0 + margin);
                    obs_pose.obs_pose.x() = obs_x;
                    obs_pose.obs_pose.y() = obs_y;
                    return true;
                }
            }
        }
    }
    return false;
}
