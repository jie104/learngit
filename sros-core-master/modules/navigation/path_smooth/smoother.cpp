//
// Created by yj on 19-11-6.
//

#include "smoother.h"

////////////////////test by 吴运才
//#include "modules/navigation/reference_paths/reference_paths.h"
Navigation g_navigate;
ReferencePaths g_gradient_of_ref_path;    //加载参考路径势场地图
////////////////////


Smoother::~Smoother(){
    // for (int i = 0; i < width; i++) {
    //     delete[] gridMap[i];
    // }
    // delete[] gridMap;
}

// 初始化用于smooth的地图。 其中0 表示无障碍，1表示有障碍。
void Smoother::initGridMap(int width1,int height1  ) {
    // if(gridMap!=nullptr){
    //     for (int i = 0; i < width; i++) {
    //     delete[] gridMap[i];
    //     }
    //     delete[] gridMap;
    // }
    // // std::cout << "localmap.width: " << localmap.width << "localmap.height" << localmap.height << std::endl;
    // width = nav_map->getMapSizeX();
    // height =  nav_map->getMapSizeY();
    // uint8_t obstacle = 0x24;
    // sros::map::NavMapData_ptr map_date = nav_map->getMapData();
    // gridMap = (unsigned char **) malloc(width * sizeof(unsigned char *));
    // int i,j;
    // for (i = 0; i < width; i++) {
    //     gridMap[i] = (unsigned char *) malloc(height * sizeof(unsigned char));
    //     malloc_count++; /* [ Malloc Count ] */
    //     for (j = 0; j < height; ++j) {
    //         gridMap[i][j] = (map_date[j * width + i]!=0)?254:0;
    //     }
    // }

    width = width1;
    height =  height1;
    // smoother_map_ = map;
    // LOG(INFO)<<"111:"<<(float)(smoother_map_->collision_map[1][1].value);
    // LOG(INFO)<<"222:"<<(float)(map->collision_map[1][1].value);
}

void Smoother::initNavConfig(NavConfig& cfg){
    cfg_ = &cfg;
}


// void Smoother::addObstacleFromGridMap(Navigation& navigate_,std::vector<Eigen::Vector2d>& oba_points) {
//     int offset_x = navigate_.getNavigationMap()->getMapZeroOffsetX();
//     int offset_y = navigate_.getNavigationMap()->getMapZeroOffsetY();
//     g_navigate = navigate_;

//     for(auto point:oba_points){
//         int map_x;
//         int map_y;
//         navigate_.Navigation::convertMapCoords( offset_x,  offset_y, point(0), point(1), &map_x, &map_y);
//         //LOG(INFO)<<"计算的障碍点的坐标："<<map_x<<":"<<map_y;
//         if(gridMap[map_x][map_y] <200){
//             gridMap[map_x][map_y] = gridMap[map_x][map_y] +(unsigned char)1;
//         }
//     }

// }

// void Smoother::deleteObstacleFromGridMap(Navigation& navigate_,std::vector<Eigen::Vector2d> oba_points) {

//     int offset_x = navigate_.getNavigationMap()->getMapZeroOffsetX();
//     int offset_y = navigate_.getNavigationMap()->getMapZeroOffsetY();
//     g_navigate = navigate_;

//     for(auto point:oba_points){
//         int map_x;
//         int map_y;
//         navigate_.Navigation::convertMapCoords( offset_x,  offset_y, point(0), point(1), &map_x, &map_y);
//         //LOG(INFO)<<"计算的障碍点的坐标："<<map_x<<":"<<map_y;
//         if(gridMap[map_x][map_y] <200){
//             gridMap[map_x][map_y] = gridMap[map_x][map_y] -(unsigned char)1;
//         }
//     }
// }

void Smoother::pathSample(sros::core::NavigationPath_vector final_path,Navigation& navigate)
{
    g_navigate = navigate;
    auto size = final_path.size();
    float step =0.3; // 每隔0.3m取一个点。
    std::vector<Vector2D>  trajectory_point;
    trajectory_point.clear();
    path.clear();
    float last_dist = 0;
    for(int i = 0;i<size;i++)
    {
        if(final_path[i].type_ == 1)
        {
            Vector2D start(final_path[i].sx_,final_path[i].sy_);
            Vector2D end(final_path[i].ex_,final_path[i].ey_);
            Vector2D start_to_end(final_path[i].ex_-final_path[i].sx_,final_path[i].ey_-final_path[i].sy_);
            float dist = std::sqrt(std::pow(start.getX()-end.getX(),2)+std::pow(start.getY()-end.getY(),2));
            if(trajectory_point.empty()){
                trajectory_point.push_back(start);
            }
            // 将上一段路径留下的剩余距离包含进去
            float new_start_x;
            float new_start_y;
            if(dist>= (step-last_dist)){
                new_start_x = start.getX()+start_to_end.getX()*(step-last_dist)/dist;
                new_start_y = start.getY()+start_to_end.getY()*(step-last_dist)/dist;
                Vector2D point(new_start_x,new_start_y) ;
                trajectory_point.push_back(point);
            }
            else {
                last_dist = last_dist + dist;
                continue;
            }

            Vector2D new_start_to_end(final_path[i].ex_-new_start_x,final_path[i].ey_-new_start_y);

            float new_dist = std::sqrt(std::pow(new_start_to_end.getX(),2)+std::pow(new_start_to_end.getY(),2));

            int num = (int)((dist-(step-last_dist))/step);

            for (int j=1;j<=num;j++){

                float x = new_start_x+new_start_to_end.getX()*step/new_dist*j;
                float y = new_start_y+new_start_to_end.getY()*step/new_dist*j;
                Vector2D point1(x,y) ;
                trajectory_point.push_back(point1);

            }
            last_dist = (dist-(step-last_dist)) - ((float)num) *step;
        }
        if(i== (size-1)){
            if((last_dist<step/2.0f)&&trajectory_point.size()>=2){
                trajectory_point.erase((trajectory_point.end()-1));
            }
            if(final_path[i].type_ == 4)
            {
                Vector2D end(final_path[i].sx_,final_path[i].sy_);
                trajectory_point.push_back(end);
            }
            else
            {
                Vector2D end(final_path[i].ex_,final_path[i].ey_);
                trajectory_point.push_back(end);
            }
        }
    }


    int offset_x = navigate.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = navigate.getNavigationMap()->getMapZeroOffsetY();
    for(auto point:trajectory_point) {
        Node3D traj_point ;
        int x,y;
        navigate.convertMapCoords(offset_x,offset_y,point.getX(),point.getY(),&x,&y);

        traj_point.setX(x);
        traj_point.setY(y);
        path.push_back(traj_point);
    }
    LOG(INFO)<<"经过采样后的轨迹点个数："<<trajectory_point.size();
}

// 采用0.5米采样并且优化之后， 重新采用0.1米采样并再次优化。
void Smoother::pathResample(sros::core::NavigationPath_vector final_path,Navigation& navigate_)
{
    auto size = final_path.size();
    float step =0.25; // 每隔0.3m取一个点。
    std::vector<Vector2D>  trajectory_point;
    trajectory_point.clear();
    path.clear();
    for(int i = 0;i<size;i++)
    {
        if(final_path[i].type_ == 1)
        {
            Vector2D start(final_path[i].sx_,final_path[i].sy_);
            Vector2D end(final_path[i].ex_,final_path[i].ey_);
            Vector2D start_to_end(final_path[i].ex_-final_path[i].sx_,final_path[i].ey_-final_path[i].sy_);
            float dist = std::sqrt(std::pow(start.getX()-end.getX(),2)+std::pow(start.getY()-end.getY(),2));
            int num = (int)(dist/step);
            for (int i=0;i<num;i++){

                float x = start.getX()+start_to_end.getX()*step/dist*i;
                float y = start.getY()+start_to_end.getY()*step/dist*i;
                Vector2D point(x,y) ;
                trajectory_point.push_back(point);
            }
        }
        if(i== (size-1)){
            if(final_path[i].type_ == 4)
            {
                Vector2D end(final_path[i].sx_,final_path[i].sy_);
                trajectory_point.push_back(end);
            }
            else
            {
                Vector2D end(final_path[i].ex_,final_path[i].ey_);
                trajectory_point.push_back(end);
            }
        }
    }
    int offset_x = navigate_.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = navigate_.getNavigationMap()->getMapZeroOffsetY();
    for(auto point:trajectory_point) {
        Node3D traj_point ;
        int x,y;
        navigate_.convertMapCoords(offset_x,offset_y,point.getX(),point.getY(),&x,&y);

        traj_point.setX(x);
        traj_point.setY(y);
        path.push_back(traj_point);
    }
    //std::cout<<"经过采样后的轨迹点个数："<<trajectory_point.size()<<std::endl;
}

// 将最终得到的path转化为全局坐标点，并且采用直线形式发送的matrix界面上。
void Smoother::convertPath(Navigation& navigate_,sros::core::NavigationPath_vector& cur_paths_,std::vector<sros::core::Pose>& opt_paths){
    int offset_x = navigate_.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = navigate_.getNavigationMap()->getMapZeroOffsetY();
    g_navigate = navigate_;
    cur_paths_.clear();
    opt_paths.clear();
    std::vector<Node3D>::iterator it;
    sros::core::Pose traj_point;
    for(it=path.begin();it<path.end()-1;it++)
    {
        Node3D point = *it;
        Node3D point_next = *(it+1);
        float x = point.getX();
        float y = point.getY();
        double point_x,point_y;
        navigate_.reverseMapCoords(offset_x,offset_y,x,y,&point_x,&point_y);
        traj_point.x() = point_x;
        traj_point.y() = point_y;

        opt_paths.push_back(traj_point);
        double point_next_x,point_next_y;
         x = point_next.getX();
         y = point_next.getY();
        navigate_.reverseMapCoords(offset_x,offset_y,x,y,&point_next_x,&point_next_y);

        traj_point.x() = point_next_x;
        traj_point.y() = point_next_y;

        sros::core::LinePath pose(point_x,point_y,point_next_x,point_next_y);
        cur_paths_.push_back(pose);
    }

    opt_paths.push_back(traj_point);
}

inline bool isCusp(std::vector<Node3D> path, int i) {
    bool revim2 = path[i - 2].getPrim() > 2 ? true : false;
    bool revim1 = path[i - 1].getPrim() > 2 ? true : false;
    bool revi   = path[i].getPrim() > 2 ? true : false;
    bool revip1 = path[i + 1].getPrim() > 2 ? true : false;
      bool revip2 = path[i + 2].getPrim() > 2 ? true : false;

    if (revim1 != revi || revi != revip1||revip1!=revip2) { return true; }

    return false;
}
inline bool isCusp2(std::vector<Node3D> path, int i) {


    bool revi   = path[i].getPrim() > 2 ? true : false;


    if (revi) { return true; }

    return false;
}

// 得到路径点在栅格地图上的坐标位置。
void Smoother::setPath(std::vector<Node3D> path) {
    this->path.clear();
    this->path = path;
}


void Smoother::smoothPath(CheckCollision& global_map) {
    int offset_x = g_navigate.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = g_navigate.getNavigationMap()->getMapZeroOffsetY();


    // current number of iterations of the gradient descent smoother
    int iterations = 0;
    // the maximum iterations for the gd smoother
    int maxIterations =200;
    // the lenght of the path in number of nodes
    int pathLength = 0;

    // path objects with all nodes oldPath the original, newPath the resulting smoothed path
    pathLength = path.size();
    std::vector<Node3D> newPath = path;
    if(pathLength<5){
        return;
    }

    // descent along the gradient untill the maximum number of iterations has been reached
    float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;

    while (iterations < maxIterations) {

        // choose the first three nodes of the path
        for (int i = 2; i < pathLength - 2; ++i) {

            Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
            Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
            Vector2D xi(newPath[i].getX(), newPath[i].getY());
            Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
            Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
            Vector2D correction;


            // the following points shall not be smoothed
            // keep these points fixed if they are a cusp point or adjacent to one

            if (isCusp(newPath, i)) {
                // LOG(INFO)<<"```````````````优化点被固定";
                continue; }

            correction = correction - obstacleTerm(xi,global_map);
            if (!isOnGrid(xi + correction)) { continue; }

            //todo not implemented yet
            // voronoiTerm();

            // ensure that it is on the grid
            vector<Pose> paths;
            for(int j = 0; j < newPath.size(); j++) {
                double x, y;
                g_navigate.reverseMapCoords(offset_x, offset_y, newPath[j].getX(), newPath[j].getY(), &x, &y);
                paths.push_back(Pose(x, y, 0));
            }
            double x0, y0, x, y, x1, y1;
            g_navigate.reverseMapCoords(offset_x, offset_y, xim1.getX(), xim1.getY(), &x0, &y0);
            g_navigate.reverseMapCoords(offset_x, offset_y, xi.getX(), xi.getY(), &x, &y);
            g_navigate.reverseMapCoords(offset_x, offset_y, xip1.getX(), xip1.getY(), &x1, &y1);
           // Vector2D ref_path_gradient = g_gradient_of_ref_path.getGrimMapGradient(x0, y0, x, y, x1, y1);

            //correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2) + ref_path_gradient;
            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);

            if (!isOnGrid(xi + correction)) { continue; }

            // ensure that it is on the grid
//            correction = correction - curvatureTerm(xim1, xi, xip1);
//            if (!isOnGrid(xi + correction)) { continue; }

            // ensure that it is on the grid

            xi = xi + alpha * correction/totalWeight;
            newPath[i].setX(xi.getX());
            newPath[i].setY(xi.getY());
            Vector2D Dxi = xi - xim1;
            newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));

        }

//        float new_x,new_y;
//        if (!isCusp2(newPath, 1)) {
//        new_x = (newPath[0].getX()+newPath[2].getX())/2.0f;
//        new_y = (newPath[0].getY()+newPath[2].getY())/2.0f;
//        newPath[1].setX(new_x);
//        newPath[1].setY(new_y);
//        }
//        if (!isCusp2(newPath, pathLength - 2)) {
//            new_x = (newPath[pathLength - 1].getX() + newPath[pathLength - 3].getX()) / 2.0f;
//            new_y = (newPath[pathLength - 1].getY() + newPath[pathLength - 3].getY()) / 2.0f;
//            newPath[pathLength - 2].setX(new_x);
//            newPath[pathLength - 2].setY(new_y);
//        }


        iterations++;
    }

    path = newPath;
}


void Smoother::smoothPath1(CheckCollision& global_map) {
    //return;
    int offset_x = g_navigate.getNavigationMap()->getMapZeroOffsetX();
    int offset_y = g_navigate.getNavigationMap()->getMapZeroOffsetY();

    // current number of iterations of the gradient descent smoother
    int iterations = 0;
    // the maximum iterations for the gd smoother
    int maxIterations =150;
    // the lenght of the path in number of nodes
    int pathLength = 0;

    // path objects with all nodes oldPath the original, newPath the resulting smoothed path
    pathLength = path.size();


    std::vector<Node3D> newPath = path;
    if(pathLength<4){
        return;
    }
    // descent along the gradient untill the maximum number of iterations has been reached
    float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;

    double t0 = sros::core::util::get_time_in_us();
    double print_run_t = 0;

    while (iterations < maxIterations) {
        iterations++;
        //LOG(INFO)<<"iterations"<<iterations<<";pathLength:"<<pathLength;
        if(pathLength<4){
            break;
        }

        {
            Vector2D xim1(newPath[0].getX(), newPath[0].getY());
            Vector2D xi(newPath[1].getX(), newPath[1].getY());
            Vector2D xip1(newPath[2].getX(), newPath[2].getY());
            Vector2D xip2(newPath[3].getX(), newPath[3].getY());
            Vector2D correction;

            correction = correction - obstacleTerm(xi,global_map);
            // LOG(INFO)<<"11";
            if (!isOnGrid(xi + correction)) { continue; }

            // ensure that it is on the grid
            correction = correction - smoothnessTerm1(xim1, xi, xip1, xip2);
            
            if (!isOnGrid(xi + correction)) { continue; }
            // ensure that it is on the grid

            xi = xi + alpha * correction/totalWeight;
            newPath[1].setX(xi.getX());
            newPath[1].setY(xi.getY());
            Vector2D Dxi = xi - xim1;
            newPath[0].setT(std::atan2(Dxi.getY(), Dxi.getX()));
        }

        // choose the first three nodes of the path
        for (int i = 2; i < pathLength - 2; ++i) {
            if(pathLength<5){
                break;
            }

            Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
            Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
            Vector2D xi(newPath[i].getX(), newPath[i].getY());
            Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
            Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
            Vector2D correction;

            // the following points shall not be smoothed
            // keep these points fixed if they are a cusp point or adjacent to one

            if (isCusp(newPath, i)) {
                // LOG(INFO)<<"```````````````优化点被固定";
                continue; }

            correction = correction - obstacleTerm(xi,global_map);
            if (!isOnGrid(xi + correction)) { continue; }

            //todo not implemented yet
            // voronoiTerm();

            // ensure that it is on the grid
            
            t0 = sros::core::util::get_time_in_us();
            vector<Pose> paths;
            for(int j = 0; j < newPath.size(); j++) {
                double x, y;
                g_navigate.reverseMapCoords(offset_x, offset_y, newPath[j].getX(), newPath[j].getY(), &x, &y);
                paths.push_back(Pose(x, y, 0));
            }
            double x0, y0, x, y, x1, y1;
            g_navigate.reverseMapCoords(offset_x, offset_y, xim1.getX(), xim1.getY(), &x0, &y0);
            g_navigate.reverseMapCoords(offset_x, offset_y, xi.getX(), xi.getY(), &x, &y);
            g_navigate.reverseMapCoords(offset_x, offset_y, xip1.getX(), xip1.getY(), &x1, &y1);
           // Vector2D ref_path_gradient = g_gradient_of_ref_path.getGrimMapGradient(x0, y0, x, y, x1, y1);
            print_run_t += sros::core::util::get_time_in_us() - t0;

            //correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2) + ref_path_gradient;
             correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2) ;

            if (!isOnGrid(xi + correction)) { continue; }

            // ensure that it is on the grid
//            correction = correction - curvatureTerm(xim1, xi, xip1);
//            if (!isOnGrid(xi + correction)) { continue; }

            // ensure that it is on the grid

            xi = xi + alpha * correction/totalWeight;
            newPath[i].setX(xi.getX());
            newPath[i].setY(xi.getY());
            Vector2D Dxi = xi - xim1;
            newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));
        }

        Vector2D xim1(newPath[pathLength - 4].getX(), newPath[pathLength - 4].getY());
        Vector2D xi(newPath[pathLength - 3].getX(), newPath[pathLength - 3].getY());
        Vector2D xip1(newPath[pathLength - 2].getX(), newPath[pathLength - 2].getY());
        Vector2D xip2(newPath[pathLength - 1].getX(), newPath[pathLength - 1].getY());
        Vector2D correction;

        correction = correction - obstacleTerm(xip1,global_map);
        if (!isOnGrid(xip1 + correction)) { continue; }

        // ensure that it is on the grid
        correction = correction - smoothnessTerm2(xim1, xi, xip1, xip2);
        if (!isOnGrid(xip1 + correction)) { continue; }
        // ensure that it is on the grid

        xip1 = xip1 + alpha * correction/totalWeight;
        newPath[pathLength - 2].setX(xip1.getX());
        newPath[pathLength - 2].setY(xip1.getY());
        Vector2D Dxi = xip1 - xi;
        newPath[pathLength - 3].setT(std::atan2(Dxi.getY(), Dxi.getX()));
    }

    // //基于人工设置的参考线对路径进行优化，当地图太大时，会占用较大内存
    // vector<Pose> opt_path;
    // for(int i = 0; i < newPath.size(); i++) {
    //     double x, y;
    //     g_navigate.reverseMapCoords(offset_x, offset_y, newPath[i].getX(), newPath[i].getY(), &x, &y);
    //     opt_path.push_back(Pose(x, y, 0));
    // }
    // g_gradient_of_ref_path.secondOptPath(0.02, opt_path);
    // newPath.clear();
    // for(int i = 0; i < opt_path.size(); i++) {
    //     double x, y;
    //     g_navigate.convertMapCoords(offset_x, offset_y, opt_path[i].x(), opt_path[i].y(), &x, &y);
    //     HybridAStar::Node3D new_node;
    //     new_node.setX(x);
    //     new_node.setY(y);
    //     newPath.push_back(new_node);
    // }
    // LOG(INFO) << "增加路径优化运行时间: " << print_run_t << "us";
    // //基于人工设置的参考线对路径进行优化，当地图太大时，会占用较大内存

    path = newPath;
}



Vector2D Smoother::obstacleTerm(Vector2D xi, CheckCollision& global_map) {
    Vector2D gradient;
    // the distance to the closest obstacle from the current node

    // the vector determining where the obstacle is
    int x = (int)xi.getX();
    int y = (int)xi.getY();

    // 每次取得x的坐标之后 都需要选取周围上下左右各0.5m的障碍点。并找出最近点
    // 根据点x计算出最近的障碍物位置
    float obsDst = cfg_->smoother_param.minRoadWidth+1.0 ;
    int length  = 40;
    int up_left =x-length;
    int up_right =x+length;
    int down_left = y-length;
    int down_right = y+length;
    float obstX,obstY;
    // LOG(INFO)<<"1223:"<<x<<";"<<y<<";"<<width<<";"<<height;
     
    for(int m = up_left;m<=up_right;m++) {
        for(int n = down_left;n<=down_right;n++){
            if (m < width && m >= 0 && n < height && n >= 0) {
                if (global_map.collision_map[m][n].value >0) {
                    //LOG(INFO)<<"12234:"<<m<<";"<<n<<";"<<(float)smoother_map_->collision_map[m][n].value ;
                    float dist = std::sqrt(pow((x - m),2)+ pow((y - n),2));
                    if (dist < obsDst) {
                        obsDst = dist;
                        obstX = m;
                        obstY = n;
                    }
                }
            }
        }
    }

    // if the node is within the map
    if (x < width && x >= 0 && y < height && y >= 0) {
        // the closest obstacle is closer than desired correct the path for that
        if (obsDst < cfg_->smoother_param.minRoadWidth) {
            Vector2D obsVct(xi.getX() - obstX, xi.getY() - obstY);
            gradient = wObstacle * 2 * (obsDst - cfg_->smoother_param.minRoadWidth) * obsVct / obsDst;
            //LOG(INFO)<<"OBSTACL:GRADIENT:"<<gradient.getX()<<":"<<gradient.getY();
            return gradient;
        }
    }
    return gradient;
}


Vector2D Smoother::curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1) {
    Vector2D gradient;
    // the vectors between the nodes
    Vector2D Dxi = xi - xim1;
    Vector2D Dxip1 = xip1 - xi;
    // orthogonal complements vector
    Vector2D p1, p2;

    // the distance of the vectors
    float absDxi = Dxi.length();
    float absDxip1 = Dxip1.length();

    // ensure that the absolute values are not null
    if (absDxi > 0 && absDxip1 > 0) {
        // the angular change at the node
        float Dphi = std::acos(Helper::clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), -1, 1));
        float kappa = Dphi / absDxi;
//        if(Dxi.dot(Dxip1) / (absDxi * absDxip1)<0)
//        std::cout<<"kappa: "<<kappa<<std::endl;

        // if the curvature is smaller then the maximum do nothing
        if (kappa <= kappaMax) {
            Vector2D zeros;
            return zeros;
        } else {
            float absDxi1Inv = 1 / absDxi;
            float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
            float u = absDxi1Inv * PDphi_PcosDphi;
            // calculate the p1 and p2 terms
            p1 = xi.ort(-xip1) / (absDxi * absDxip1);
            p2 = -xip1.ort(xi) / (absDxi * absDxip1);
            // calculate the last terms
            float s = Dphi / (absDxi * absDxi*absDxi);
            Vector2D ones(1, 1);
            Vector2D ki = u * (-p1 - p2) - (s * Dxi);
            Vector2D kim1 = u * p2 + (s * Dxi);
            Vector2D kip1 = u * p1;

            // calculate the gradient
            gradient = wCurvature * (0.3 * kim1 + 0.4 * ki + 0.3 * kip1);

            if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
                std::cout << "nan values in curvature term" << std::endl;
                Vector2D zeros;
                return zeros;
            }
                // return gradient of 0
            else {
                // std::cout<<"curvature gradient:"<<gradient.getX()<<" ; :"<<gradient.getY()<<std::endl;
                return gradient;
            }
        }
    }
        // return gradient of 0
    else {
        std::cout << "abs values not larger than 0" << std::endl;
        Vector2D zeros;
        return zeros;
    }
}


Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
    Vector2D gradient = wSmoothness * (xim2 - 4.0 * xim1 + 6.0 * xi - 4.0 * xip1 + xip2);
    return  gradient;
}


Vector2D Smoother::smoothnessTerm1(Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
    Vector2D gradient = wSmoothness * ( -2.0 * xim1 + 5.0 * xi - 4.0 * xip1 + xip2);
    return  gradient;
}

Vector2D Smoother::smoothnessTerm2(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1) {
    Vector2D gradient = wSmoothness * ( xim2 - 4.0 * xim1 + 5.0 * xi - 2.0 * xip1);
    return  gradient;
}
