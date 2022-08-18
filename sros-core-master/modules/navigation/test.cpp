#include <iostream>
#include "include/navigation.h"
#include "include/display.h"

using namespace std;

//查看内存申请情况，因为空间最终全部释放所以malloc_count最后输出必为0，方便测试使用
int malloc_count = 0;

int main() {
    //声明导航类
    Navigation *nav = new Navigation;
    //文件路径
    char file_path[MAX_FILE_LENGTH] = "/home/jc/workspace/test_map/large_map.map";
    //图片处理
    nav->loadMap(file_path);
    //进行路径计算, Path是vector结构。参数为图片+起点终点横纵坐标
    Paths paths = nav->navigate(0, 0, 4400, 8000);
    //路径输出
    displayPath(paths);

    Position pos = getPosition();
    Obstacle_vector obs;
    cv::Point p;
    p.x = pos.x - RADAR_RADUIS / 5;
    for (int i = dci(pos.y - RADAR_RADUIS / 2); i < dci(pos.y + RADAR_RADUIS / 1.5); i += 1) {
        p.y = i;
        obs.push_back(p);
    }

    p.y = pos.y - RADAR_RADUIS / 2;
    for (int i = dci(pos.x - RADAR_RADUIS / 2); i < dci(pos.x + RADAR_RADUIS / 1.5); i += 1) {
        p.x = i;
        obs.push_back(p);
    }

    Paths paths2;
    paths2 = nav->getDynamicPath(obs);
    displayPath(paths2);
    delete nav;
    printf("malloc_count = %d\n", malloc_count);

    return 0;
}
