/*
 * @Author: your name
 * @Date: 2020-08-24 10:00:53
 * @LastEditTime: 2021-06-21 15:38:10
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sros/modules/navigation/lib/src/jps_grid.cpp
 */
#include <malloc.h>

#include "../include/jps_grid.h"
#include "../include/geometry.h"

struct node createNode(unsigned char walkable) {
    struct node nd;
    nd.grid_layered_walkable_ = (walkable == 0) ? INIT_WALKABLE_TRUE : INIT_WALKABLE_FALSE;
    return nd;
}

struct grid createGrid(int width, int height, sros::map::PyramidNavMap_ptr matrix) {
    struct grid gd;
    gd.width = width;
    gd.height = height;
    std::cout << "BuildNode Start" << std::endl;
    gd.nodes = _buildNodes(width, height, matrix);
    std::cout << "BuildNode Complete" << std::endl;
    return gd;
}

struct node **_buildNodes(int width, int height, sros::map::PyramidNavMap_ptr matrix) {
    int i, j;
    struct node **nodes;
    nodes = (struct node **) malloc(height * sizeof(struct node *));
    malloc_count++; /* [ Malloc Count ] */

    for (i = 0; i < height; i++) {
        nodes[i] = (struct node *) malloc(width * sizeof(struct node));
        malloc_count++; /* [ Malloc Count ] */
        for (j = 0; j < width; ++j) {
            nodes[i][j] = createNode(matrix->isOccInGridIndex(i * width + j));
        }
    }
    return nodes;
}

struct node *getNodeAt(struct grid *gd, int x, int y) {
    return &gd->nodes[y][x];
}

bool isWalkableAt(struct grid *gd, int x, int y) {
    return isInside(gd, x, y) && GETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, WALKABLE_TRUE);
}

bool isInside(struct grid *gd, int x, int y) {
    return (x >= 0 && x < gd->width) && (y >= 0 && y < gd->height);
}

void setWalkableAt(struct grid *gd, int x, int y, bool walkable) {
    if (walkable)
        SETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, WALKABLE_TRUE, true);
    else
        SETWALKABLE(gd->nodes[y][x].grid_layered_walkable_, WALKABLE_FALSE, false);
}

// /*
// * 实际地图坐标转化为导航坐标
// */
// void convertMapCoords(int offset_x, int offset_y, double x, double y, int *cx, int *cy) {
//     *cx = dci(offset_x + x * 100 / 2);
//     *cy = dci(offset_y - y * 100 / 2);
//     return;
// }

// /*
//  * 导航坐标转换为实际地图坐标
//  */
// void reverseMapCoords(int offset_x, int offset_y, int cx, int cy, double *x, double *y) {
//     *x = (cx - offset_x) * 100 / 2;
//     *y = (offset_y - cy) * 100 / 2;
//     return;
// }