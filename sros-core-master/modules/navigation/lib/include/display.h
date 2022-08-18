#ifndef __included_display_h
#define __included_display_h

#include "neighbors.h"
#include "heap.h"
#include "core/navigation_path.h"

/* Draws the grid / map */
void displayGrid(struct grid *gd);

void displayPath(sros::core::NavigationPathi_vector paths);

void displayNPath(sros::core::NavigationPath_vector paths);

/* Displays rudimentary information about a given node */
void displayNodeInfo(struct grid *gd, int x, int y);

/* Draws the grid / map with the computed path */
void displaySolution(struct grid *gd, struct neighbor_xy_list *path);

/* Lists information about all walkable nodes adjecent to a node */
void listNeighbors(struct grid *gd, struct neighbor_xy_list *list);

/* Lists all entry in the open list ( contains eligible nodes for inspection ) */
void listOpenList(struct grid *gd, struct open_list *list);

/* 直线段显示,直线的转角只有45度 */
struct neighbor_xy_list *showWithPGM_Line(struct grid *gd, struct neighbor_xy_list *head,
                                          int width, int height);

/* 直线段处理之后图像显示，转角不固定，含直线+圆弧+原地旋转 */
void showWithPGM_Angle_Arc(struct grid *gd, sros::core::NavigationPathi_vector paths, int width, int height);

void display(struct grid *gd, int width, int height);

#endif
