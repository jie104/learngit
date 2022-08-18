#include <stdio.h>
#include <stdbool.h>
#include <fstream>
#include <vector>
#include "../include/display.h"
#include "../include/geometry.h"
#include "../include/navigation.h"

using namespace std;

void displaySolution(struct grid *gd, struct neighbor_xy_list *path) {
    int i, j;
    bool found = false;
    struct neighbor_xy_list *path_pos = path;
    for (i = 0; i < gd->height; i++) {
        for (j = 0; j < gd->width; ++j) {
            if (isWalkableAt(gd, j, i)) {
                while (path != (path_pos = path_pos->left)) {
                    if (path_pos->y == i && path_pos->x == j) {
                        printf("o");
                        found = true;
                    }
                }
                if (!found)
                    printf(".");
            } else
                printf("#");

            found = false;
        }
        printf("\n");
    }
}

void displayGrid(struct grid *gd) {
    int i, j;
    for (i = 0; i < gd->height; i++) {
        for (j = 0; j < gd->width; ++j) {
            if (isWalkableAt(gd, j, i))
                printf(".");
            else
                printf("#");
        }
        printf("\n");
    }
}

void displayPath(sros::core::NavigationPathi_vector paths) {
    int Seq = 0;
    sros::core::NavigationPathi_vector::iterator i;
    for (i = paths.begin(); i != paths.end(); ++i) {
        sros::core::NavigationPath<int> p = *(i);
        if (p.type_ == sros::core::PATH_LINE) {
            printf("Seq: %d line:(%d, %d) --> (%d, %d)   direction %d \n", ++Seq, p.sx_, p.sy_, p.ex_, p.ey_,p.direction_);
        } else if (p.type_ == sros::core::PATH_ROTATE) {
            printf("Seq: %d Rotate: %lf\n", ++Seq, p.rotate_angle_);
        } else if (p.type_ == sros::core::PATH_ARC) {
            printf("Seq: %d Arc:(%d, %d) --> (%d, %d), center:(%d %d), arc_radius: %lf  direction %d  \n", ++Seq, p.sx_, p.sy_,
                   p.ex_, p.ey_, p.cx_, p.cy_, p.radius_,p.direction_);
        } else if (p.type_ == sros::core::PATH_BEZIER) {
            printf("Seq: %d Control_Point:(%d, %d)(%d %d)(%d %d)(%d %d), radius: %lf    direction %d \n",
                   ++Seq, p.sx_, p.sy_, p.cx_, p.cy_, p.dx_, p.dy_, p.ex_, p.ey_, p.radius_,p.direction_);
        }
    }
    return;
}

void displayNPath(sros::core::NavigationPath_vector paths) {
    int i = 0;
    for (auto item : paths) {
        if (item.type_ == sros::core::PATH_LINE) {
            printf("Seq: %d line:(%lf, %lf) --> (%lf, %lf)\n", ++i, item.sx_, item.sy_, item.ex_, item.ey_);
        } else if (item.type_ == sros::core::PATH_ROTATE) {
            printf("Seq: %d Rotate: %lf\n", ++i, item.rotate_angle_);
        } else if (item.type_ == sros::core::PATH_ARC) {
            printf("Seq: %d Arc:(%lf, %lf) --> (%lf, %lf), center:(%lf, %lf), arc_radius:%lf\n", ++i,
                   item.sx_, item.sy_, item.ex_, item.ey_,
                   item.cx_, item.cy_, item.radius_);
        } else if (item.type_ == sros::core::PATH_BEZIER) {
            printf("Seq: %d Bezier:(%lf, %lf),(%lf, %lf),(%lf, %lf),(%lf, %lf), radius: %lf\n",
                   ++i, item.sx_, item.sy_, item.cx_, item.cy_, item.dx_, item.dy_, item.ex_,
                   item.ey_, item.radius_);
        }
    }
    return;
}

void displayNodeInfo(struct grid *gd, int x, int y) {
    printf("x: %i ", x);
    printf("\ny: %i ", y);
//    printf("\nf: %i ", gd->nodes[y][x].f);
    if (isWalkableAt(gd, x, y))
        printf("\nwalkable: yes\n\n");
    else
        printf("\nwalkable: no\n\n");
}

void listNeighbors(struct grid *gd, struct neighbor_xy_list *list) {
    struct neighbor_xy_list *head = list;
    struct neighbor_xy_list *current = list;
    while (head != (current = current->right)) {
        displayNodeInfo(gd, current->x, current->y);
    }
}

void listOpenList(struct grid *gd, struct open_list *list) {
    struct open_list *head = list;
    struct open_list *current = list;
    while (head != (current = current->right)) {
        displayNodeInfo(gd, current->x, current->y);
    }
}

struct neighbor_xy_list *showWithPGM_Line(struct grid *gd, struct neighbor_xy_list *head,
                                          int width, int height) {
    struct neighbor_xy_list *pos = head->left;
    int xi, yi, dx, dy;
    unsigned char black = 0, white = 255;
    while (head != NULL && (head != (pos = pos->left))) {
        xi = pos->x;
        yi = pos->y;
        dx = xi - pos->right->x;
        dy = yi - pos->right->y;
        if (abs(dx) > 1 || abs(dy) > 1) {
            int incrX = dx / std::max(abs(dx), 1);
            int incrY = dy / std::max(abs(dy), 1);
            pos = neighbor_xy_insert_right(pos, pos->right->x + incrX, pos->right->y + incrY);
        }
    }
    pos = head;
    while (head != NULL && (head != (pos = pos->left))) {
        //printf("Step %d: x:%d y:%d\n", count++, path_pos->x, path_pos->y);
        setWalkableAt(gd, pos->x, pos->y, false);
    }

    ofstream ou("/home/jc/workspace/navigation-line_rotate/copy.pgm");
    if (ou.fail()) {
        cerr << "open file:copy.pgm failed." << endl;
        return NULL;
    }
    ou << "P5" << endl << width << " " << height << endl << "255" << endl;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (isWalkableAt(gd, j, i))
                ou << white;
            else
                ou << black;
        }
    }
    ou.close();
    return head;
}

void showWithPGM_Angle_Arc(struct grid *gd, sros::core::NavigationPathi_vector paths, int width, int height) {
    int obs_x, obs_y;
    sros::core::NavigationPathi_vector::iterator i;
    for (i = paths.begin(); i != paths.end(); ++i) {
        sros::core::NavigationPath<int> p = *(i);
        if (p.type_ == sros::core::PATH_LINE) {
            bresenhamLine(gd, p.sx_, p.sy_, p.ex_, p.ey_, &obs_x, &obs_y, width, height, true, false);
            //printf("Seq: %d line:(%d, %d) --> (%d, %d)\n", p.ucSeries_, p.sx_, p.sy_, p.ex_, p.ey_);
        } else if (p.type_ == sros::core::PATH_ARC) {
            //draw_circle(gd, p.cy_, p.cx_, ARC_RADIUS, width, height, true);
            Geometry_Point center, start, end;
            center.x = p.cx_, center.y = p.cy_;
            start.x = p.sx_, start.y = p.sy_;
            end.x = p.ex_, end.y = p.ey_;
            arcWalkable(gd, center, p.radius_, start, end, &obs_x, &obs_y, width, height, true);
        } else if (p.type_ == sros::core::PATH_BEZIER) {
            Geometry_Point ps, pA, pB, ps2;
            ps.x = p.sx_, ps.y = p.sy_;
            pA.x = p.cx_, pA.y = p.cy_;
            pB.x = p.dx_, pB.y = p.dy_;
            ps2.x = p.ex_, ps2.y = p.ey_;
//            drawBezier(gd, ps, pA, pB, ps2, width, height);
        }
    }
    return;
}

void display(struct grid *gd, int width, int height) {
//    struct node *tgrid;
//    for (int i = 0; i < width; ++i) {
//        for (int j = 0; j < height; ++j) {
//            tgrid = getNodeAt(gd, i, j);
//            tgrid->walkable = tgrid->static_walkable & tgrid->dynamic_walkable &
//                    (!tgrid->static_buffer) & (!tgrid->dynamic_buffer);
//        }
//    }

    ofstream ou("/home/copy.pgm");
    if (ou.fail()) {
        cerr << "open file:copy.pgm failed." << endl;
        return;
    }
    unsigned char white = 255, black = 0;
    ou << "P5" << endl << width << " " << height << endl << "255" << endl;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (isWalkableAt(gd, j, i))
                ou << white;
            else
                ou << black;
        }
    }
    ou.close();
    return;
}
