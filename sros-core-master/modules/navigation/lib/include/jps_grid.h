#ifndef __included_jps_grid_h
#define __included_jps_grid_h

#include <stdbool.h>
#include "core/map/NavigationMap.hpp"

extern int malloc_count;

#define SETWALKABLE(x, walkable_index, flag) (x = flag ? (x | walkable_index) : (x & walkable_index))
#define GETWALKABLE(x, walkable_index) ((x & walkable_index) == 0 ? false : true)

enum WalkableIndex : unsigned char {
    DYNAMIC_BUFFER_TRUE = 0x01,
    STATIC_BUFFER_TRUE = 0x02,
    DYNAMIC_WALKABLE_TRUE = 0x04,
    STATIC_WALKABLE_TRUE = 0x08,
    WALKABLE_TRUE = 0x10,
    VIEW_WALKABLE_TRUE = 0x20,

    INIT_WALKABLE_FALSE = 0x24,
    INIT_WALKABLE_TRUE = 0x3C,

    VIEW_WALKABLE_FALSE = 0xDF,
    WALKABLE_FALSE = 0xEF,
    STATIC_WALKABLE_FALSE = 0xF7,
    DYNAMIC_WALKABLE_FALSE = 0xFB,
    STATIC_BUFFER_FALSE = 0xFD,
    DYNAMIC_BUFFER_FALSE = 0xFE,
};

/*
 * Contains all relevant information for a position in the grid
 * Every Bit present: 0|0|view_walkable|walkable|static_walkable|dynamic_walkable|static_buffer|dynamic_buffer
 * walkable = static_walkable & dynamic_walkable & !static_buffer & !dynamic_buffer
 */
struct node {
    unsigned char grid_layered_walkable_;
};

/* Forms the grid */
struct grid {
    int width, height;
    struct node **nodes;
};

/* New Node */
struct node createNode(unsigned char walkable);

/* Create the grid based on with, height and a matrix */
struct grid createGrid(int width, int height, sros::map::PyramidNavMap_ptr matrix);

/* Build the 2D node grid from the matrix */
struct node **_buildNodes(int width, int height, sros::map::PyramidNavMap_ptr matrix);

/* Return a pointer to a node, identified by the x and y coordinates */
struct node *getNodeAt(struct grid *gd, int x, int y);

/* Return True / False whether a node is walkable or not ( automatically performs isInside check ) */
bool isWalkableAt(struct grid *gd, int x, int y);

/* Return True / False whether a node is inside the grid or not */
bool isInside(struct grid *gd, int x, int y);

/* Allows you to manually set a cell to walkable TRUE / FALSE. Generally not used. */
void setWalkableAt(struct grid *gd, int x, int y, bool walkable);

void convertMapCoords(int offset_x, int offset_y, double x, double y, int *cx, int *cy);

void reverseMapCoords(int offset_x, int offset_y, int cx, int cy, double *x, double *y);

#endif
