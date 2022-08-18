#include <stdlib.h>

#include "../include/neighbors.h"

struct neighbor_xy_list *neighbor_xy_new() {
    struct neighbor_xy_list *newlist = (struct neighbor_xy_list *) malloc(sizeof(struct neighbor_xy_list));
    malloc_count++; /* [ Malloc Count ] */
    newlist->right = newlist;
    newlist->left = newlist;
    newlist->x = -1;
    newlist->y = -1;
    return newlist;
}

void neighbor_xy_clean(struct neighbor_xy_list *head) {
    if (head != NULL) {
        struct neighbor_xy_list *pos = head;
        struct neighbor_xy_list *tmp = head;
        do {
            tmp = pos->right;
            free(pos);
            malloc_count--; /* [ Malloc Count ] */
            pos = tmp;
        } while (pos != head);
    }
}

struct neighbor_xy_list *neighbor_xy_insert_right(struct neighbor_xy_list *list, int x, int y) {
    struct neighbor_xy_list *newlist = (struct neighbor_xy_list *) malloc(sizeof(struct neighbor_xy_list));
    malloc_count++;         /* [ Malloc Count ] */
    newlist->x = x;
    newlist->y = y;
    newlist->left = list;
    newlist->right = list->right;
    list->right = newlist;
    newlist->right->left = newlist;
    return newlist;
}

void sp_tree_insert(int x, int y, int px, int py, sp_tree &sptree) {
    int64_t key, value;
    key = ((int64_t) x << 32) | y;
    value = ((int64_t) px << 32) | py;
    if (sptree.find(key) == sptree.end()) {
        sptree.insert(sp_tree::value_type(key, value));
    } else {
        sptree[key] = value;
    }
}

bool sp_tree_getvalue(int x, int y, int &px, int &py, sp_tree &sptree) {
    int64_t key, value;
    key = ((int64_t) x << 32) | y;
    if (sptree.find(key) == sptree.end()) {
        return false;
    } else {
        value = sptree[key];
        px = value >> 32;
        py = value & 0xFFFFFFFF;
        return true;
    }
}

struct neighbor_xy_list *getNeighbors(struct grid *gd, int x, int y) {
    struct neighbor_xy_list *head = neighbor_xy_new();
    struct neighbor_xy_list *current = head;

    bool d0 = false;
    bool d1 = false;
    bool d2 = false;
    bool d3 = false;

    /* UP */
    if (isWalkableAt(gd, x, y - 1)) {
        current = neighbor_xy_insert_right(current, x, y - 1);
        d0 = d1 = true;
    }

    /* RIGHT */
    if (isWalkableAt(gd, x + 1, y)) {
        current = neighbor_xy_insert_right(current, x + 1, y);
        d1 = d2 = true;
    }

    /* DOWN */
    if (isWalkableAt(gd, x, y + 1)) {
        current = neighbor_xy_insert_right(current, x, y + 1);
        d2 = d3 = true;
    }

    /* LEFT */
    if (isWalkableAt(gd, x - 1, y)) {
        current = neighbor_xy_insert_right(current, x - 1, y);
        d3 = d0 = true;
    }

    /* UP + LEFT */
    if (d0 && isWalkableAt(gd, x - 1, y - 1)) {
        current = neighbor_xy_insert_right(current, x - 1, y - 1);
    }

    /* UP + RIGHT */
    if (d1 && isWalkableAt(gd, x + 1, y - 1)) {
        current = neighbor_xy_insert_right(current, x + 1, y - 1);
    }

    /* DOWN + RIGHT */
    if (d2 && isWalkableAt(gd, x + 1, y + 1)) {
        current = neighbor_xy_insert_right(current, x + 1, y + 1);
    }

    /* DOWN + LEFT */
    if (d3 && isWalkableAt(gd, x - 1, y + 1)) {
        current = neighbor_xy_insert_right(current, x - 1, y + 1);
    }

    return head;
}

struct neighbor_xy_list *_findNeighbors(struct grid *gd, struct node *activeNode, int x, int y, sp_tree &sptree) {
    int px, py, dx, dy;

    struct neighbor_xy_list *head = neighbor_xy_new();
    struct neighbor_xy_list *current = head;

    struct neighbor_xy_list *neighborNodes_head;
    struct neighbor_xy_list *neighborNodes_current;

    if (sp_tree_getvalue(x, y, px, py, sptree)) {

        dx = (x - px) / std::max(abs(x - px), 1);
        dy = (y - py) / std::max(abs(y - py), 1);

        /* Diagonals */
        if (dx != 0 && dy != 0) {
            if (isWalkableAt(gd, x, (y + dy))) {
                current = neighbor_xy_insert_right(current, x, (y + dy));
            }
            if (isWalkableAt(gd, (x + dx), y)) {
                current = neighbor_xy_insert_right(current, (x + dx), y);
            }
            if (isWalkableAt(gd, x, (y + dy)) || isWalkableAt(gd, (x + dx), y)) {
                current = neighbor_xy_insert_right(current, (x + dx), (y + dy));
            }
            if (!isWalkableAt(gd, (x - dx), y) && isWalkableAt(gd, x, (y + dy))) {
                current = neighbor_xy_insert_right(current, (x - dx), (y + dy));
            }
            if (!isWalkableAt(gd, x, (y - dy)) && isWalkableAt(gd, (x + dx), y)) {
                current = neighbor_xy_insert_right(current, (x + dx), (y - dy));
            }

            /* Horizontal / Vertical */
        } else {
            if (dx == 0) {
                if (isWalkableAt(gd, x, (y + dy))) {
                    if (isWalkableAt(gd, x, (y + dy))) {
                        current = neighbor_xy_insert_right(current, x, (y + dy));
                    }
                    if (!isWalkableAt(gd, (x + 1), y)) {
                        current = neighbor_xy_insert_right(current, (x + 1), (y + dy));
                    }
                    if (!isWalkableAt(gd, (x - 1), y)) {
                        current = neighbor_xy_insert_right(current, (x - 1), (y + dy));
                    }
                }
            } else {
                if (isWalkableAt(gd, (x + dx), y)) {
                    if (isWalkableAt(gd, (x + dx), y)) {
                        current = neighbor_xy_insert_right(current, (x + dx), y);
                    }
                    if (!isWalkableAt(gd, x, (y + 1))) {
                        current = neighbor_xy_insert_right(current, (x + dx), (y + 1));
                    }
                    if (!isWalkableAt(gd, x, (y - 1))) {
                        current = neighbor_xy_insert_right(current, (x + dx), (y - 1));
                    }
                }
            }
        }
    } else {
        neighborNodes_head = getNeighbors(gd, x, y);
        neighborNodes_current = neighborNodes_head;
        while (neighborNodes_head != (neighborNodes_current = neighborNodes_current->right)) {
            current = neighbor_xy_insert_right(current, neighborNodes_current->x, neighborNodes_current->y);
        }
        neighbor_xy_clean(neighborNodes_head);
    }

    return head;
}
