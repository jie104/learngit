#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../include/path.h"

/* Activate for a LOT of debug output */
#define DEBUG 0
#define DEBUG_JUMP 0

int euclidean(int dx, int dy) {
    double distance = sqrt((double) (dx * dx + dy * dy)) * 10.0;
    int idistance = (int) distance;
    if (DEBUG)
        printf("Euclidean engaged! regular:%lf , (int)*10:%d\n", distance, idistance);
    return idistance;
}

int manhattan(int dx, int dy) {
    int distance = (dx + dy) * 10;
    if (DEBUG)
        printf("Manhattan engaged! regular:%d        , (int)*10:%d\n", (dx + dy), distance);
    return distance;
}

int *_jump(struct grid *gd, int x, int y, int px, int py, struct node *endNode) {
    int dx = x - px;
    int dy = y - py;
    int *jx, *jy;
    if (DEBUG)
        printf("    _jump attempt: x:%d/y:%d px:%d/py:%d dx:%d/dy:%d\n", x, y, px, py, dx, dy);
    if (!isWalkableAt(gd, x, y)) {
        if (DEBUG)
            printf("    x:%d/y:%d is not walkable. Exiting _jump\n", x, y);
        return NULL;
    } else if (getNodeAt(gd, x, y) == endNode) {
        int *i = (int *) malloc(2 * sizeof(int));
        if (DEBUG_JUMP)
            printf("    _jump(1) return value: x:%d/y:%d\n", x, y);
        malloc_count++; /* [ Malloc Count ] */
        i[0] = x;
        i[1] = y;
        return i;
    }

    if (dx != 0 && dy != 0) {
        if ((isWalkableAt(gd, (x - dx), (y + dy)) && !isWalkableAt(gd, (x - dx), y)) ||
            (isWalkableAt(gd, (x + dx), (y - dy)) && !isWalkableAt(gd, x, (y - dy)))) {
            int *i = (int *) malloc(2 * sizeof(int));
            if (DEBUG_JUMP)
                printf("    _jump(2) return value: x:%d/y:%d\n", x, y);
            malloc_count++;         /* [ Malloc Count ] */
            i[0] = x;
            i[1] = y;
            return i;
        }
    } else {
        if (dx != 0) {
            if ((isWalkableAt(gd, (x + dx), (y + 1)) && !isWalkableAt(gd, x, (y + 1))) ||
                (isWalkableAt(gd, (x + dx), (y - 1)) && !isWalkableAt(gd, x, (y - 1)))) {
                int *i = (int *) malloc(2 * sizeof(int));
                if (DEBUG_JUMP)
                    printf("    _jump(3) return value: x:%d/y:%d\n", x, y);
                malloc_count++;         /* [ Malloc Count ] */
                i[0] = x;
                i[1] = y;
                return i;
            }
        } else {
            if ((isWalkableAt(gd, (x + 1), (y + dy)) && !isWalkableAt(gd, (x + 1), y)) ||
                (isWalkableAt(gd, (x - 1), (y + dy)) && !isWalkableAt(gd, (x - 1), y))) {
                int *i = (int *) malloc(2 * sizeof(int));
                if (DEBUG_JUMP)
                    printf("    _jump(4) return value: x:%d/y:%d\n", x, y);
                malloc_count++;         /* [ Malloc Count ] */
                i[0] = x;
                i[1] = y;
                return i;
            }
        }
    }

    if (dx != 0 && dy != 0) {
        if (DEBUG)
            printf("    Recursive _jumping(1) with ( x:%d/y:%d px:%d/py:%d )\n", (x + dx), y, x, y);
        jx = _jump(gd, (x + dx), y, x, y, endNode);

        if (DEBUG)
            printf("    Recursive _jumping(2) with ( x:%d/y:%d px:%d/py:%d )\n", x, (y + dy), x, y);
        jy = _jump(gd, x, (y + dy), x, y, endNode);

        if (jx || jy) {
            int *i;

            if (DEBUG_JUMP)
                printf("    _jump(5) return value: x:%d/y:%d\n", x, y);

            if (jx) {
                free(jx);
                malloc_count--; /* [ Malloc Count ] */
            }
            if (jy) {
                free(jy);
                malloc_count--; /* [ Malloc Count ] */
            }

            i = (int *) malloc(2 * sizeof(int));
            malloc_count++; /* [ Malloc Count ] */
            i[0] = x;
            i[1] = y;
            return i;
        }
    }

    if (isWalkableAt(gd, (x + dx), y) || isWalkableAt(gd, x, (y + dy))) {
        if (DEBUG)
            printf("    Recursive _jumping(3) with ( x:%d/y:%d px:%d/py:%d )\n", (x + dx), (y + dy), x, y);
        return _jump(gd, (x + dx), (y + dy), x, y, endNode);
    } else {
        if (DEBUG)
            printf("    Returning NULL\n");
        return NULL;
    }
}

void _identifySuccessors(struct grid *gd,
                         struct node *activeNode, int activeX, int activeY,
                         struct open_list *current,
                         struct node *endNode, int endX, int endY,
                         sp_tree &sptree,
                         node_evaluation_map &ne_map) {
    int *jumpPoint, jg, jh, jf, ag, ah, af;
    bool jump_opened, jump_closed, act_opened, act_closed;
    struct neighbor_xy_list *neighbors_head = _findNeighbors(gd, activeNode, activeX, activeY, sptree);
    struct neighbor_xy_list *neighbors_current = neighbors_head;
    while (neighbors_head != (neighbors_current = neighbors_current->right)) {
        if (DEBUG) {
            if (isWalkableAt(gd, neighbors_current->x, neighbors_current->y))
                printf("Neighbor x:%d/y:%d is walkable!\n", neighbors_current->x, neighbors_current->y);
            else
                printf("Neighbor x:%d/y:%d is NOT walkable!\n", neighbors_current->x, neighbors_current->y);
        }
        jumpPoint = _jump(gd, neighbors_current->x, neighbors_current->y, activeX, activeY, endNode);
        if (DEBUG)
            printf("Jump point not set!\n\n");
        if (jumpPoint != NULL) {
            int jx, jy, d, ng;
            if (DEBUG)
                printf("Jump point set!\n\n");
            jx = jumpPoint[0];
            jy = jumpPoint[1];
            //printf("jump_point: %d  %d\n",jy,jx);
            free(jumpPoint);
            malloc_count--; /* [ Malloc Count ] */

            node_evaluation_getvalue(jx, jy, jg, jh, jf, jump_opened, jump_closed, ne_map);
            if (jump_closed) {
                continue;
            }

            d = euclidean(abs(jx - activeX), abs(jy - activeY));
            node_evaluation_getvalue(activeX, activeY, ag, ah, af, act_opened, act_closed, ne_map);
            ng = ag + d;
            if (!jump_opened || ng < jg) {
                jg = ng;
                if (!jh)
                    jh = manhattan(abs(jx - endX), abs(jy - endY));
                /* jumpNode->h = jumpNode->h || manhattan(abs(jx - endX), abs(jy - endY)); // ASK FIDELIS !! */
                jf = jg + jh;
                if (DEBUG)
                    printf("Node g:%d h:%d f:%d\n", jg, jh, jf);
                node_evaluation_insert(jx, jy, jg, jh, jf, ne_map);
                sp_tree_insert(jx, jy, activeX, activeY, sptree);
                if (!jump_opened) {
                    current = ol_insert_right(current, jx, jy, jf);
                    //printf("jump_point:(%d %d)\n", current->x, current->y);
                    node_evaluation_modify_opened(jx, jy, true, ne_map);
                } else {
                    ol_listsort(current->right);
                }
            }
        }
    }
    neighbor_xy_clean(neighbors_head);
}

void _astar(struct grid *gd,
            struct node *activeNode, int activeX, int activeY,
            struct open_list *current,
            int endX, int endY,
            sp_tree &sptree,
            node_evaluation_map &ne_map) {
    int jg, jh, jf, ag, ah, af;
    bool jump_opened, jump_closed, act_opened, act_closed;
    struct neighbor_xy_list *neighbors_head = _findNeighbors(gd, activeNode, activeX, activeY, sptree);
    struct neighbor_xy_list *neighbors_current = neighbors_head;
    while (neighbors_head != (neighbors_current = neighbors_current->right)) {
        if (neighbors_current != NULL) {
            int jx, jy, d, ng;
            jx = neighbors_current->x;
            jy = neighbors_current->y;
            node_evaluation_getvalue(jx, jy, jg, jh, jf, jump_opened, jump_closed, ne_map);
            if (jump_closed) {
                continue;
            }
            d = euclidean(abs(jx - activeX), abs(jy - activeY));
            node_evaluation_getvalue(activeX, activeY, ag, ah, af, act_opened, act_closed, ne_map);
            ng = ag + d;
            if (!jump_opened || ng < jg) {
                jg = ng;
                if (!jh)
                    jh = manhattan(abs(jx - endX), abs(jy - endY));
                jf = jg + jh;
                node_evaluation_insert(jx, jy, jg, jh, jf, ne_map);
                sp_tree_insert(jx, jy, activeX, activeY, sptree);
                if (!jump_opened) {
                    current = ol_insert_right(current, jx, jy, jf);
                    //printf("jump_point:(%d %d)\n", current->x, current->y);
                    node_evaluation_modify_opened(jx, jy, true, ne_map);
                } else {
                    ol_listsort(current->right);
                }
            }
        }
    }
}

struct neighbor_xy_list *backtrace(int endX, int endY, sp_tree &sptree) {
    int nextX = endX, nextY = endY, preX, preY, waypoint = 1;
    struct neighbor_xy_list *head = neighbor_xy_new();
    struct neighbor_xy_list *current = head;
    current = neighbor_xy_insert_right(current, endX, endY);

    while (sp_tree_getvalue(nextX, nextY, preX, preY, sptree)) {
//        std::cout << "nextX nextY, preX preY: " << nextX << " " << nextY << " " << preX << " " << preY << std::endl;
        current = neighbor_xy_insert_right(current, preX, preY);
        nextX = preX;
        nextY = preY;
        ++waypoint;
    }
    std::cout << "路径点数: " << waypoint << std::endl;
    return head;
}

struct neighbor_xy_list *findPath(struct grid *gd, int startX, int startY, int endX, int endY,bool localplan_or_not) {
    struct open_list *head = ol_new();
    struct open_list *current = head;
    struct node *startNode = getNodeAt(gd, startX, startY);
    struct node *endNode = getNodeAt(gd, endX, endY);
    struct node *activeNode;
    sp_tree sptree;
    node_evaluation_map ne_map;
    int counter = 0, activeX, activeY;

    /* Initialize the start node */
    current = ol_insert_right(current, startX, startY, 0);
    node_evaluation_insert(startX, startY, 0, 0, 0, ne_map);
    node_evaluation_modify_opened(startX, startY, true, ne_map);

    head = ol_listsort(head);
    current = head->left;

    while (head != current) { /* List is empty when current marker rests on head */
        if (DEBUG)
            printf("Cycle %d\n", counter);

        activeX = current->x;
        activeY = current->y;
        activeNode = getNodeAt(gd, activeX, activeY);
        current = ol_del_free(current);

        node_evaluation_modify_closed(activeX, activeY, true, ne_map);

        if (activeNode == endNode) {
            struct neighbor_xy_list *goal;
            goal = backtrace(endX, endY, sptree);
            ol_clean(head);
            printf("搜索的地图点数: %d\n", counter);
            LOG(INFO)<<"搜索的地图点数"<<counter;
            return goal;
        }

        /* Begin identifying successors... */
        _identifySuccessors(gd, activeNode, activeX, activeY, current,
                            endNode, endX, endY,
                            sptree, ne_map);

//        _astar(gd, activeNode, activeX, activeY, current, endX, endY, sptree, ne_map);

        head = ol_listsort(head);

        /* Instead of sorting it everytime a item is added, I will sort when I need to grab the lowest value. */
        current = head->right;

        /*
        if (DEBUG) {
            listOpenList(gd, head);
            printf("Currently active node:\n");
            displayNodeInfo(gd, current->x, current->y);
        }
        */
        counter++;
        int max_counter = 5000;
        if(localplan_or_not){
            max_counter = 50;
        }
        else{
            max_counter = 5000;
        }
        if (counter >= max_counter) {
            ol_clean(head);
            //printf("搜索的地图点数: %d\n", counter);
            //printf("\n----------\nLimit reached\n----------\n");
            LOG(INFO)<<"搜索的地图点数大于5000.返回。"<<counter;
            return NULL;
        }

    }
    ol_clean(head);
    printf("搜索的地图点数: %d\n", counter);
    printf("\n----------\nReturning NULL because head = current\n----------\n");
    LOG(INFO)<<"搜索的地图点数"<<counter<<";checkpoint:"<<current->x<<";"<<current->y;
    return NULL;
}

struct neighbor_xy_list *smooth_path(struct grid *gd, struct neighbor_xy_list *head) {
    struct neighbor_xy_list *pos = head->left;
    int xi, yi, dx, dy;
    while (head != NULL && (head != (pos = pos->left))) {
        xi = pos->x;
        yi = pos->y;
        dx = xi - pos->right->x;
        dy = yi - pos->right->y;
        if (dx == 1 && dy == -1) { /* Up & Right */
            if (!isWalkableAt(gd, xi, yi + 1)) {
                pos = neighbor_xy_insert_right(pos, pos->x - 1, pos->y);
            } else if (!isWalkableAt(gd, xi - 1, yi)) {
                pos = neighbor_xy_insert_right(pos, pos->x, pos->y + 1);
            }
        } else if (dx == 1 && dy == 1) {    /* Down & Right */
            if (!isWalkableAt(gd, xi - 1, yi)) {
                pos = neighbor_xy_insert_right(pos, pos->x, pos->y - 1);
            } else if (!isWalkableAt(gd, xi, yi - 1)) {
                pos = neighbor_xy_insert_right(pos, pos->x - 1, pos->y);
            }
        } else if (dx == -1 && dy == 1) {    /* Down & Left */
            if (!isWalkableAt(gd, xi, yi - 1)) {
                pos = neighbor_xy_insert_right(pos, pos->x + 1, pos->y);
            } else if (!isWalkableAt(gd, xi + 1, yi)) {
                pos = neighbor_xy_insert_right(pos, pos->x, pos->y - 1);
            }
        } else if (dx == -1 && dy == -1) {    /* Up & Left */
            if (!isWalkableAt(gd, xi + 1, yi)) {
                pos = neighbor_xy_insert_right(pos, pos->x, pos->y + 1);
            } else if (!isWalkableAt(gd, xi, yi + 1)) {
                pos = neighbor_xy_insert_right(pos, pos->x + 1, pos->y);
            }
        } else if (abs(dx) > 1 || abs(dy) > 1) {
            int incrX = dx / std::max(abs(dx), 1);
            int incrY = dy / std::max(abs(dy), 1);
            pos = neighbor_xy_insert_right(pos, pos->right->x + incrX, pos->right->y + incrY);
        }
    }
    return head;
}
