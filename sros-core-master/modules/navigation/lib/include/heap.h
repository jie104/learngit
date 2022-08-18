#ifndef __included_heap_h
#define __included_heap_h

#include "jps_grid.h"

extern int malloc_count;

/* Circular Doubly Linked List that contains a list of all open nodes ( nodes eligible for investigation ) */
struct open_list {
    struct open_list *left;
    struct open_list *right;
    int x;
    int y;
    int f;
};

/* evaluating value of each node. f = g + h.*/
struct evaluation_value {
    int g;
    int h;
    int f;
    bool opened;
    bool closed;
};

/* x,y node map to it's evaluate value */
typedef std::map<int64_t, struct evaluation_value> node_evaluation_map;

/* insert node-evaluation to map */
void node_evaluation_insert(int x, int y, int g, int h, int f, node_evaluation_map &ne_map);

/* get node's evaluation */
bool node_evaluation_getvalue(int x, int y,
                              int &g, int &h, int &f,
                              bool &opened, bool &closed,
                              node_evaluation_map &ne_map);

/* modify node opened state to map */
void node_evaluation_modify_opened(int x, int y, bool opened, node_evaluation_map &ne_map);

/* modify node closed state to map */
void node_evaluation_modify_closed(int x, int y, bool closed, node_evaluation_map &ne_map);

/* New list */
struct open_list *ol_new();

/* Clean list */
void ol_clean(struct open_list *head);

/* Add to list */
struct open_list *ol_insert_right(struct open_list *list, int x, int y, int f);

/* Used to delete a entry & free the ressources */
struct open_list *ol_del_free(struct open_list *list);

/* Comparison function. Used to determine which node should be examined next */
int cmp(struct open_list *one, struct open_list *two);

/* Perform a mergesort on the list ( pass & receive the list's head ) */
struct open_list *ol_listsort(struct open_list *list);

#endif
