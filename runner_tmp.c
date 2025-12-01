/*#include <stdio.h>
#include "assigned_box_planner_greedy.h"
#ifndef GREEDY_AREA
#define GREEDY_AREA 10000
#endif
int main(void){
 Point car={2,2};
 BoxTargetPair pairs[6]={{ {6,8},{3,5} },{{4,3},{4,4}},{{5,11},{2,10}},{{9,4},{10,10}},{{8,1},{13,0}},{{3,2},{13,15}}};
 Point obstacles[18]={{2,4},{3,4},{2,5},{4,5},{5,4},{5,5},{1,10},{1,11},{2,11},{10,9},{9,9},{9,10},{12,0},{12,1},{13,1},{13,14},{14,14},{14,15}};
 Point path[GREEDY_AREA]; size_t steps=0;
 int ret=plan_boxes_greedy(18,18,car,pairs,6,obstacles,18,path,GREEDY_AREA,&steps);
 printf("ret=%d steps=%zu\n", ret, steps);
 return 0;
}
*/