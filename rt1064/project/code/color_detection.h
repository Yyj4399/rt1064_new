#ifndef __COLOR_DETECTION_H_
#define __COLOR_DETECTION_H_

#include "zf_common_typedef.h"



#define TARGET_DISTANCE         16                 //目标到达距离
#define TARGET_X                160

extern uint8 camera_x_flag;
extern uint8 camera_distance_flag;

void color_distance_handle(void);
void Continuous_detection(int camera_x,float distance);

#endif
