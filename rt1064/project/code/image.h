#ifndef _image_h_
#define _image_h_


#include "zf_common_typedef.h"
#include "zf_common_font.h"
#include "zf_device_mt9v03x.h"

//#include "servo_pid.h"
//#include "element_handle.h"


#define WHITE_PIXEL                255
#define BLACK_PIXEL                0
//定义膨胀和腐蚀的阈值区间
#define THRESHOLD_MAX              255*5
#define THRESHOLD_MIN              255*2
//#define BORDERLEFT                 1
//#define BORDERRIGHT                254
#define MID_W                      94
// 定义邻近拐点的阈值（行数差≤此值则合并，可根据实际调整）
#define BREAKPOINT_MERGE_THRESHOLD 6
#define LEFT_MAX                   4
#define RIGHT_MAX                  183
#define OFFSET                     10
//#define ADVANCER_PERCEIVE          22

#define SCALING_RATIO              0.4

extern uint8 car_stop_flag;
extern float final_mid_line;
extern uint8 Zebra_stop_flag;
extern uint8 Left_Lost_Time;
extern uint8 Right_Lost_Time;
extern uint8 road_left[120];
extern uint8 road_right[120];
extern uint8 line;
extern uint8 l_d_num;
extern uint8 l_u_num;
extern uint8 r_d_num;
extern uint8 r_u_num;
extern float line_error;
extern uint8 huandao_flag;
extern uint8 huandao_num_flag;
extern uint8 crossing_flag_help;
extern uint8 num_line;
extern int straight_speed;
extern int turn_speed;
extern int yuanhuan_speed;
extern int left_right_choose;
extern uint8 qianzhan_value;
extern uint8 qianzhan_change_value;
extern uint8 Zebra_end_flag;
extern int start_accelerate;
extern int end_decelerate;
extern int slow_turn_speed;
extern int turn_num;
extern uint8 car_go_flag;
extern volatile uint8 push_box_flag;
extern volatile float now_rotate_angle;


//uint8 otsuThreshold_less(uint8 *image, uint16 col, uint16 row);
void image_postprocess(void);
void image_filter(uint8(*image)[MT9V03X_W]);
void car_stop(void);
uint8 get_left_start(uint8 num);
uint8 get_right_start(uint8 num);
void research_longest_line(void);
void research_road(void);
void image_process(void);
void image_show(void);
void image_data_clear(void);
void xieji(uint8 begin, uint8 end, uint8 y_begin, uint8 y_end,uint8 arr[]);
void least_squares(uint8 begin, uint8 end, uint8 *border, float *xielv, float *jieju);
void crossing_add(uint8 num_d_l, uint8 num_u_l, uint8 num_d_r, uint8 num_u_r);
uint8 my_limit_min(uint8 num1, uint8 num2);
void straight_line_handle(void);
float find_mid_line_weight(void);
void Zebra_crossing_handle(void);
int Continuity_Change_Left(int start,int end, uint8 line);
int Continuity_Change_Right(int start,int end, uint8 line);
//uint8  yuanhuan_see_handle(void);
void yuanhuan_right_in_handle(void);
void huandao_clear(void);
uint8 research_20point(uint8 num,uint8 arr[]);
int find_min_min(uint8 arr[], int start, int end);
void yuanhuan_left_in_handle(void);
void speed_strategy(void);
uint8 Detect_S_Curve(uint8 *center, uint8 start);
void color_binary_detection(void);

#endif

