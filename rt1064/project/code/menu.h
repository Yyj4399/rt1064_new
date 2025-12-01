#ifndef _menu_h_
#define _menu_h_

#include "zf_common_typedef.h"


#define GREEDY_AREA 10000

// 菜单项类型定义，便于统一管理
typedef enum
{
    MENU_ITEM_PAGE = 0,
    MENU_ITEM_PARAM_INT,
    MENU_ITEM_PARAM_FLOAT,
    MENU_ITEM_ACTION,
    MENU_ITEM_VIEW
} menu_item_type_t;

typedef struct menu_item menu_item_t;
typedef void (*menu_action_fn)(void);
typedef void (*menu_draw_fn)(void);

// 参数绑定配置，可选最小/最大值与显示精度
typedef struct
{
    void *value_ptr;
    float min;
    float max;
    uint8 read_only;
    uint8 precision;
    uint8 width;
} menu_param_cfg_t;

struct menu_item
{
    const char *title;
    menu_item_type_t type;
    const menu_item_t *const *children;
    uint8 child_count;
    menu_item_t *parent;
    menu_param_cfg_t param;
    menu_action_fn action;
    menu_draw_fn draw;
};

void menu_init(void);
void menu_display(void);
void menu_switch(void);

uint8 param_save_to_flash(void);
uint8 param_load_from_flash(void);

#endif
