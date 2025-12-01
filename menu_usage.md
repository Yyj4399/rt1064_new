# 菜单重构与使用说明

## 本次改动
- 重写菜单为数据驱动的`menu_item`树结构，支持子菜单、参数、动作、视图四类节点，新增参数配置字段（步进精度、最值、只读标记）。
- 新的导航状态机：导航/编辑/视图三态，减少层级编码逻辑，进入/退出统一，用`need_clear`控制刷新。
- CarGo 启停改为显式动作：`Start`按确认键启动，`Stop`清零目标速度并发送停止控制，可在其他菜单继续调参。
- 参数显示统一在列表右侧，编辑时显示`*`标记，第一批参数默认步进设为1（更安全）。
- 路径查看改为分页视图，进入后用 K1/K2 翻页，底部提示当前页数。
- Flash 菜单包含保存/加载/清除三动作，并在状态栏提示结果。
- 图像页、状态页保持原有信息展示，并将可调/只读参数区分显示。

## 操作方式
- 导航：K1 上移，K2 下移，K3 进入/确认，K4 返回。
- 编辑参数：选中参数后按 K3 进入编辑，K1 减小，K2 增大，K3 切换步进（100 → 10 → 1 → 0.1 → 0.01 循环），K4 退出编辑。
- 视图页：
  - Path 视图需按 K3 进入后，K1 上一页、K2 下一页、K4 退出。
  - 其它视图（Camera/PID/Image）按 K4 退出即可。
- 启停：在 CarGo 菜单选中 `Start` 按 K3 启动；选中 `Stop` 按 K3 停车（立即清零速度并发送停止控制）。
- 参数保存：Status → Flash → 选择 `Save/Load/Clear`，按 K3 执行，结果会在右上角提示。

## 扩展指引（添加新菜单/参数）
1. 在 `menu.c` 中声明新的 `menu_item_t`，根据需要设置 `type`：
   - `MENU_ITEM_PAGE`：子菜单，需要在 children 数组中注册；
   - `MENU_ITEM_PARAM_INT/FLOAT`：可调参数，设置 `.param.value_ptr`、`min/max`、`precision`、`width`，只读参数将 `read_only` 置 1；
   - `MENU_ITEM_ACTION`：绑定 `.action` 回调；
   - `MENU_ITEM_VIEW`：绑定 `.draw` 回调用于渲染详细信息。
2. 将新节点加入对应 children 数组，`menu_bind_parents` 会自动建立父子关系。
3. 复杂页面可编写独立 `draw_xxx` 函数，在列表右侧或底部输出数据，避免遮挡列表区（x≥120 更安全）。
4. 如需自定义键行为，可在 `menu_handle_view_keys` 中加入对应视图的处理逻辑。

## 添加新的参数/子菜单的详细步骤
以下步骤默认你保持原有风格与硬件限制（IPS200 + 四按键），仅调整数据结构即可：

1) 选择位置与类型  
   - 新增子菜单：创建 `menu_item_t new_page = { .title = "XXX", .type = MENU_ITEM_PAGE, .children = (const menu_item_t *const *)xxx_children, .child_count = ARRAY_SIZE(xxx_children) };`。  
   - 新增参数：根据数值类型选择 `MENU_ITEM_PARAM_INT` 或 `MENU_ITEM_PARAM_FLOAT`，如 `menu_item_t new_param = { .title = "gain", .type = MENU_ITEM_PARAM_FLOAT, .param = { .value_ptr = &your_var, .min = 0, .max = 10, .precision = 2, .width = 3 } };`。若只读展示，将 `.read_only = 1`。
   - 新增动作：使用 `MENU_ITEM_ACTION`，设置 `.action = your_fn`，回调签名为 `void your_fn(void)`。
   - 新增视图：使用 `MENU_ITEM_VIEW`，设置 `.draw = your_draw_fn`，回调签名为 `void your_draw_fn(void)`，在其中调用 `ips200_show_*` 输出内容。

2) 关联父菜单  
   - 找到目标父菜单的 children 数组（例如 `main_children`、`param_children` 等），把新节点指针追加进去，并更新 `child_count = ARRAY_SIZE(该数组)`。  
   - 如果是全新父菜单，需要先定义该父菜单，再定义其 children 数组，最后把父菜单接到更高层的 children 中。

3) 绑定父子关系  
   - 不需要手工写 parent，`menu_bind_parents(&main_menu);` 在 `menu_init` 中已执行，会自动回填 `parent`，确保返回操作正常工作。

4) 校验显示与步进  
   - `width` 控制显示宽度，整数类型用 `ips200_show_int`，浮点用 `ips200_show_float`，`precision` 决定小数位。  
   - 步进值来源于全局 `level` 数组（100/10/1/0.1/0.01），编辑模式下按 K3 切换；如需特殊步进，可在 `menu_handle_edit_adjust` 中按 `active_item` 条件定制。

5) 保存/加载顺序（涉及参数持久化）  
   - 若新参数需要存 Flash：在 `param_save_to_flash` 和 `param_load_from_flash` 中追加同一顺序的 buffer 读写，注意使用正确的 `int32_type` 或 `float_type`。  
   - 确认 `FLASH_PAGE_INDEX` 空间足够；当前占用 21 项，可酌情扩展顺序，但要兼容旧数据请保持原有排序在前，新参数追加在末尾。

6) 调试与显示布局  
   - 避免覆盖列表区域：列表从 y=16 开始往下 16 像素一行，详细信息建议 x>=120 或 y>=176 处绘制。  
   - 视图页如需分页，参考 `draw_status_path` 的实现，结合 `menu_handle_view_keys` 添加翻页逻辑。

简易示例：在 Speed 菜单新增一个只读显示的 `cur_speed`  
```c
static int cur_speed = 0; // 需在其它模块定期更新
static menu_item_t cur_speed_item = {
    .title = "cur_speed",
    .type = MENU_ITEM_PARAM_INT,
    .param = { .value_ptr = &cur_speed, .read_only = 1, .width = 5 },
};
static menu_item_t *const speed_children[] = { &base_speed_item, ..., &gyro_kd_item, &cur_speed_item };
// child_count = ARRAY_SIZE(speed_children);
```
示例：新增“调试”子菜单，包含一个浮点参数与一个动作
```c
// 1) 声明节点
static float debug_gain = 0.5f;          // 调试增益，默认 0.5，可在菜单内修改
static menu_item_t debug_gain_item = {
    .title = "dbg_gain",
    .type = MENU_ITEM_PARAM_FLOAT,
    .param = { .value_ptr = &debug_gain, .min = 0.0f, .max = 5.0f, .precision = 2, .width = 4 },
};
static void action_reset_state(void) {
    // 在这里清空你的状态，比如滤波器/计数器
}
static menu_item_t debug_reset_item = {
    .title = "reset",
    .type = MENU_ITEM_ACTION,
    .action = action_reset_state,
};

// 2) 组装子菜单与父菜单
static menu_item_t *const debug_children[] = { &debug_gain_item, &debug_reset_item }; // 调试子菜单的孩子节点
static menu_item_t debug_menu = {
    .title = "Debug",
    .type = MENU_ITEM_PAGE,
    .children = (const menu_item_t *const *)debug_children,
    .child_count = ARRAY_SIZE(debug_children),
};

// 3) 把 Debug 菜单挂到主菜单（例如放在最后）
static menu_item_t *const main_children[] = { &cargo_menu, &param_menu, &status_menu, &image_menu, &debug_menu }; // 主菜单孩子节点
// main_menu.child_count = ARRAY_SIZE(main_children);

// 4) 如果需要保存/加载 debug_gain，记得在 param_save_to_flash/param_load_from_flash 末尾追加：
// flash_union_buffer[21].float_type = debug_gain; // 保存
// debug_gain = flash_union_buffer[21].float_type; // 载入
```

示例：新增一个可修改的整型参数（带注释）
```c
// 1) 声明变量
static int test_threshold = 50; // 测试阈值，默认 50，可在菜单调节范围 0~255

// 2) 声明菜单项
static menu_item_t test_threshold_item = {
    .title = "test_th",                         // 菜单显示名称
    .type  = MENU_ITEM_PARAM_INT,               // 整型可调参数
    .param = {
        .value_ptr = &test_threshold,           // 绑定的变量地址
        .min       = 0,                         // 最小值限制
        .max       = 255,                       // 最大值限制
        .precision = 0,                         // 整型无需小数位
        .width     = 4,                         // 显示宽度
        .read_only = 0                          // 0 表示可修改
    },
};

// 3) 添加到目标父菜单（假设挂到 Speed 菜单最后）
static menu_item_t *const speed_children[] = {
    &base_speed_item, &turn_speed_item, &speed_k_item, &speed_limit_item,
    &yuanhuan_speed_item, &gyro_kp_item, &gyro_ki_item, &gyro_kd_item,
    &test_threshold_item      // 新增项
};
// param_speed_menu.child_count = ARRAY_SIZE(speed_children);

// 4) 若需持久化到 Flash，保持顺序一致：
// flash_union_buffer[21].int32_type = test_threshold; // 保存
// test_threshold = flash_union_buffer[21].int32_type; // 载入
```
这样就能在列表右侧显示当前速度值，按键不会修改（只读）。

## 兼容性注意
- 保留所有原有参数、状态信息和 Flash 存储顺序，旧数据仍可加载。
- 显示/按键仍基于 IPS200 与四按键，未引入额外依赖。
- 默认步进由 100 调整为 1，避免意外大步调参，如需大步调节可在编辑模式下多次按 K3 切换。

## width 与取值范围说明
- `width` 是数字输出的字符宽度（对应 `ips200_show_int/float` 的位数参数），用于控制整数部分占多少位。每个字符在 IPS200 上约 8×16 像素，竖屏宽度 240 像素，列表区从 x≈16 开始，数值区常放在 x≈140 之后；预估占用像素可用 `width*8 + 符号 + 小数点 + 小数位*8` 粗算，避免越界。
- 浮点显示总字符数≈ `width + 1(小数点) + precision + 1(可能的符号)`，设置 `width` 时需要给整数部分足够位数，否则会截断或顶到屏幕边缘。
- `min/max` 仅用于菜单调节范围，类型上限由 `int32`/`float` 决定；实际限制应结合硬件/控制安全范围来设。若需要很大的范围，保证 `width` 足够展示（如 6 位可覆盖 ±999999），否则数值会显示不全。
- 如果参数只读展示，可把 `read_only` 设为 1，并酌情缩小 `width` 以节省屏幕空间；可调参数建议留足 1 位符号和当前范围的最大整数位。
