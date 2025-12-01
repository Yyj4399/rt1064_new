# assigned_box_planner_greedy 说明

## 当前修改（版本 1）
- 保持逐步贪心推箱框架，但改为按综合评分挑选方向：到目标的曼哈顿距离 + 邻域拥挤度权重 + 反向惩罚 + 重复位置惩罚，避免贴障碍来回推。
- 每步都跑一遍小车 BFS 校验可达性；如果不可达则跳过该方向，不再硬推。
- 增加入参/上限校验：箱子最多 10 个，`pair_count==0`/`path_capacity==0` 直接报错；路径不足时返回错误而非静默截断。
- 简化循环检测：最近 20 个箱子位置用惩罚而非直接失败，超 2000 步仍会超时保护。
- `find_simple_path` 加入边界检查，限定网格总格子数不超过 400。

## 当前修改（版本 2）
- 方向选择优先按“推后到目标的曼哈顿距离”最小，再用评分（邻域/反向/重复惩罚）做次级排序，避免因局部拥挤度惩罚过大而远离目标致卡死。

## 函数使用
```c
int plan_boxes_greedy(int rows, int cols, Point car,
                      const BoxTargetPair *pairs, size_t pair_count,
                      const Point *obstacles, size_t obstacle_count,
                      Point *path_buffer, size_t path_capacity,
                      size_t *out_steps);
```
- `rows/cols`：网格行列数（行列乘积需 ≤ 400）。
- `car`：小车初始坐标。
- `pairs`/`pair_count`：箱子与目标列表（最多 10 对）。
- `obstacles`/`obstacle_count`：障碍坐标列表。
- `path_buffer`/`path_capacity`：输出小车行走路径的缓存及容量（仅小车轨迹，不含箱子）。
- `out_steps`：实际写入的步数。

返回值：
- 0 成功；
- -1 参数为空；
- -2 没有箱子；
- -3 箱子数量超上限(10)；
- -4 路径缓存容量为 0；
- -5 单箱迭代超步数；
- -6 当前箱子无可行推动方向；
- -7 路径缓存不足。

## 调用示例
```c
Point car = {0, 0};
BoxTargetPair pairs[] = {
    {{1, 1}, {3, 3}},
    {{2, 0}, {0, 3}},
};
Point obstacles[] = {{1, 2}, {2, 2}};

Point path[256];
size_t steps = 0;

int ret = plan_boxes_greedy(5, 5, car, pairs, 2,
                            obstacles, 2,
                            path, 256, &steps);
if (ret == 0) {
  // path[0..steps-1] 为小车依次要经过的坐标
} else {
  // 根据错误码处理
}
```

> 维护约定：后续如再修改该算法，请在本文件下方追加“当前修改”条目，并更新使用说明/示例（如接口或返回码变化）。 
