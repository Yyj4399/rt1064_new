#include "assigned_box_planner_greedy.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

#define MAX_BOX_COUNT 10
#define MAX_CELL_LIMIT 400
#define MAX_GREEDY_STEPS 2000
#define MAX_BOX_PATH_LEN (MAX_GREEDY_STEPS + 1)
#define LOOP_CHECK_SIZE 20
#define ADJ_PENALTY_WEIGHT 4
#define REVERSE_PENALTY 5
#define TURN_CHANGE_PENALTY 1
#define REPEAT_POS_PENALTY 15
#define DEADLOCK_PENALTY 1000

static Point box_raw_paths[MAX_BOX_COUNT][MAX_BOX_PATH_LEN];
static size_t box_raw_lens[MAX_BOX_COUNT];

typedef struct {
  Point opt_waypoints[MAX_BOX_PATH_LEN];
  Point min_turn_path[MAX_BOX_PATH_LEN];
  Point dedup[MAX_BOX_PATH_LEN];
  Point fast_path[MAX_BOX_PATH_LEN];
  Point detour_path[MAX_BOX_PATH_LEN];
} PlannerWorkspace;

static PlannerWorkspace g_ws;  // 复用大数组，避免反复占用栈

static int abs_i(int v) { return v >= 0 ? v : -v; }

static int manhattan(Point a, Point b) {
  return abs_i(a.row - b.row) + abs_i(a.col - b.col);
}

static int in_bounds(int rows, int cols, int row, int col) {
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

static int is_obstacle(const Point *obstacles, size_t count, int row, int col) {
  for (size_t i = 0; i < count; ++i) {
    if (obstacles[i].row == row && obstacles[i].col == col) {
      return 1;
    }
  }
  return 0;
}

static int is_box_at(const Point *boxes, size_t count, int row, int col,
                     size_t skip_idx) {
  for (size_t i = 0; i < count; ++i) {
    if (i == skip_idx) {
      continue;
    }
    if (boxes[i].row == row && boxes[i].col == col) {
      return 1;
    }
  }
  return 0;
}

static int count_adjacent_blockers(int rows, int cols, const Point *obstacles,
                                   size_t obstacle_count, const Point *boxes,
                                   size_t box_count, size_t skip_idx, int row,
                                   int col) {
  int count = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      int nr = row + dr;
      int nc = col + dc;
      if (!in_bounds(rows, cols, nr, nc)) {
        continue;
      }
      if (is_obstacle(obstacles, obstacle_count, nr, nc) ||
          is_box_at(boxes, box_count, nr, nc, skip_idx)) {
        count++;
      }
    }
  }
  return count;
}

// Second pass: check if the moving box can still reach its target after a push.
static int can_box_reach_target(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target) {
  int total_cells = rows * cols;
  if (total_cells > MAX_CELL_LIMIT || total_cells <= 0) {
    return 0;
  }

  if (start.row == target.row && start.col == target.col) {
    return 1;
  }

  uint8_t visited[MAX_CELL_LIMIT];
  int queue[MAX_CELL_LIMIT];
  int head = 0;
  int tail = 0;

  memset(visited, 0, sizeof(visited));

  int start_idx = start.row * cols + start.col;
  queue[tail++] = start_idx;
  visited[start_idx] = 1;

  while (head < tail) {
    if (head >= MAX_CELL_LIMIT) {
      break;
    }
    int curr_idx = queue[head++];
    int row = curr_idx / cols;
    int col = curr_idx % cols;

    if (row == target.row && col == target.col) {
      return 1;
    }

    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (int i = 0; i < 4; ++i) {
      int new_row = row + dirs[i][0];
      int new_col = col + dirs[i][1];
      int push_row = row - dirs[i][0];
      int push_col = col - dirs[i][1];

      if (!in_bounds(rows, cols, new_row, new_col) ||
          !in_bounds(rows, cols, push_row, push_col)) {
        continue;
      }

      if (is_obstacle(obstacles, obstacle_count, new_row, new_col) ||
          is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
        continue;
      }

      if (is_box_at(boxes, box_count, new_row, new_col, moving_idx) ||
          is_box_at(boxes, box_count, push_row, push_col, moving_idx)) {
        continue;
      }

      int idx = new_row * cols + new_col;
      if (!visited[idx]) {
        visited[idx] = 1;
        queue[tail++] = idx;
      }
    }
  }

  return 0;
}

// 简单的BFS寻找车辆到目标的路径，避免障碍与箱子
static int find_simple_path(int rows, int cols, Point start, Point target,
                            const Point *obstacles, size_t obstacle_count,
                            const Point *boxes, size_t box_count, Point *path,
                            size_t path_capacity, size_t *path_len) {
  int total_cells = rows * cols;
  if (total_cells > MAX_CELL_LIMIT || total_cells <= 0) {
    return 0;
  }

  if (!in_bounds(rows, cols, start.row, start.col) ||
      !in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }

  if (start.row == target.row && start.col == target.col) {
    *path_len = 0;
    return 1;
  }

  uint8_t visited[MAX_CELL_LIMIT];
  int parent[MAX_CELL_LIMIT];
  int queue[MAX_CELL_LIMIT];
  int head = 0;
  int tail = 0;

  memset(visited, 0, sizeof(visited));
  for (int i = 0; i < total_cells; ++i) {
    parent[i] = -1;
  }

  int start_idx = start.row * cols + start.col;
  int target_idx = target.row * cols + target.col;

  queue[tail++] = start_idx;
  visited[start_idx] = 1;

  int found = 0;

  while (head < tail) {
    if (head >= MAX_CELL_LIMIT) {
      break;
    }
    int curr_idx = queue[head++];
    if (curr_idx == target_idx) {
      found = 1;
      break;
    }

    int r = curr_idx / cols;
    int c = curr_idx % cols;

    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (int i = 0; i < 4; ++i) {
      int nr = r + dirs[i][0];
      int nc = c + dirs[i][1];

      if (in_bounds(rows, cols, nr, nc)) {
        int n_idx = nr * cols + nc;
        if (n_idx < total_cells && !visited[n_idx]) {
          if (!is_obstacle(obstacles, obstacle_count, nr, nc) &&
              !is_box_at(boxes, box_count, nr, nc, box_count)) {
            visited[n_idx] = 1;
            parent[n_idx] = curr_idx;
            if (tail < MAX_CELL_LIMIT) {
              queue[tail++] = n_idx;
            }
          }
        }
      }
    }
  }

  if (found) {
    Point temp_path[MAX_CELL_LIMIT];
    int steps = 0;
    int curr = target_idx;
    while (curr != start_idx && curr >= 0) {
      temp_path[steps].row = curr / cols;
      temp_path[steps].col = curr % cols;
      steps++;
      curr = parent[curr];
    }

    if (steps > (int)path_capacity) {
      return 0;
    }

    *path_len = (size_t)steps;
    for (int i = 0; i < steps; ++i) {
      path[i] = temp_path[steps - 1 - i];
    }
    return 1;
  }

  return 0;
}

// 判断以 a、b 为对角线的矩形内是否存在障碍物
static int rect_has_obstacle(int rows, int cols, const Point *obstacles,
                             size_t obstacle_count, const Point *boxes,
                             size_t box_count, size_t skip_idx, Point a,
                             Point b) {
  int rmin = a.row < b.row ? a.row : b.row;
  int rmax = a.row > b.row ? a.row : b.row;
  int cmin = a.col < b.col ? a.col : b.col;
  int cmax = a.col > b.col ? a.col : b.col;

  // 车辆需要绕行，矩形外扩一圈检测障碍
  rmin = rmin > 0 ? rmin - 1 : 0;
  cmin = cmin > 0 ? cmin - 1 : 0;
  rmax = (rmax + 1 < rows) ? rmax + 1 : rows - 1;
  cmax = (cmax + 1 < cols) ? cmax + 1 : cols - 1;

  for (size_t i = 0; i < obstacle_count; ++i) {
    int r = obstacles[i].row;
    int c = obstacles[i].col;
    if (r >= rmin && r <= rmax && c >= cmin && c <= cmax) {
      return 1;
    }
  }
  for (size_t i = 0; i < box_count; ++i) {
    if (i == skip_idx) {
      continue;
    }
    int r = boxes[i].row;
    int c = boxes[i].col;
    if (r >= rmin && r <= rmax && c >= cmin && c <= cmax) {
      return 1;
    }
  }
  return 0;
}

// 将箱子路径在矩形范围内尽量合并为单拐点（或直线）路径
static int optimize_box_path(const Point *raw, size_t raw_len, Point *out,
                             size_t out_cap, size_t *out_len, int rows,
                             int cols, const Point *obstacles,
                             size_t obstacle_count, const Point *boxes,
                             size_t box_count, size_t skip_idx) {
  if (!raw || !out || !out_len || raw_len == 0) {
    return 0;
  }

  size_t write_idx = 0;
  out[write_idx++] = raw[0];

  size_t i = 0;
  while (i + 1 < raw_len) {
    size_t best = i + 1;
    // 找到最远的 j，使得以 raw[i]、raw[j] 形成的矩形内无障碍
    for (size_t j = raw_len - 1; j > i; --j) {
      if (!rect_has_obstacle(rows, cols, obstacles, obstacle_count, boxes,
                             box_count, skip_idx, raw[i], raw[j])) {
        best = j;
        break;
      }
    }

    Point start = raw[i];
    Point end = raw[best];
    Point pivot_candidates[2] = {
        {start.row, end.col},
        {end.row, start.col},
    };

    // 选择第一个有效的转折点（可能与起点/终点重合意味着直线）
    Point pivot = pivot_candidates[0];
    if (!in_bounds(rows, cols, pivot.row, pivot.col)) {
      pivot = pivot_candidates[1];
    }

    if (write_idx >= out_cap) {
      return 0;
    }
    if (pivot.row != start.row || pivot.col != start.col) {
      if (pivot.row != end.row || pivot.col != end.col) {
        out[write_idx++] = pivot;
      }
    }

    if (write_idx >= out_cap) {
      return 0;
    }
    out[write_idx++] = end;
    i = best;
  }

  *out_len = write_idx;
  return 1;
}

// 将直角/直线路径展开为逐格路径，确保相邻点曼哈顿距离为 1
static int expand_orthogonal_path(const Point *in, size_t in_len, Point *out,
                                  size_t out_cap, size_t *out_len) {
  if (!in || !out || !out_len || in_len == 0) {
    return 0;
  }
  size_t write_idx = 0;
  out[write_idx++] = in[0];
  for (size_t i = 0; i + 1 < in_len; ++i) {
    Point a = in[i];
    Point b = in[i + 1];
    if (a.row != b.row && a.col != b.col) {
      return 0;  // 只接受直角或直线
    }
    int dr = (b.row > a.row) ? 1 : (b.row < a.row ? -1 : 0);
    int dc = (b.col > a.col) ? 1 : (b.col < a.col ? -1 : 0);
    int steps = abs_i(b.row - a.row) + abs_i(b.col - a.col);
    for (int s = 0; s < steps; ++s) {
      if (write_idx >= out_cap) {
        return 0;
      }
      Point nxt = {out[write_idx - 1].row + dr, out[write_idx - 1].col + dc};
      out[write_idx++] = nxt;
    }
  }
  *out_len = write_idx;
  return 1;
}

// 去除路径中的回环/重复节点，保留无回路的简化轨迹
static int simplify_box_path(const Point *in, size_t in_len, Point *out,
                             size_t out_cap, size_t *out_len) {
  if (!in || !out || !out_len || in_len == 0) {
    return 0;
  }
  size_t w = 0;
  for (size_t i = 0; i < in_len; ++i) {
    Point p = in[i];
    if (w > 0 && out[w - 1].row == p.row && out[w - 1].col == p.col) {
      continue;  // 连续重复
    }
    size_t loop_idx = SIZE_MAX;
    for (size_t j = 0; j < w; ++j) {
      if (out[j].row == p.row && out[j].col == p.col) {
        loop_idx = j;
        break;
      }
    }
    if (loop_idx != SIZE_MAX) {
      w = loop_idx + 1;  // 剪掉回环分支
    } else {
      if (w >= out_cap) {
        return 0;
      }
      out[w++] = p;
    }
  }
  if (w == 0) {
    return 0;
  }
  *out_len = w;
  return 1;
}

typedef struct {
  int turns;
  int steps;
} TurnCost;

static int validate_path(int rows, int cols, const Point *path, size_t len,
                         const Point *obstacles, size_t obstacle_count,
                         const Point *boxes, size_t box_count,
                         size_t skip_idx) {
  if (!path || len == 0) {
    return 0;
  }
  for (size_t i = 0; i < len; ++i) {
    int r = path[i].row;
    int c = path[i].col;
    if (!in_bounds(rows, cols, r, c)) {
      return 0;
    }
    if (is_obstacle(obstacles, obstacle_count, r, c) ||
        is_box_at(boxes, box_count, r, c, skip_idx)) {
      return 0;
    }
    if (i > 0) {
      int d = abs_i(path[i].row - path[i - 1].row) +
              abs_i(path[i].col - path[i - 1].col);
      if (d != 1) {
        return 0;
      }
    }
  }
  return 1;
}

static int count_turns(const Point *path, size_t len) {
  if (!path || len < 2) {
    return 0;
  }
  int turns = 0;
  int last_dr = path[1].row - path[0].row;
  int last_dc = path[1].col - path[0].col;
  for (size_t i = 2; i < len; ++i) {
    int dr = path[i].row - path[i - 1].row;
    int dc = path[i].col - path[i - 1].col;
    if (dr != last_dr || dc != last_dc) {
      turns++;
      last_dr = dr;
      last_dc = dc;
    }
  }
  return turns;
}

static int path_has_jump_or_diag(const Point *path, size_t len) {
  if (!path || len < 2) {
    return 0;
  }
  for (size_t i = 1; i < len; ++i) {
    int dr = abs_i(path[i].row - path[i - 1].row);
    int dc = abs_i(path[i].col - path[i - 1].col);
    if (dr + dc != 1) {
      return 1;
    }
  }
  return 0;
}

// 判断在当前步方向的左/右侧是否有障碍/箱子/边界，用于贴边绕行
static int has_side_blocker(int rows, int cols, const Point *obstacles,
                            size_t obstacle_count, const Point *boxes,
                            size_t box_count, size_t moving_idx, int r, int c,
                            int dr, int dc, int check_left) {
  int side_r = r + (check_left ? -dc : dc);
  int side_c = c + (check_left ? dr : -dr);
  if (!in_bounds(rows, cols, side_r, side_c)) {
    return 1;
  }
  if (is_obstacle(obstacles, obstacle_count, side_r, side_c) ||
      is_box_at(boxes, box_count, side_r, side_c, moving_idx)) {
    return 1;
  }
  return 0;
}

// 为箱子寻找可行推行路径（忽略小车，但要求推点可站人），用于绕行
static int find_box_detour_path(int rows, int cols, const Point *obstacles,
                                size_t obstacle_count, const Point *boxes,
                                size_t box_count, size_t moving_idx,
                                Point start, Point target, int prefer_left,
                                Point *out_path, size_t out_cap,
                                size_t *out_len) {
  int total_cells = rows * cols;
  if (total_cells > MAX_CELL_LIMIT || total_cells <= 0) {
    return 0;
  }
  if (!in_bounds(rows, cols, start.row, start.col) ||
      !in_bounds(rows, cols, target.row, target.col)) {
    return 0;
  }
  if (start.row == target.row && start.col == target.col) {
    if (out_cap < 1) {
      return 0;
    }
    out_path[0] = start;
    *out_len = 1;
    return 1;
  }

  uint8_t visited[MAX_CELL_LIMIT];
  int parent[MAX_CELL_LIMIT];
  int queue[MAX_CELL_LIMIT];
  int head = 0, tail = 0;
  memset(visited, 0, sizeof(visited));
  for (int i = 0; i < total_cells; ++i) parent[i] = -1;

  int start_idx = start.row * cols + start.col;
  int target_idx = target.row * cols + target.col;
  queue[tail++] = start_idx;
  visited[start_idx] = 1;

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  while (head < tail) {
    int idx = queue[head++];
    int r = idx / cols;
    int c = idx % cols;
    if (idx == target_idx) {
      break;
    }
    typedef struct {
      int d;
      int bias_good;
      int dist;
      int adj;
    } DirOrder;
    DirOrder ord[4];
    for (int k = 0; k < 4; ++k) {
      int nr = r + dirs[k][0];
      int nc = c + dirs[k][1];
      int bias_left = has_side_blocker(rows, cols, obstacles, obstacle_count,
                                       boxes, box_count, moving_idx, nr, nc,
                                       dirs[k][0], dirs[k][1], 1);
      int bias_right = has_side_blocker(rows, cols, obstacles, obstacle_count,
                                        boxes, box_count, moving_idx, nr, nc,
                                        dirs[k][0], dirs[k][1], 0);
      int bias = 0;
      if (prefer_left > 0) {
        bias = bias_left;
      } else if (prefer_left < 0) {
        bias = bias_right;
      }
      ord[k].d = k;
      ord[k].bias_good = bias;
      ord[k].dist = abs_i(target.row - nr) + abs_i(target.col - nc);
      ord[k].adj = count_adjacent_blockers(rows, cols, obstacles,
                                           obstacle_count, boxes, box_count,
                                           moving_idx, nr, nc);
    }
    for (int a = 0; a < 4; ++a) {
      for (int b = a + 1; b < 4; ++b) {
        int ai = ord[a].bias_good, bi = ord[b].bias_good;
        int swap = 0;
        if (bi > ai) {
          swap = 1;
        } else if (bi == ai) {
          if (ord[b].dist < ord[a].dist ||
              (ord[b].dist == ord[a].dist && ord[b].adj < ord[a].adj)) {
            swap = 1;
          }
        }
        if (swap) {
          DirOrder tmp = ord[a];
          ord[a] = ord[b];
          ord[b] = tmp;
        }
      }
    }

    for (int oi = 0; oi < 4; ++oi) {
      int d = ord[oi].d;
      int nr = r + dirs[d][0];
      int nc = c + dirs[d][1];
      int push_r = r - dirs[d][0];
      int push_c = c - dirs[d][1];
      if (!in_bounds(rows, cols, nr, nc) || !in_bounds(rows, cols, push_r, push_c)) {
        continue;
      }
      if (is_obstacle(obstacles, obstacle_count, nr, nc) ||
          is_obstacle(obstacles, obstacle_count, push_r, push_c)) {
        continue;
      }
      if (is_box_at(boxes, box_count, nr, nc, moving_idx) ||
          is_box_at(boxes, box_count, push_r, push_c, moving_idx)) {
        continue;
      }
      int nidx = nr * cols + nc;
      if (!visited[nidx]) {
        visited[nidx] = 1;
        parent[nidx] = idx;
        queue[tail++] = nidx;
      }
    }
  }

  if (!visited[target_idx]) {
    return 0;
  }

  Point rev[MAX_CELL_LIMIT];
  size_t rev_len = 0;
  int cur = target_idx;
  while (cur >= 0) {
    rev[rev_len].row = cur / cols;
    rev[rev_len].col = cur % cols;
    rev_len++;
    cur = parent[cur];
  }
  if (rev_len > out_cap) {
    return 0;
  }
  *out_len = rev_len;
  for (size_t i = 0; i < rev_len; ++i) {
    out_path[i] = rev[rev_len - 1 - i];
  }
  return 1;
}

// 在限定矩形内寻找转弯数最少的路径（次优为步数最短），考虑障碍和箱子
static int min_turn_path_rect_internal(
    int rows, int cols, const Point *obstacles, size_t obstacle_count,
    const Point *boxes, size_t box_count, size_t skip_idx, Point start,
    Point end, int rmin, int rmax, int cmin, int cmax, int max_turns,
    Point *out_path, size_t out_cap, size_t *out_len) {
  if (!in_bounds(rows, cols, start.row, start.col) ||
      !in_bounds(rows, cols, end.row, end.col)) {
    return 0;
  }
  int total_cells = rows * cols;
  if (total_cells > MAX_CELL_LIMIT) {
    return 0;
  }

  const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  TurnCost cost[4][MAX_CELL_LIMIT];
  int parent[4][MAX_CELL_LIMIT];
  for (int d = 0; d < 4; ++d) {
    for (int i = 0; i < total_cells; ++i) {
      cost[d][i].turns = INT_MAX;
      cost[d][i].steps = INT_MAX;
      parent[d][i] = -1;
    }
  }

  int queue[MAX_CELL_LIMIT * 4];
  int qh = 0, qt = 0;

  int start_idx = start.row * cols + start.col;
  for (int d = 0; d < 4; ++d) {
    cost[d][start_idx].turns = 0;
    cost[d][start_idx].steps = 0;
    queue[qt++] = start_idx * 4 + d;
  }

  while (qh < qt) {
    int node = queue[qh++];
    int idx = node / 4;
    int dir = node % 4;
    int r = idx / cols;
    int c = idx % cols;
    TurnCost curr = cost[dir][idx];

    for (int ndir = 0; ndir < 4; ++ndir) {
      int nr = r + dirs[ndir][0];
      int nc = c + dirs[ndir][1];
      if (nr < rmin || nr > rmax || nc < cmin || nc > cmax) {
        continue;
      }
      if (!in_bounds(rows, cols, nr, nc)) {
        continue;
      }
      if (is_obstacle(obstacles, obstacle_count, nr, nc) ||
          is_box_at(boxes, box_count, nr, nc, skip_idx)) {
        continue;
      }
      int nidx = nr * cols + nc;
      TurnCost nxt = curr;
      nxt.steps += 1;
      if (dir != ndir) {
        nxt.turns += 1;
      }
      if (max_turns >= 0 && nxt.turns > max_turns) {
        continue;
      }
      TurnCost old = cost[ndir][nidx];
      int better = 0;
      if (nxt.turns < old.turns) {
        better = 1;
      } else if (nxt.turns == old.turns && nxt.steps < old.steps) {
        better = 1;
      }
      if (better) {
        cost[ndir][nidx] = nxt;
        parent[ndir][nidx] = idx * 4 + dir;
        queue[qt++] = nidx * 4 + ndir;
      }
    }
  }

  int end_idx = end.row * cols + end.col;
  int best_dir = -1;
  TurnCost best = {INT_MAX, INT_MAX};
  for (int d = 0; d < 4; ++d) {
    TurnCost cand = cost[d][end_idx];
    if (cand.turns < best.turns ||
        (cand.turns == best.turns && cand.steps < best.steps)) {
      best = cand;
      best_dir = d;
    }
  }
  if (best_dir < 0 || best.steps == INT_MAX) {
    return 0;
  }

  Point rev_path[MAX_CELL_LIMIT];
  size_t rev_len = 0;
  int cur_node = end_idx * 4 + best_dir;
  while (cur_node >= 0) {
    int idx = cur_node / 4;
    Point p = {idx / cols, idx % cols};
    rev_path[rev_len++] = p;
    int pd = cur_node % 4;
    int par = parent[pd][idx];
    if (par == -1) {
      break;
    }
    cur_node = par;
  }

  if (rev_len > out_cap) {
    return 0;
  }
  *out_len = rev_len;
  for (size_t i = 0; i < rev_len; ++i) {
    out_path[i] = rev_path[rev_len - 1 - i];
  }
  return 1;
}

static int min_turn_path_rect(int rows, int cols, const Point *obstacles,
                              size_t obstacle_count, const Point *boxes,
                              size_t box_count, size_t skip_idx, Point start,
                              Point end, int rmin, int rmax, int cmin, int cmax,
                              Point *out_path, size_t out_cap,
                              size_t *out_len) {
  return min_turn_path_rect_internal(rows, cols, obstacles, obstacle_count,
                                     boxes, box_count, skip_idx, start, end,
                                     rmin, rmax, cmin, cmax, -1, out_path,
                                     out_cap, out_len);
}

// 在矩形上/下半区尝试最小拐点路径，不通则切换半区，均失败则拐点阈值+1继续
static int try_half_min_turn(int rows, int cols, const Point *obstacles,
                             size_t obstacle_count, const Point *boxes,
                             size_t box_count, size_t skip_idx, Point start,
                             Point end, int rmin, int rmax, int cmin, int cmax,
                             int base_turns, Point *out_path, size_t out_cap,
                             size_t *out_len) {
  if (rmin > rmax) {
    int tmp = rmin;
    rmin = rmax;
    rmax = tmp;
  }
  if (cmin > cmax) {
    int tmp = cmin;
    cmin = cmax;
    cmax = tmp;
  }
  int mid = rmin + (rmax - rmin) / 2;
  int turn_limit_start = base_turns >= 0 ? base_turns : 0;
  int turn_limit_end = turn_limit_start + rows + cols;
  if (turn_limit_end < turn_limit_start) {
    turn_limit_end = turn_limit_start;
  }

  for (int max_turns = turn_limit_start; max_turns <= turn_limit_end;
       ++max_turns) {
    for (int half = 0; half < 2; ++half) {
      int hrmin = (half == 0) ? rmin : mid;
      int hrmax = (half == 0) ? mid : rmax;
      if (hrmin > hrmax) {
        int tmp = hrmin;
        hrmin = hrmax;
        hrmax = tmp;
      }
      // 半区外扩一圈检测/尝试
      hrmin = hrmin > 0 ? hrmin - 1 : hrmin;
      hrmax = (hrmax + 1 < rows) ? hrmax + 1 : rows - 1;
      int hcmin = cmin > 0 ? cmin - 1 : cmin;
      int hcmax = (cmax + 1 < cols) ? cmax + 1 : cols - 1;
      if (start.row < hrmin || start.row > hrmax || end.row < hrmin ||
          end.row > hrmax || start.col < hcmin || start.col > hcmax ||
          end.col < hcmin || end.col > hcmax) {
        continue;
      }

      size_t cand_len = 0;
      if (min_turn_path_rect_internal(rows, cols, obstacles, obstacle_count,
                                      boxes, box_count, skip_idx, start, end,
                                      hrmin, hrmax, hcmin, hcmax, max_turns,
                                      out_path, out_cap, &cand_len)) {
        *out_len = cand_len;
        return 1;
      }
    }
  }

  return 0;
}

// 构建小车从 A 到 B 的路径，优先半区重试，不通再回退
static int build_car_segment_with_half_retry(
    int rows, int cols, Point start, Point target, const Point *obstacles,
    size_t obstacle_count, const Point *boxes, size_t box_count,
    size_t skip_idx, Point *out, size_t out_cap, size_t *out_len) {
  if (start.row == target.row && start.col == target.col) {
    *out_len = 0;
    return 1;
  }

  Point base_path[256];
  size_t base_len = 0;
  if (!find_simple_path(rows, cols, start, target, obstacles, obstacle_count,
                        boxes, box_count, base_path, 256, &base_len)) {
    return 0;
  }

  Point dedup[256];
  size_t dedup_len = 0;
  if (!simplify_box_path(base_path, base_len, dedup, 256, &dedup_len) ||
      dedup_len == 0) {
    return 0;
  }

  int rmin = rows, rmax = 0, cmin = cols, cmax = 0;
  for (size_t t = 0; t < dedup_len; ++t) {
    if (dedup[t].row < rmin) rmin = dedup[t].row;
    if (dedup[t].row > rmax) rmax = dedup[t].row;
    if (dedup[t].col < cmin) cmin = dedup[t].col;
    if (dedup[t].col > cmax) cmax = dedup[t].col;
  }

  Point candidate[256];
  size_t candidate_len = 0;
  int base_turns = count_turns(dedup, dedup_len);
  int ok = try_half_min_turn(rows, cols, obstacles, obstacle_count, boxes,
                             box_count, skip_idx, dedup[0],
                             dedup[dedup_len - 1], rmin, rmax, cmin, cmax,
                             base_turns, candidate, 256, &candidate_len);
  if (ok &&
      !validate_path(rows, cols, candidate, candidate_len, obstacles,
                     obstacle_count, boxes, box_count, skip_idx)) {
    ok = 0;
  }

  if (!ok) {
    ok = min_turn_path_rect(rows, cols, obstacles, obstacle_count, boxes,
                            box_count, skip_idx, dedup[0],
                            dedup[dedup_len - 1], rmin, rmax, cmin, cmax,
                            candidate, 256, &candidate_len);
    if (!ok) {
      ok = min_turn_path_rect(rows, cols, obstacles, obstacle_count, boxes,
                              box_count, skip_idx, dedup[0],
                              dedup[dedup_len - 1], 0, rows - 1, 0, cols - 1,
                              candidate, 256, &candidate_len);
    }
    if (ok &&
        !validate_path(rows, cols, candidate, candidate_len, obstacles,
                       obstacle_count, boxes, box_count, skip_idx)) {
      ok = 0;
    }
  }

  if (!ok) {
    if (dedup_len > out_cap) {
      return 0;
    }
    for (size_t i = 0; i < dedup_len; ++i) {
      out[i] = dedup[i];
    }
    *out_len = dedup_len;
    return 1;
  }

  if (candidate_len > out_cap) {
    return 0;
  }
  for (size_t i = 0; i < candidate_len; ++i) {
    out[i] = candidate[i];
  }
  *out_len = candidate_len;
  return 1;
}

// 按给定的箱子路径模拟推箱并写入小车路径
static int simulate_box_path(int rows, int cols, Point *car_pos,
                             Point *boxes, size_t box_count,
                             size_t moving_idx, const Point *obstacles,
                             size_t obstacle_count, const Point *box_path,
                             size_t box_path_len, Point target,
                             Point *path_buffer, size_t path_capacity,
                             size_t *out_steps) {
  if (box_path_len < 2) {
    return 1;
  }

  size_t start_steps = *out_steps;
  Point saved_boxes[MAX_BOX_COUNT];
  for (size_t i = 0; i < box_count; ++i) {
    saved_boxes[i] = boxes[i];
  }
  Point saved_car = *car_pos;
  int fail_code = 0;

  for (size_t i = 0; i + 1 < box_path_len; ++i) {
    Point curr = box_path[i];
    Point next = box_path[i + 1];
    int dr = next.row - curr.row;
    int dc = next.col - curr.col;
    if (abs_i(dr) + abs_i(dc) != 1) {
      return 0;
    }

    int push_r = curr.row - dr;
    int push_c = curr.col - dc;
    if (!in_bounds(rows, cols, push_r, push_c)) {
      return 0;
    }
    if (is_obstacle(obstacles, obstacle_count, next.row, next.col) ||
        is_obstacle(obstacles, obstacle_count, push_r, push_c)) {
      return 0;
    }
    if (is_box_at(boxes, box_count, next.row, next.col, moving_idx) ||
        is_box_at(boxes, box_count, push_r, push_c, moving_idx)) {
      return 0;
    }

    Point push_from = {push_r, push_c};
    Point car_seg[256];
    size_t car_seg_len = 0;
    if (!build_car_segment_with_half_retry(rows, cols, *car_pos, push_from,
                                           obstacles, obstacle_count, boxes,
                                           box_count, box_count, car_seg, 256,
                                           &car_seg_len)) {
      fail_code = 0;
      goto FAIL;
    }

    if (*out_steps + car_seg_len + 1 > path_capacity) {
      fail_code = -7;
      goto FAIL;
    }
    for (size_t p = 0; p < car_seg_len; ++p) {
      path_buffer[(*out_steps)++] = car_seg[p];
    }

    *car_pos = push_from;
    boxes[moving_idx] = next;
    if (!can_box_reach_target(rows, cols, obstacles, obstacle_count, boxes,
                              box_count, moving_idx, next, target)) {
      fail_code = 0;
      goto FAIL;
    }
    *car_pos = curr;
    path_buffer[(*out_steps)++] = *car_pos;
  }
  return 1;

FAIL:
  for (size_t i = 0; i < box_count; ++i) {
    boxes[i] = saved_boxes[i];
  }
  *car_pos = saved_car;
  *out_steps = start_steps;
  return fail_code ? fail_code : 0;
}

// 基于优化后的箱子路径重新生成小车路径
static int rebuild_car_path(
    int rows, int cols, Point car, const BoxTargetPair *pairs,
    size_t pair_count, const Point *obstacles, size_t obstacle_count,
    const size_t *box_order_seq, const Point box_paths[][MAX_BOX_PATH_LEN],
    const size_t *box_path_lens, Point *path_buffer, size_t path_capacity,
    size_t *out_steps) {
  Point sim_boxes[MAX_BOX_COUNT];
  for (size_t i = 0; i < pair_count; ++i) {
    sim_boxes[i] = pairs[i].box;
  }

  Point car_pos = car;
  *out_steps = 0;

  size_t dedup_len = 0;

  for (size_t oi = 0; oi < pair_count; ++oi) {
    size_t idx = box_order_seq[oi];
    const Point *raw_path_full = box_paths[idx];
    size_t raw_len_full = box_path_lens[idx];

    if (!simplify_box_path(raw_path_full, raw_len_full, g_ws.dedup,
                           MAX_BOX_PATH_LEN,
                           &dedup_len)) {
      return -6;
    }

    int rmin = rows, rmax = 0, cmin = cols, cmax = 0;
    for (size_t t = 0; t < dedup_len; ++t) {
      if (g_ws.dedup[t].row < rmin) rmin = g_ws.dedup[t].row;
      if (g_ws.dedup[t].row > rmax) rmax = g_ws.dedup[t].row;
      if (g_ws.dedup[t].col < cmin) cmin = g_ws.dedup[t].col;
      if (g_ws.dedup[t].col > cmax) cmax = g_ws.dedup[t].col;
    }

    size_t opt_len = 0;
    size_t best_len = 0;
    int have_best = 0;
    int base_turns = count_turns(g_ws.dedup, dedup_len);
    int half_ok = try_half_min_turn(
        rows, cols, obstacles, obstacle_count, sim_boxes, pair_count, idx,
        g_ws.dedup[0], g_ws.dedup[dedup_len - 1], rmin, rmax, cmin, cmax,
        base_turns, g_ws.min_turn_path, MAX_BOX_PATH_LEN, &opt_len);
    if (half_ok &&
        validate_path(rows, cols, g_ws.min_turn_path, opt_len, obstacles,
                      obstacle_count, sim_boxes, pair_count, idx)) {
      have_best = 1;
      best_len = opt_len;
    }

    int opt_ok = optimize_box_path(g_ws.dedup, dedup_len, g_ws.opt_waypoints,
                                   MAX_BOX_PATH_LEN, &opt_len, rows, cols,
                                   obstacles, obstacle_count, sim_boxes,
                                   pair_count, idx);
    if (!have_best && opt_ok) {
      opt_ok = min_turn_path_rect(rows, cols, obstacles, obstacle_count,
                                  sim_boxes, pair_count, idx,
                                  g_ws.opt_waypoints[0],
                                  g_ws.opt_waypoints[opt_len - 1], rmin, rmax,
                                  cmin, cmax, g_ws.min_turn_path,
                                  MAX_BOX_PATH_LEN, &opt_len);
      if (!opt_ok) {
        opt_ok = min_turn_path_rect(rows, cols, obstacles, obstacle_count,
                                    sim_boxes, pair_count, idx,
                                    g_ws.opt_waypoints[0],
                                    g_ws.opt_waypoints[opt_len - 1], 0,
                                    rows - 1, 0, cols - 1, g_ws.min_turn_path,
                                    MAX_BOX_PATH_LEN, &opt_len);
      }
      if (opt_ok &&
          !validate_path(rows, cols, g_ws.min_turn_path, opt_len, obstacles,
                         obstacle_count, sim_boxes, pair_count, idx)) {
        opt_ok = 0;
      }
      if (opt_ok) {
        have_best = 1;
        best_len = opt_len;
      }
    }

    const Point *candidate_path = g_ws.dedup;
    size_t candidate_len = dedup_len;

    if (have_best) {
      candidate_path = g_ws.min_turn_path;
      candidate_len = best_len;
    }

    Point saved_boxes[MAX_BOX_COUNT];
    for (size_t k = 0; k < pair_count; ++k) {
      saved_boxes[k] = sim_boxes[k];
    }
    Point saved_car = car_pos;
    size_t seg_start = *out_steps;

    int sim_res = simulate_box_path(rows, cols, &car_pos, sim_boxes,
                                    pair_count, idx, obstacles, obstacle_count,
                                    candidate_path, candidate_len,
                                    pairs[idx].target, path_buffer,
                                    path_capacity, out_steps);
    int need_half_retry =
        sim_res && path_has_jump_or_diag(path_buffer + seg_start,
                                         *out_steps - seg_start);
    if (sim_res == -7) {
      return -7;
    }
    if (!sim_res || need_half_retry) {
      for (size_t k = 0; k < pair_count; ++k) {
        sim_boxes[k] = saved_boxes[k];
      }
      car_pos = saved_car;
      *out_steps = seg_start;
      // 回退到原始箱子路径并再次触发半区搜索
      sim_res = simulate_box_path(rows, cols, &car_pos, sim_boxes, pair_count,
                                  idx, obstacles, obstacle_count, g_ws.dedup,
                                  dedup_len, pairs[idx].target, path_buffer,
                                  path_capacity, out_steps);
      if (sim_res == -7) {
        return -7;
      }
      if (!sim_res ||
          path_has_jump_or_diag(path_buffer + seg_start,
                                *out_steps - seg_start)) {
        return -6;
      }
    }
  }

  return 0;
}

int plan_boxes_greedy(int rows, int cols, Point car, const BoxTargetPair *pairs,
                      size_t pair_count, const Point *obstacles,
                      size_t obstacle_count, Point *path_buffer,
                      size_t path_capacity, size_t *out_steps) {
  if (!pairs || !path_buffer || !out_steps) {
    return -1;
  }
  if (pair_count == 0) {
    return -2;
  }
  if (pair_count > MAX_BOX_COUNT) {
    return -3;
  }
  if (path_capacity == 0) {
    return -4;
  }

  *out_steps = 0;
  size_t greedy_steps = 0;

  Point current_car = car;
  Point current_boxes[MAX_BOX_COUNT];

  for (size_t i = 0; i < pair_count; ++i) {
    current_boxes[i] = pairs[i].box;
    box_raw_lens[i] = 0;
  }

  int picked[MAX_BOX_COUNT] = {0};
  size_t order_seq[MAX_BOX_COUNT] = {0};

  for (size_t order_idx = 0; order_idx < pair_count; ++order_idx) {
    size_t box_idx = SIZE_MAX;
    int best_score = INT_MAX;
    for (size_t i = 0; i < pair_count; ++i) {
      if (picked[i]) {
        continue;
      }
      int dist_car = manhattan(current_car, current_boxes[i]);
      int dist_goal = manhattan(current_boxes[i], pairs[i].target);
      int score = dist_car + dist_goal;  // �򵥹��ܺ���，��СС��ǰ���ܲ���
      if (score < best_score) {
        best_score = score;
        box_idx = i;
      }
    }
    if (box_idx == SIZE_MAX) {
      return -6;
    }
    picked[box_idx] = 1;
    order_seq[order_idx] = box_idx;
    Point box = current_boxes[box_idx];
    Point target = pairs[box_idx].target;

    if (box_raw_lens[box_idx] < MAX_BOX_PATH_LEN) {
      box_raw_paths[box_idx][box_raw_lens[box_idx]++] = box;
    }

    if (box.row == target.row && box.col == target.col) {
      continue;
    }

    Point recent_positions[LOOP_CHECK_SIZE];
    for (int i = 0; i < LOOP_CHECK_SIZE; ++i) {
      recent_positions[i].row = -1;
      recent_positions[i].col = -1;
    }
    int pos_idx = 0;
    int step_count = 0;
    int last_dr = 0;
    int last_dc = 0;
    int oscillate_hits = 0;
    int detour_attempts = 0;
    int solved_fast = 0;
    
    // ???????? BFS ?????????
    Point fast_path[MAX_BOX_PATH_LEN];
    size_t fast_len = 0;
    if (find_box_detour_path(rows, cols, obstacles, obstacle_count,
                             current_boxes, pair_count, box_idx, box, target,
                             0, fast_path, MAX_BOX_PATH_LEN, &fast_len) &&
        fast_len >= 2) {
      for (size_t di = 1;
           di < fast_len && box_raw_lens[box_idx] < MAX_BOX_PATH_LEN; ++di) {
        box_raw_paths[box_idx][box_raw_lens[box_idx]++] = fast_path[di];
      }
      size_t tmp_steps = 0;
      int sim_res =
          simulate_box_path(rows, cols, &current_car, current_boxes,
                            pair_count, box_idx, obstacles, obstacle_count,
                            fast_path, fast_len, target, path_buffer,
                            path_capacity, &tmp_steps);
      if (sim_res == -7) {
        return -7;
      }
      if (sim_res) {
        box = current_boxes[box_idx];
        last_dr = 0;
        last_dc = 0;
        solved_fast = 1;
      } else {
        box_raw_lens[box_idx] = 1;
      }
    }

    // �ӷ���һ��ֹͣ，ֱ���Դ�����·�����·���Ի�ת��
    while (!solved_fast && (box.row != target.row || box.col != target.col)) {
      if (step_count++ >= MAX_GREEDY_STEPS) {
        return -5;
      }

      recent_positions[pos_idx % LOOP_CHECK_SIZE] = box;
      pos_idx++;

      int curr_repeat = 0;
      for (int rp = 0; rp < LOOP_CHECK_SIZE; ++rp) {
        if (recent_positions[rp].row == box.row &&
            recent_positions[rp].col == box.col) {
          curr_repeat++;
        }
      }
      if (curr_repeat >= 3) {
        oscillate_hits++;
      } else if (oscillate_hits > 0) {
        oscillate_hits--;
      }

      if (oscillate_hits >= 3 && detour_attempts < 3) {
        detour_attempts++;
        Point detour_path[MAX_BOX_PATH_LEN];
        size_t detour_len = 0;
        int detour_ok = find_box_detour_path(
            rows, cols, obstacles, obstacle_count, current_boxes, pair_count,
            box_idx, box, target, 1, detour_path, MAX_BOX_PATH_LEN,
            &detour_len);
        if (!detour_ok || detour_len < 2) {
          detour_ok = find_box_detour_path(
              rows, cols, obstacles, obstacle_count, current_boxes, pair_count,
              box_idx, box, target, -1, detour_path, MAX_BOX_PATH_LEN,
              &detour_len);
        }
        if (!detour_ok || detour_len < 2) {
          detour_ok = find_box_detour_path(
              rows, cols, obstacles, obstacle_count, current_boxes, pair_count,
              box_idx, box, target, 0, detour_path, MAX_BOX_PATH_LEN,
              &detour_len);
        }
        if (detour_ok && detour_len >= 2) {
          for (size_t di = 1;
               di < detour_len && box_raw_lens[box_idx] < MAX_BOX_PATH_LEN;
               ++di) {
            box_raw_paths[box_idx][box_raw_lens[box_idx]++] = detour_path[di];
          }
          size_t tmp_steps = 0;
          int sim_res =
              simulate_box_path(rows, cols, &current_car, current_boxes,
                                pair_count, box_idx, obstacles,
                                obstacle_count, detour_path, detour_len,
                                target, path_buffer, path_capacity, &tmp_steps);
          if (sim_res == -7) {
            return -7;
          }
          if (sim_res) {
            box = current_boxes[box_idx];
            last_dr = 0;
            last_dc = 0;
            continue;
          }
        }
      }

      typedef struct {
        int dr;
        int dc;
        int dist;
        int adj_pen;
        int reverse_pen;
        int repeat_pen;
        int reach_pen;
        int car_cost;   // �˴����¼С���������Ҫ�Ĳ��ͻ���ת���
        int car_turns;  // �˴����¼С�����ͻ��������ת��
        int score;
        int feasible;
        Point push_from;
        Point box_next;
        Point path[256];
        size_t path_len;
      } DirCandidate;

      DirCandidate candidates[4];
      const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

      for (int i = 0; i < 4; ++i) {
        candidates[i].dr = dirs[i][0];
        candidates[i].dc = dirs[i][1];
        candidates[i].feasible = 0;
        candidates[i].score = INT_MAX;
        candidates[i].path_len = 0;
        candidates[i].reach_pen = DEADLOCK_PENALTY;

        int new_box_row = box.row + dirs[i][0];
        int new_box_col = box.col + dirs[i][1];
        int push_row = box.row - dirs[i][0];
        int push_col = box.col - dirs[i][1];

        if (!in_bounds(rows, cols, new_box_row, new_box_col) ||
            !in_bounds(rows, cols, push_row, push_col)) {
          continue;
        }

        if (is_obstacle(obstacles, obstacle_count, new_box_row, new_box_col) ||
            is_obstacle(obstacles, obstacle_count, push_row, push_col)) {
          continue;
        }

        if (is_box_at(current_boxes, pair_count, new_box_row, new_box_col,
                      box_idx) ||
            is_box_at(current_boxes, pair_count, push_row, push_col,
                      box_idx)) {
          continue;
        }

        Point push_from = {push_row, push_col};
        Point temp_path[256];
        size_t temp_len = 0;

        if (!build_car_segment_with_half_retry(rows, cols, current_car,
                                               push_from, obstacles,
                                               obstacle_count, current_boxes,
                                               pair_count, pair_count,
                                               temp_path, 256, &temp_len)) {
          continue;
        }

        int dist_after = manhattan((Point){new_box_row, new_box_col}, target);
        int adj_pen = count_adjacent_blockers(
            rows, cols, obstacles, obstacle_count, current_boxes, pair_count,
            box_idx, new_box_row, new_box_col);
        adj_pen += count_adjacent_blockers(rows, cols, obstacles,
                                           obstacle_count, current_boxes,
                                           pair_count, box_idx, push_row,
                                           push_col);

        int reverse_pen = (last_dr == -dirs[i][0] && last_dc == -dirs[i][1])
                              ? REVERSE_PENALTY
                              : 0;
        int car_turns = (temp_len > 1) ? count_turns(temp_path, temp_len) : 0;
        int car_cost = (int)temp_len + car_turns;  // ????+????
        int turn_change_pen = (last_dr || last_dc) && (last_dr != dirs[i][0] || last_dc != dirs[i][1]) ? TURN_CHANGE_PENALTY : 0;

        int repeat_count = 0;
        for (int rp = 0; rp < LOOP_CHECK_SIZE; ++rp) {
          if (recent_positions[rp].row == new_box_row &&
              recent_positions[rp].col == new_box_col) {
            repeat_count++;
          }
        }
        int repeat_pen = repeat_count * REPEAT_POS_PENALTY;

        int reach_pen = can_box_reach_target(
            rows, cols, obstacles, obstacle_count, current_boxes, pair_count,
            box_idx, (Point){new_box_row, new_box_col}, target)
                            ? 0
                            : DEADLOCK_PENALTY;

        candidates[i].reach_pen = reach_pen;
        if (reach_pen > 0) {
          continue;
        }

        int score = dist_after + ADJ_PENALTY_WEIGHT * adj_pen + reverse_pen +
                    repeat_pen + car_cost + turn_change_pen;
        candidates[i].dist = dist_after;
        candidates[i].adj_pen = adj_pen;
        candidates[i].reverse_pen = reverse_pen;
        candidates[i].repeat_pen = repeat_pen;
        candidates[i].car_cost = car_cost;
        candidates[i].car_turns = car_turns;
        candidates[i].score = score;
        candidates[i].feasible = 1;
        candidates[i].push_from = push_from;
        candidates[i].box_next.row = new_box_row;
        candidates[i].box_next.col = new_box_col;
        candidates[i].path_len = temp_len;
        for (size_t pi = 0; pi < temp_len; ++pi) {
          candidates[i].path[pi] = temp_path[pi];
        }
      }

      int best_idx = -1;
      int best_score = INT_MAX;
      int best_car_cost = INT_MAX;
      int best_dist = INT_MAX;
      for (int i = 0; i < 4; ++i) {
        if (!candidates[i].feasible) {
          continue;
        }
        if (candidates[i].dist < best_dist ||
            (candidates[i].dist == best_dist &&
             (candidates[i].score < best_score ||
              (candidates[i].score == best_score &&
               candidates[i].car_cost < best_car_cost)))) {
          best_car_cost = candidates[i].car_cost;
          best_dist = candidates[i].dist;
          best_score = candidates[i].score;
          best_idx = i;
        }
      }

      if (best_idx < 0) {
        // 贪心方向无路，尝试为箱子规划绕行路径
        Point detour_path[MAX_BOX_PATH_LEN];
        size_t detour_len = 0;
        int detour_ok = find_box_detour_path(rows, cols, obstacles,
                                             obstacle_count, current_boxes,
                                             pair_count, box_idx, box, target,
                                             1, detour_path,
                                             MAX_BOX_PATH_LEN, &detour_len);
        if (!detour_ok || detour_len < 2) {
          detour_ok = find_box_detour_path(rows, cols, obstacles,
                                           obstacle_count, current_boxes,
                                           pair_count, box_idx, box, target,
                                           -1, detour_path,
                                           MAX_BOX_PATH_LEN, &detour_len);
        }
        if (!detour_ok || detour_len < 2) {
          detour_ok = find_box_detour_path(rows, cols, obstacles,
                                           obstacle_count, current_boxes,
                                           pair_count, box_idx, box, target, 0,
                                           detour_path, MAX_BOX_PATH_LEN,
                                           &detour_len);
        }
        if (!detour_ok || detour_len < 2) {
          return -6;
        }

        // 记录绕行箱子路径
        for (size_t di = 1; di < detour_len && box_raw_lens[box_idx] < MAX_BOX_PATH_LEN;
             ++di) {
          box_raw_paths[box_idx][box_raw_lens[box_idx]++] = detour_path[di];
        }

        // 模拟推箱绕行，更新小车与箱子位置
        size_t tmp_steps = 0;
        int sim_res = simulate_box_path(rows, cols, &current_car, current_boxes,
                                        pair_count, box_idx, obstacles,
                                        obstacle_count, detour_path, detour_len,
                                        target, path_buffer, path_capacity,
                                        &tmp_steps);
        if (sim_res == -7) {
          return -7;
        }
        if (!sim_res) {
          return -6;
        }
        box = current_boxes[box_idx];
        break;
      }

      DirCandidate chosen = candidates[best_idx];

      for (size_t i = 0; i < chosen.path_len; ++i) {
        if (greedy_steps < path_capacity) {
          path_buffer[greedy_steps] = chosen.path[i];
        }
        greedy_steps++;
      }
      current_car = chosen.push_from;

      Point box_old_pos = box;
      box = chosen.box_next;
      current_boxes[box_idx] = box;

      if (box_raw_lens[box_idx] < MAX_BOX_PATH_LEN) {
        box_raw_paths[box_idx][box_raw_lens[box_idx]++] = box;
      }

      if (box_old_pos.row != chosen.push_from.row ||
          box_old_pos.col != chosen.push_from.col) {
        current_car = box_old_pos;
        if (greedy_steps < path_capacity) {
          path_buffer[greedy_steps] = current_car;
        }
        greedy_steps++;
      }

      last_dr = chosen.dr;
      last_dc = chosen.dc;
    }
  }

  int rebuild_res = rebuild_car_path(rows, cols, car, pairs, pair_count,
                                     obstacles, obstacle_count, order_seq,
                                     box_raw_paths, box_raw_lens, path_buffer,
                                     path_capacity, out_steps);
  if (rebuild_res != 0) {
    return rebuild_res;
  }

  return 0;
}
