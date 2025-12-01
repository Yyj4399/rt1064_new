# IMU963RA 姿态移植说明

## 主要改动
- `rt1064/project/code/Attitude.c`：改为使用 imu963ra 的陀螺仪/加速度计数据，增加静置 500 次的零偏标定（阈值 5 dps，约 1 秒），EKF 更新周期设为 5 ms，并输出 Euler 角与 IMU 原始向量。
- `rt1064/project/code/Attitude.h`：新增 `attitude_euler_t`、`imu963ra_data_t` 结构体并导出全局 `eulerAngle`、`icm_data`，便于其他模块读取姿态与 IMU 数据。
- `rt1064/project/code/EKF_Platform.h`：移除缺失的 Ifx LUT 依赖，改用标准 `cosf/atan2f` 和本地 matrix 辅助接口。
- `rt1064/project/code/QuaternionEKF.c`：补充 `<string.h>` 以确保 memcpy/memset 声明。
- `rt1064/project/code/matrix.c`：补充 `<math.h>` 以提供 `fabsf` 声明。

## 在 `main` 中的使用示例
```c
#include "Attitude.h"
#include "zf_device_imu963ra.h"

int main(void)
{
    system_init();
    imu963ra_init();    // 先初始化 IMU
    Attitude_Init();    // 静置校准陀螺零偏

    while (1)
    {
        Attitude_Calculate();   // 与 ATTITUDE_UPDATE_PERIOD_S (5 ms) 对齐周期调用
        printf("roll=%.2f pitch=%.2f yaw=%.2f\r\n",
               eulerAngle.roll, eulerAngle.pitch, eulerAngle.yaw);
        system_delay_ms(5);
    }
}
```
- 如果主循环周期不同，请在 `Attitude.c` 中调整 `ATTITUDE_UPDATE_PERIOD_S` 以匹配实际周期。
- 零偏标定在板子静止时完成，必要时可重新调用 `Attitude_Init` 以重新标定。

## 调参提示
- `IMU_QuaternionEKF_Init` 参数未改（Q1=100，Q2=1e-5，R=1e8，lambda=0.9996）。减小 `R` 会更信任加速度计，增大 `R` 会更依赖陀螺积分。
- 量程缩放仍由 `zf_device_imu963ra.h` 中的 `IMU963RA_*_SAMPLE_DEFAULT` 宏决定，如需更改传感器量程请同步修改。
- 现已无外部 Ifx LUT 依赖，使用标准数学库和本地矩阵实现即可。
