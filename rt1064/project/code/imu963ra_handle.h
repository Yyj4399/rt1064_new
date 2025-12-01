#ifndef __IMU963RA_HANDLE_H_
#define __IMU963RA_HANDLE_H_

#include "zf_common_typedef.h"



extern float yaw_angle;
extern float z_gyro;
extern float Z_gyro_final;
extern float dtimu;

typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
	  float X_magdata;
	  float Y_magdata;
	  float Z_magdata;
} gyro_param_t;

extern float yaw;
extern float yaw_add;

void ICM_hubu(void);
void gyroOffset_init(void);
double LowPassFilter_Silding(double dataNewest,double dataMiddle,double dataLast);
uint16 LowPassFilter_Average(uint16 data[],uint16 length);

#endif
