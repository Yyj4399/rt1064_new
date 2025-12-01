#include "imu963ra_handle.h"
#include "math.h"
#include "zf_device_imu963ra.h"
#include "zf_driver_delay.h"

bool GyroOffset_init = 0;

float yaw;
float yaw_angle=0;//车身为单位的角度旋转
float dtimu = 0.005;//采样时间
float z_gyro=0;
float z_gyro_less=0;
float z_gyro_last=0;

float Z_gyro_final=0;
gyro_param_t GyroOffset;

/*陀螺仪零飘初始化*/
void gyroOffset_init(void)      
{
	uint16_t i;
    GyroOffset.Zdata = 0;
    for ( i= 0; i < 1000; ++i) {
		imu963ra_get_gyro(); 
        GyroOffset.Zdata += imu963ra_gyro_z;
        system_delay_ms(5);
    }

    GyroOffset.Zdata /= 1000;

	GyroOffset_init=1;
}
/*互补滤波*/
void ICM_hubu(void)
{
		//imu660ra_get_acc(); //陀螺仪采值
    imu963ra_get_gyro();//陀螺仪采值

    z_gyro_last = z_gyro_less;
	z_gyro_less = z_gyro;
	z_gyro= ((float)imu963ra_gyro_z-GyroOffset.Zdata);	  
	Z_gyro_final=LowPassFilter_Silding(z_gyro,z_gyro_less,z_gyro_last);
	
	yaw = (float)((Z_gyro_final)/ 14.3f)*dtimu*-1;//乘了-1，因为现在陀螺仪有问题
	if(fabs(yaw)<0.01) yaw=0;
	yaw_angle += yaw;

//	if (yaw_angle >180) 
//	{	
//        yaw_angle -= 360;
//    } 
//	 if (yaw_angle < -180) 
//	{
//        yaw_angle+=360;
//    }

		//a=atan(imu963ra_mag_y/imu963ra_mag_x);
//		a=(-atan2((imu963ra_mag_y),(imu963ra_mag_x)))*180/3.14159265;
//    if(a<0)
//		{
//			a=a+360;
//		}
		
}
double LowPassFilter_Silding(double dataNewest,double dataMiddle,double dataLast)
{
	double result;
	result = 0.5f *dataNewest+ 0.3f *dataMiddle+0.2f *dataLast;
	return result;
}			
uint16 LowPassFilter_Average(uint16 data[],uint16 length)
{
	int32 add=0;
	int16 result;
	int i;
	for(i=0;i<length;i++)
	{
		add += data[i];
	}
    result=add/length;
    return result;
}

