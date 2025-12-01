#include "motor.h"
#include "string.h"
#include "config.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_encoder.h"
#include "pid.h"

int up_left_speed = 0;
int up_right_speed = 0;
int down_left_speed = 0;
int down_right_speed = 0;

int speed_three_array[3] = {0};
int speed_encoder[4] = {0};

extern int encoder_data_1;
extern int encoder_data_2;
extern int encoder_data_3;
extern int encoder_data_4;

void motor_init(void)
{
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
}

void encoder_init(void)
{
	encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);                     // 初始化编码器模块与引脚 正交解码编码器模式
}

void motor_pwm(int up_left_speed,int up_right_speed,int down_left_speed,int down_right_speed)
{
	if(up_left_speed >= 0)                                                           // 正转
    {
		gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                     // DIR输出高电平
        pwm_set_duty(MOTOR1_PWM, up_left_speed);                   // 计算占空比
     }
     else                                                                    // 反转
     {
		gpio_set_level(MOTOR1_DIR, GPIO_LOW);                    // DIR输出低电平
        pwm_set_duty(MOTOR1_PWM, -up_left_speed);                // 计算占空比

     }
	 if (up_right_speed >= 0)
	 {
		 gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                       // DIR输出高电平
         pwm_set_duty(MOTOR2_PWM, up_right_speed);                   // 计算占空比
	 }
	 else
	 {
		 gpio_set_level(MOTOR2_DIR, GPIO_LOW);                     // DIR输出低电平
         pwm_set_duty(MOTOR2_PWM, -up_right_speed);                // 计算占空比
	 }
	 if (down_left_speed >= 0)
	 {
		 gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                       // DIR输出高电平
         pwm_set_duty(MOTOR3_PWM, down_left_speed);                   // 计算占空比
	 }
	 else
	 {
		 gpio_set_level(MOTOR3_DIR, GPIO_LOW);                      // DIR输出低电平
         pwm_set_duty(MOTOR3_PWM, -down_left_speed);                // 计算占空比
	 }
	 if (down_right_speed >= 0)
	 {
		 gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                       // DIR输出高电平
         pwm_set_duty(MOTOR4_PWM, down_right_speed);                  // 计算占空比
	 }
	 else
	 {
		 gpio_set_level(MOTOR4_DIR, GPIO_LOW);                       // DIR输出低电平
         pwm_set_duty(MOTOR4_PWM, -down_right_speed);                // 计算占空比
	 }
}

int Limit_int(int left_limit, int target_num, int right_limit)
{
	if (left_limit > right_limit )
	{
		int temp = left_limit;
		left_limit = right_limit;
		right_limit  =temp;
	}
	if (target_num < left_limit)
	{
		return left_limit;
	}
	else if (target_num > right_limit)
	{
		return right_limit;
	}
	else 
	{
		return target_num;
	}
}

void motor_control(int* input_speed_encoder)
{
	int motorUL_pwm_value = 0;
	int motorUR_pwm_value = 0;
	int motorDL_pwm_value = 0;
	int motorDR_pwm_value = 0;
	motorUL_pwm_value = Limit_int(LIMIT_PWM_MIN, PID_Add_Calculate(&ULpid, encoder_data_4, input_speed_encoder[0]), LIMIT_PWM_MAX);   //上左
	motorUR_pwm_value = Limit_int(LIMIT_PWM_MIN, PID_Add_Calculate(&URpid, encoder_data_3, input_speed_encoder[1]), LIMIT_PWM_MAX);   //上右
	motorDL_pwm_value = Limit_int(LIMIT_PWM_MIN, PID_Add_Calculate(&DLpid, encoder_data_1, input_speed_encoder[2]), LIMIT_PWM_MAX);   //下左
	motorDR_pwm_value = Limit_int(LIMIT_PWM_MIN, PID_Add_Calculate(&DRpid, encoder_data_2, input_speed_encoder[3]), LIMIT_PWM_MAX);   //下右
	motor_pwm(motorDR_pwm_value, motorDL_pwm_value,motorUR_pwm_value,motorUL_pwm_value);
}


double pulse_per_meter = 0;
float rx_plus_ry_cali = 0.3;
double angular_correction_factor = 1.0;
double linear_correction_factor = 1.5;
//double angular_correction_factor = 1.0;
float r_x = 0;
float r_y = 0;

/**
  * @函数作用：运动学解析参数初始化
  */
void Kinematics_Init(void)
{
	//轮子转动一圈，移动的距离为轮子的周长WHEEL_DIAMETER*3.1415926，编码器产生的脉冲信号为ENCODER_RESOLUTION。则电机编码器转一圈产生的脉冲信号除以轮子周长可得轮子前进1m的距离所对应编码器计数的变化
    pulse_per_meter = (float)(ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926))/linear_correction_factor;      //14986.176
    
    r_x = D_X/2;
    r_y = D_Y/2;
    rx_plus_ry_cali = (r_x + r_y)/angular_correction_factor;
	memset(&speed_three_array, 0, sizeof(speed_three_array));
	memset(&speed_encoder, 0, sizeof(speed_encoder));
}

/**
  * @函数作用：逆向运动学解析，底盘三轴速度-->轮子速度
  * @输入：机器人三轴速度 m/s
  * @输出：电机应达到的目标速度（一个PID控制周期内，电机编码器计数值的变化）
  */
void Kinematics_Inverse(int* input, int* output)
{
	float v_tx   = (float)input[0]/100.0f;         //正值向前
	float v_ty   = (float)input[1]/100.0f;         //正值向左
	float omega = (float)input[2]/20.0f;           //正值向左
	static float v_w[4] = {0};
	
	v_w[0] = v_tx - v_ty - (r_x + r_y)*omega;               //rx+ry=0.215
	v_w[1] = v_tx + v_ty + (r_x + r_y)*omega;
	v_w[2] = v_tx + v_ty - (r_x + r_y)*omega;
	v_w[3] = v_tx - v_ty + (r_x + r_y)*omega;

    //计算一个PID控制周期内，电机编码器计数值的变化
	output[0] = (int)(v_w[0] * pulse_per_meter/PID_RATE);   //上左    *150
	output[1] = (int)(v_w[1] * pulse_per_meter/PID_RATE);   //上右
	output[2] = (int)(v_w[2] * pulse_per_meter/PID_RATE);   //下左
	output[3] = (int)(v_w[3] * pulse_per_meter/PID_RATE);   //下右
	
	output[0] = Limit_int(LIMIT_ENCODER_MIN, output[0], LIMIT_ENCODER_MAX);
	output[1] = Limit_int(LIMIT_ENCODER_MIN, output[1], LIMIT_ENCODER_MAX);
	output[2] = Limit_int(LIMIT_ENCODER_MIN, output[2], LIMIT_ENCODER_MAX);
	output[3] = Limit_int(LIMIT_ENCODER_MIN, output[3], LIMIT_ENCODER_MAX);
}

