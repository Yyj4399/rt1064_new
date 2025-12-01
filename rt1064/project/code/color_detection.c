#include "color_detection.h"
#include "data_handle.h"
#include "math.h"

float image_center_y_calculation(int center_y)
{
	if (center_y >= CAMERA_HEIGHT)
	{
		center_y = CAMERA_HEIGHT;
	}
	else if (center_y <= 0)
	{
		center_y = 0;
	}
	float distance = (-1.05)*(float)center_y + 227.0;
	if (distance < 0)
		distance = 0;
	return distance;
}

float image_width_calculation(int width)
{
	if (width >= CAMERA_WIDTH)
	{
		width = CAMERA_WIDTH;
	}
	else if (width <= 0)
	{
		width = 0;
	}
	float distance = (float)0.0233*width*width + (-4.602)*width + 266.775;
	if (distance < 0)
		distance = 0;
	return distance;
}

float image_area_calculation(int area)
{
	if (area < 0)
		area = 0;
	float small_area = (float)area/1000.0f;
	float distance = (-0.2478)*small_area*small_area*small_area + (6.3954)*small_area*small_area + (-53.52)*small_area + 191.835;
	if (distance < 0)
		distance = 0;
	return distance;
}

void color_distance_handle(void)
{
	if (blob_info.valid == 1)             //确认检测到有红色块
	{
		if (blob_info.area < 700 && blob_info.cy < 130)
		{
			blob_info.distance = 200;
		}
		else if (blob_info.area >= 700 && blob_info.area <= 3600)
		{
			blob_info.distance = 0.9*image_area_calculation(blob_info.area) + 0.1*image_center_y_calculation(blob_info.cy);
		}
		else if (blob_info.area > 3600)
		{
			blob_info.distance = image_center_y_calculation(blob_info.cy);
		}
//		Continuous_detection(blob_info.cx,blob_info.distance);
	}
	else
	{
		blob_info.distance = 0;
	}
}

uint8 camera_x_flag = 0;
uint8 continuous_camera_x_num = 0;
uint8 camera_distance_flag = 0;
uint8 continuous_camera_distance_num = 0;

void Continuous_detection(int camera_x,float distance)
{
	if (!camera_x_flag && distance < 22)
	{
		if (abs(camera_x - TARGET_X) < 6 && camera_x != 0)
		{
			continuous_camera_x_num++;
		}
		else
		{
			continuous_camera_x_num = 0;
		}
		if (continuous_camera_x_num >= 10)
		{
			camera_x_flag = 1;
			continuous_camera_x_num = 0;
		}
	}
	
	if (!camera_distance_flag)
	{
		if (fabs(distance - TARGET_DISTANCE) < 6 && distance != 0)
		{
			continuous_camera_distance_num++;
		}
		else
		{
			continuous_camera_distance_num = 0;
		}
		if (continuous_camera_distance_num >= 10)
		{
			camera_distance_flag = 1;
			continuous_camera_distance_num = 0;
		}
	}
}