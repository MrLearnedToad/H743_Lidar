#ifndef _LIDAR_H_
#define _LIDAR_H_
#include "main.h"
#include "stdint.h"




/*结构体声明*/
typedef struct sl_lidar_response_measurement_node_hq_t
{
    float   angle_z_q14;
    uint32_t   dist_mm_q2;
    uint8_t    quality;
    uint8_t    flag;
} __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;

// Point2f结构体
typedef struct Point2f
{
    float x;
    float y;
	int points;
}Point2f;

typedef struct chassis_info
{
	int chassis_coordinate_x;
	int chassis_coordinate_y;
	int chassis_coordinate_z;
	int turret_angle;
	int turret_dist;
}chassis_info;

/**
 * @brief 根据雷达扫描的信息计算出四个头顶球的世界坐标
 * 
 * @param Nodes 雷达扫描一圈的数组
 * @param NodeCount 雷达扫描一圈获得的数组长度
 * @param BOHH_Coordinate 计算出的四个头顶球的坐标，应传入Point2f长度为4的数组，分别代表A C I K的头顶球的坐标；若没找到对应的头顶球，则坐标为(0.0,0.0)
 */
void Get_BOHH_Coordinate(sl_lidar_response_measurement_node_hq_t *Nodes, size_t NodeCount, Point2f *BOHH_Coordinate);

/**
 * @brief 找到头顶球的世界xy坐标之后，将头顶球的世界xy坐标转化成电控需要的云台角度和距离
 * 
 * @param BOHH_Coordinate 找到的头顶球的世界xy坐标
 * @param Angle 返回的电控需要的云台角度
 * @param Distance 返回的电控需要的头顶球距离云台的距离
 */
void Tran_XY_To_Angle_Distance(Point2f BOHH_Coordinate, float *Angle, float *Distance);


extern sl_lidar_response_measurement_node_hq_t nodes[8192];
extern uint8_t lidar_init_flag;
extern int lidar_buffer_pointer;
extern uint8_t lidar_buffer_switcher;
extern Point2f BOHH[4];

void data_update(uint8_t *raw_data);

#endif
