#ifndef _LIDAR_H_
#define _LIDAR_H_
#include "main.h"
#include "stdint.h"




/*�ṹ������*/
typedef struct sl_lidar_response_measurement_node_hq_t
{
    float   angle_z_q14;
    uint32_t   dist_mm_q2;
    uint8_t    quality;
    uint8_t    flag;
} __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;

// Point2f�ṹ��
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
 * @brief �����״�ɨ�����Ϣ������ĸ�ͷ�������������
 * 
 * @param Nodes �״�ɨ��һȦ������
 * @param NodeCount �״�ɨ��һȦ��õ����鳤��
 * @param BOHH_Coordinate ��������ĸ�ͷ��������꣬Ӧ����Point2f����Ϊ4�����飬�ֱ����A C I K��ͷ��������ꣻ��û�ҵ���Ӧ��ͷ����������Ϊ(0.0,0.0)
 */
void Get_BOHH_Coordinate(sl_lidar_response_measurement_node_hq_t *Nodes, size_t NodeCount, Point2f *BOHH_Coordinate);

/**
 * @brief �ҵ�ͷ���������xy����֮�󣬽�ͷ���������xy����ת���ɵ����Ҫ����̨�ǶȺ;���
 * 
 * @param BOHH_Coordinate �ҵ���ͷ���������xy����
 * @param Angle ���صĵ����Ҫ����̨�Ƕ�
 * @param Distance ���صĵ����Ҫ��ͷ���������̨�ľ���
 */
void Tran_XY_To_Angle_Distance(Point2f BOHH_Coordinate, float *Angle, float *Distance);


extern sl_lidar_response_measurement_node_hq_t nodes[8192];
extern uint8_t lidar_init_flag;
extern int lidar_buffer_pointer;
extern uint8_t lidar_buffer_switcher;
extern Point2f BOHH[4];

void data_update(uint8_t *raw_data);

#endif
