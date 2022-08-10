#include "Lidar.h"
#include "math.h"
#include "arm_math.h"
sl_lidar_response_measurement_node_hq_t nodes[8192];
uint8_t lidar_init_flag;
int lidar_buffer_pointer=0;
uint8_t lidar_buffer_switcher=0;
int debug;
// A C I K�ĸ�ͷ�������������λ��
const Point2f Pratical_Center_A = (Point2f){8.0f, 7.2f};
const Point2f Pratical_Center_C = (Point2f){8.0f, 2.2f};
const Point2f Pratical_Center_I = (Point2f){1.0f, 7.2f};
const Point2f Pratical_Center_K = (Point2f){1.0f, 2.2f};

// Ѱ��ͷ����ķ�Χ
const float BOHH_Scope = 1.0f;

const static double Pi = 3.14159265358979f;

// �״��ڳ����ϵ�X Y Z����
static float Lidar_X = 4.5f;
static float Lidar_Y = 0.7f;
static float Lidar_A = Pi/2.0f;

Point2f BOHH[4];

void data_update(uint8_t *raw_data)
{
	static uint16_t angle_last;
	uint16_t angle;
	angle=((raw_data[3]&0x7f)<<8)|raw_data[2];
	
	if(angle_last>angle)
	{
		Get_BOHH_Coordinate(nodes,lidar_buffer_pointer,BOHH);
		lidar_buffer_switcher=!lidar_buffer_switcher;
		debug=lidar_buffer_pointer;
		lidar_buffer_pointer=0;
	}
	int offset=angle-angle_last;
	if(offset<0)
		offset+=360*64;
	debug=raw_data[3]&0x80;
	for(int i=0;i<40;i++)
	{
		nodes[lidar_buffer_pointer].angle_z_q14=angle+offset/40*i;
		if(nodes[lidar_buffer_pointer].angle_z_q14>360*64)
			nodes[lidar_buffer_pointer].angle_z_q14-=360*64;
		//nodes[lidar_buffer_pointer].angle_z_q14/=64;
		nodes[lidar_buffer_pointer].dist_mm_q2=(raw_data[4+i*2+1]<<8)|(raw_data[4+i*2]);
		lidar_buffer_pointer++;
	}
	angle_last=angle;
	return;
}

/**
 * @brief �����״�ɨ�����Ϣ������ĸ�ͷ�������������
 * 
 * @param Nodes �״�ɨ��һȦ������
 * @param NodeCount �״�ɨ��һȦ��õ����鳤��
 * @param BOHH_Coordinate ��������ĸ�ͷ��������꣬Ӧ����Point2f����Ϊ4�����飬�ֱ����A C I K��ͷ��������ꣻ��û�ҵ���Ӧ��ͷ����������Ϊ(0.0,0.0)
 */
void Get_BOHH_Coordinate(sl_lidar_response_measurement_node_hq_t *Nodes, size_t NodeCount, Point2f *BOHH_Coordinate)
{
    // �ĸ�ͷ����ľ�ֵͳ��
    struct SUM_BOHH
    {
        double Sum_X;
        double Sum_Y;
        int Num;
    }Sum_Bohh[4];
	
	for(int i=0;i<4;i++)
	{
		Sum_Bohh[i].Num=0;
		Sum_Bohh[i].Sum_X=0;
		Sum_Bohh[i].Sum_Y=0;
	}

    for(int i=0; i<NodeCount; i++)
    {
        // ��ȡ�õ�ĽǶȺ;���
        float angle_in_degrees = Nodes[i].angle_z_q14  / 64;
        float distance_in_meters = Nodes[i].dist_mm_q2 / 1000.f;

        // ̫���ĵ������������
        if(fabs(distance_in_meters) < 0.05f) continue;

        // �ǶȻ��㵽����
        float angle_in_radians = angle_in_degrees * Pi/180.0000f;

        // ���㵽�״�xy����ϵ
        float point_lidar_x = cos(angle_in_radians)*distance_in_meters;
        float point_lidar_y = sin(-angle_in_radians)*distance_in_meters;
        
        // ���㵽����xy����ϵ
        float point_world_x = point_lidar_x*cos(Lidar_A) - point_lidar_y*sin(Lidar_A);
        float point_world_y = point_lidar_x*sin(Lidar_A) + point_lidar_y*cos(Lidar_A);

        point_world_x += Lidar_X, point_world_y += Lidar_Y;

        /*****ͳ����������xy����ϵ��BOHH�����ֵ*****/
		debug=i;
        // A
        if( ( point_world_x >= Pratical_Center_A.x - BOHH_Scope && point_world_x <= Pratical_Center_A.x + BOHH_Scope ) &&
            ( point_world_y >= Pratical_Center_A.y - BOHH_Scope && point_world_y <= Pratical_Center_A.y + BOHH_Scope ) )
        {
            Sum_Bohh[0].Sum_X += point_world_x;
            Sum_Bohh[0].Sum_Y += point_world_y;
            Sum_Bohh[0].Num ++;
        }

        // C
        if( ( point_world_x >= Pratical_Center_C.x - BOHH_Scope && point_world_x <= Pratical_Center_C.x + BOHH_Scope ) &&
            ( point_world_y >= Pratical_Center_C.y - BOHH_Scope && point_world_y <= Pratical_Center_C.y + BOHH_Scope ) )
        {
            Sum_Bohh[1].Sum_X += point_world_x;
            Sum_Bohh[1].Sum_Y += point_world_y;
            Sum_Bohh[1].Num ++;
        }

        // I
        if( ( point_world_x >= Pratical_Center_I.x - BOHH_Scope && point_world_x <= Pratical_Center_I.x + BOHH_Scope ) &&
            ( point_world_y >= Pratical_Center_I.y - BOHH_Scope && point_world_y <= Pratical_Center_I.y + BOHH_Scope ) )
        {
            Sum_Bohh[2].Sum_X += point_world_x;
            Sum_Bohh[2].Sum_Y += point_world_y;
            Sum_Bohh[2].Num ++;
        }

        // K
        if( ( point_world_x >= Pratical_Center_K.x - BOHH_Scope && point_world_x <= Pratical_Center_K.x + BOHH_Scope ) &&
            ( point_world_y >= Pratical_Center_K.y - BOHH_Scope && point_world_y <= Pratical_Center_K.y + BOHH_Scope ) )
        {
            Sum_Bohh[3].Sum_X += point_world_x;
            Sum_Bohh[3].Sum_Y += point_world_y;
            Sum_Bohh[3].Num ++;
        }

        /*****ͳ����������xy����ϵ��BOHH�����ֵ*****/

    }

    // ���ֵ������
    for(int i=0; i<4; i++)
    {
        if(Sum_Bohh[i].Num != 0)
        {
			if(Sum_Bohh[i].Num>1)
			{
				BOHH_Coordinate[i].x = Sum_Bohh[i].Sum_X / (double)Sum_Bohh[i].Num;
				BOHH_Coordinate[i].y = Sum_Bohh[i].Sum_Y / (double)Sum_Bohh[i].Num;
				BOHH_Coordinate[i].points=Sum_Bohh[i].Num;
				if(i==0)
					i=i;
			}
			else
			{
				BOHH_Coordinate[i].x=0;
				BOHH_Coordinate[i].y=0;
				BOHH_Coordinate[i].points=0;
			}
			if(BOHH_Coordinate[2].x!=0||BOHH_Coordinate[2].y!=0)
			{
				__ASM("nop");
			}
        }
        else
            BOHH_Coordinate[i].x = BOHH_Coordinate[i].y = 0.0f;
    }
}

/**
 * @brief �ҵ�ͷ���������xy����֮�󣬽�ͷ���������xy����ת���ɵ����Ҫ����̨�ǶȺ;���
 * 
 * @param BOHH_Coordinate �ҵ���ͷ���������xy����
 * @param Angle ���صĵ����Ҫ����̨�Ƕ�
 * @param Distance ���صĵ����Ҫ��ͷ���������̨�ľ���
 */
void Tran_XY_To_Angle_Distance(Point2f BOHH_Coordinate, float *Angle, float *Distance)
{
    // �ȴ�����xy����ת���״�xy����
    float res_x = BOHH_Coordinate.x - Lidar_X;
    float res_y = BOHH_Coordinate.y - Lidar_Y;
	
    float point_lidar_x = res_x*cos(Lidar_A) + res_y*sin(Lidar_A);
    float point_lidar_y = res_x*sin(-Lidar_A) + res_y*cos(Lidar_A);
	
	if(BOHH_Coordinate.x==0&&BOHH_Coordinate.y==0)
	{
		*Angle=-190;
		*Distance=-190;
		return;
	}
	
    // ���״�xy����ƽ�Ƶ���̨xy����
    float point_shooter_x = point_lidar_x + 65.18f*1e-3;
    float point_shooter_y = point_lidar_y - 286.36f*1e-3;

    // ����̨xy������㵽�����Ҫ�ĽǶȺ;���
    *Angle = atan2(point_shooter_y, point_shooter_x);

    while(*Angle >= 2*Pi) *Angle -= 2*Pi;
    while(*Angle < 0) *Angle += 2*Pi;

    *Angle = *Angle * 180.0f / Pi;
    if(*Angle >= 180.0f) *Angle -= 360.0f;

    *Distance = sqrt(point_shooter_x*point_shooter_x + point_shooter_y*point_shooter_y);
}
