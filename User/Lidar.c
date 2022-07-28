#include "Lidar.h"

sl_lidar_response_measurement_node_hq_t nodes[2][8192];
uint8_t lidar_init_flag;
int lidar_buffer_pointer=0;
uint8_t lidar_buffer_switcher=0;
int debug=0;

void data_update(uint8_t *raw_data)
{
	static uint16_t angle_last=0;
	uint16_t angle;
	angle=((raw_data[3]&0x7F)<<8)|raw_data[2];
	if((angle<angle_last))
	{
		lidar_buffer_switcher=!lidar_buffer_switcher;
		debug=lidar_buffer_pointer;
		lidar_buffer_pointer=0;
	}
	angle=((raw_data[3]&0x7F)<<8)|raw_data[2];
	for(int i=0;i<40;i++)
	{
		nodes[lidar_buffer_switcher][lidar_buffer_pointer].angle_z_q14=angle;
		nodes[lidar_buffer_switcher][lidar_buffer_pointer].dist_mm_q2=*(short*)(&raw_data[4+i*2]);
		lidar_buffer_pointer++;
	}
	angle_last=angle;
	return;
}
