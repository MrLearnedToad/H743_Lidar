#ifndef _LIDAR_H_
#define _LIDAR_H_
#include "main.h"
#include "stdint.h"
/*结构体声明*/
typedef struct sl_lidar_response_measurement_node_hq_t
{
    uint16_t   angle_z_q14;
    uint32_t   dist_mm_q2;
    uint8_t    quality;
    uint8_t    flag;
} __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;


extern sl_lidar_response_measurement_node_hq_t nodes[2][8192];
extern uint8_t lidar_init_flag;
extern int lidar_buffer_pointer;
extern uint8_t lidar_buffer_switcher;

void data_update(uint8_t *raw_data);

#endif
