#include "user_task.h"
#include "my_robot.h"
#include "math.h"
#include "Resolve.h"
#include "fdcan_bsp.h"
#include "move.h"
#include "arm_math.h"
#include "Remote_Control.h"
#include "semphr.h"
/*函数指针区*/
int (*autodirveshortdistance)(mission_queue*)=auto_drive_shortdistance;
int (*autodrivelongdistance)(mission_queue*)=auto_drive_longdistance;
int (*grabposset)(mission_queue*)=grab_pos_set;
int (*hookgrasp)(mission_queue*)=hook_grasp;
int (*hookrelease)(mission_queue*)=hook_release;
int (*switcherdirectionset)(mission_queue*)=switcher_direction_set;
int (*catapultactivate)(mission_queue*)=catapult_activate;
int (*autopickup)(mission_queue *current_task)=auto_pick_up;
int (*autoplace)(mission_queue *current_task)=auto_place;
int (*posregulatorposset)(mission_queue *current_task)=pos_regulator_pos_set;
int (*pickupactivatorposset)(mission_queue *current_task)=pick_up_activator_pos_set;
int (*autoturn)(mission_queue *current_task)=auto_turn;
int (*taskqueuedelay)(mission_queue *current_task)=task_queue_delay;
int (*moveforward)(mission_queue *current_task)=move_forward;
int (*fuckblock)(mission_queue *current_task)=fuck_block;
int (*autopickuplong)(mission_queue *current_task)=auto_pick_up_long;
int (*moveforward2)(mission_queue *current_task)=move_forward2;
/*全局变量区*/
Ort target_pos;
int current_target_ID=0;
float short_drive_deadzone=0.2f;
extern uint8_t get_block_flag;
uint8_t thread_lock=0;
uint8_t final_point_lock;
fdcan_msg_queue *fdcan_msg_queue_head=NULL;
Ort r_correction_value;

/******************* **************************************************************
  *@  name      : auto_drive_shortdistance
  *@  function  : 机器人短距离移动函数
  *@  input     : current_task
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
int auto_drive_shortdistance(mission_queue *current_task)
{
    static int flag_running=0;
    double distance;
//    Ort velocity_vector;
    distance=sqrtf((current_pos.x-current_task->info.x)*(current_pos.x-current_task->info.x)+(current_pos.y-current_task->info.y)*(current_pos.y-current_task->info.y));
//    pid_distance=-Pid_Run(&pid_pos,0,distance);
//    velocity_vector.x=(current_task->info.x-current_pos.x)/distance*pid_distance;
//    velocity_vector.y=(current_task->info.y-current_pos.y)/distance*pid_distance;
//    dX=velocity_vector.x;
//    dY=velocity_vector.y;
    if(flag_running==0)
    {
        //send_log(2,current_pos.x,current_pos.y,current_task->info.x,current_task->info.y,&huart3);
        
        pre_plan(current_task->info);
        global_clock=1;
        flag_running=1;
        flags[auto_drive_status]=moving;
    }
    
    if(distance<0.05||global_clock>500||Read_Button(27)==1)
    {
//        dX=0;
//        dY=0;
        current_task->flag_finish=1;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
    }
    if((distance<short_drive_deadzone||(current_task->info.z!=moving_partially_complete3&&(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)||HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)||HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))))&&current_task->info.z>6&&current_task->info.z<10)
    {
//        dX=0;
//        dY=0;
        current_task->flag_finish=1;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
    }
    if(current_task->info.z==moving_place_block&&HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1))
    {
        current_task->flag_finish=1;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
    }
        
    return 0;
}

int auto_turn(mission_queue *current_task)
{
    dZ=current_task->info.z;
    current_task->flag_finish=1;
    return 0;
}

/*********************************************************************************
  *@  name      : auto_drive_longdistance
  *@  function  : 机器人长距离移动函数
  *@  input     : current_task
  *@  output    : NULL 
  *@  note      : NULL
*********************************************************************************/
uint8_t msg;
TaskHandle_t task_handle_temp;
int auto_drive_longdistance(mission_queue *current_task)
{
    
    static int flag_running=0,flag_start_signal_send=0,error_sum=0;
    static Ort current_point;
    uint8_t set_flags[20];
    Ort info;
    for(int i=0;i<20;i++)
    {
        set_flags[i]=either;
    }
    barrier *barr=find_barrier(1);
    
    if(final_point.x==0||final_point.y==0)
    {
        msg=7;
        current_task->flag_finish=1;
        final_point_lock=0;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
        flag_start_signal_send=0;
        xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
        error_sum=0;
        send_log(8,0,0,0,0,&huart3);
        return 0;
    }
    
    if(flag_running==0)//开始起步部分
    {
        flag_running=1;
        current_point=planned_path[0];
        dZ=-atan2f(current_point.x-current_pos.x,current_point.y-current_pos.y)*180.0f/3.1415926f+135.0f;
        if(dZ<-180)
        {
            dZ=dZ+360;
        }
        else if(dZ>180)
        {
            dZ=dZ-360;
        }
        pre_plan(current_point);   
        flags[auto_drive_status]=moving;
        global_clock=3;
        return 0;
    }
    dZ=-atan2f(barr->location.x-current_pos.x,barr->location.y-current_pos.y)*180.0f/3.1415926f+135.0f;
    if(dZ<-180)
    {
        dZ=dZ+360;
    }
    else if(dZ>180)
    {
        dZ=dZ-360;
    }
    if(flag_start_signal_send==0&&fabs(current_pos.z+dZ)<20)
    {
        msg=6;     
        //xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
        flag_start_signal_send=1;
    }
    
    if(current_point.x!=final_point.x||current_point.y!=final_point.y)
    {
        if(((current_pos.x-current_point.x)*(current_pos.x-current_point.x)+(current_pos.y-current_point.y)*(current_pos.y-current_point.y))<=deadzone*deadzone)//检测是否到达路径点附近，到达且不是最后一个则前往下一个路径点
        {
            for(int i=0;i<5;i++)
            {
                if(pow(current_pos.x-planned_path[i].x,2)+pow(current_pos.y-planned_path[i].y,2)>0.04f&&planned_path[i].x>0)
                {
                    current_point=planned_path[i];
                    planned_path[i].x=-1;
                    break;
                }
                if(i==4)
                {
                    error_sum++;
                    if(error_sum<=5)
                        break;
                    msg=7;
                    current_task->flag_finish=1;
                    final_point_lock=0;
                    flags[auto_drive_status]=current_task->info.z;
                    flag_running=0;
                    flags[drivemode]=manualmode;
                    flag_start_signal_send=0;
                    //xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
                    error_sum=0;
                    //send_log(9,0,0,0,0,&huart3);
                    return 0;
                }
            }
                
            
            if(((final_point.x-current_point.x)*(final_point.x-current_point.x)+(final_point.y-current_point.y)*(final_point.y-current_point.y))<0.00010f)
            {
                dZ=-atan2f(barr->location.x-current_point.x,barr->location.y-current_point.y)*180.0f/3.1415926f;
                final_point_lock=1;
            }
            thread_lock=1;
            pre_plan(current_point);
            //send_log(0x05,current_point.x,current_point.y,0,0,&huart3);
            global_clock=7;
            thread_lock=0;
        }
    }
    if(((final_point.x-current_pos.x)*(final_point.x-current_pos.x)+(final_point.y-current_pos.y)*(final_point.y-current_pos.y))<0.09f||
        Read_Button(27)==1)//到达目的地附近
    {
        msg=7;
        error_sum=0;
        current_task->flag_finish=1;
        final_point_lock=0;
        flags[auto_drive_status]=current_task->info.z;
        flag_running=0;
        flags[drivemode]=manualmode;
        flag_start_signal_send=0;
        send_log(10,final_point.x,final_point.y,(float)dX,(float)dY,&huart3);
        if(((final_point.x-current_pos.x)*(final_point.x-current_pos.x)+(final_point.y-current_pos.y)*(final_point.y-current_pos.y))<0.10f)
            add_mission(AUTOPLACE,set_flags,0,&info);
        xTaskCreate(send_msg_synchronal,"kksk",100,&msg,osPriorityNormal,&task_handle_temp);
        return 0;
    }
    return 0;
}
/*********************************************************************************
  *@  name      : grab_pos_set
  *@  function  : 抬升机构位置设定
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int grab_pos_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[grab_status]==stop)
    {
        flags[grab_status]=moving;
        can_msg[0]=(int)current_task->info.x;
        flags[grab_pos]=can_msg[0];
        fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[0]==1)
    {
        current_task->flag_finish=1;
        flags[grab_status]=stop;
        cmd_feedback[0]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : hook_grasp
  *@  function  : 夹块
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int hook_grasp(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[hook_status]==stop)
    {
        flags[hook_status]=moving;
        can_msg[1]=block_num-1;
        if(block_num>6)
            can_msg[1]-=5;
        flags[hook_pos]=grasp;
        fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[1]==1)
    {
        current_task->flag_finish=1;
        flags[hook_status]=stop;
        cmd_feedback[1]=0;
    }
    return 0;
}    

/*********************************************************************************
  *@  name      : hook_release
  *@  function  : 释放
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int hook_release(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[hook_status]==stop)
    {
        flags[hook_status]=moving;
        can_msg[2]=1;
        flags[hook_pos]=release;
        fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[2]==1)
    {
        current_task->flag_finish=1;
        flags[hook_status]=stop;
        cmd_feedback[2]=0;
    }
    return 0;
}    
int debuggg=0;
/*********************************************************************************
  *@  name      : switcher_direction_set
  *@  function  : 设定翻转位置
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int switcher_direction_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    
    if(flags[switcher_status]==stop&&current_task->info.y!=1)
    {
        flags[switcher_status]=moving;
        flags[switcher_pos]=current_task->info.x;
        can_msg[3]=current_task->info.x+1;
        fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
    }
    if(flags[switcher_status]==stop&&current_task->info.y==1)
    {
        
//        {
            if(block_color==down)
            {
                if(flags[switcher_pos]==up)
                {
                    flags[switcher_status]=moving;
                    flags[switcher_pos]=down;
                    can_msg[3]=down+1;
                    osDelay(500);
                    fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
                }
                else if(flags[switcher_pos]==down)
                {
                    flags[switcher_status]=moving;
                    flags[switcher_pos]=up;
                    can_msg[3]=up+1;
                    osDelay(500);
                    fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
                }
            }
            else if(block_color==up)
            {
                current_task->flag_finish=1;
                flags[switcher_status]=stop;
                cmd_feedback[3]=0;
            }
            else if(block_color==forward)
            {
                while(flags[hook_pos]==release||flags[hook_status]==moving)
                    osDelay(50);
                flags[switcher_status]=moving;
                flags[switcher_pos]=up;
                can_msg[3]=up+1;
                osDelay(100);
                fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
            }
            else if(block_color==backward)
            {
                while(flags[hook_pos]==release||flags[hook_status]==moving)
                    osDelay(50);
                flags[switcher_status]=moving;
                flags[switcher_pos]=down;
                can_msg[3]=down+1;
                osDelay(100);
                fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
            }
            debuggg=block_color;
//        }
    }
    if(cmd_feedback[3]==1)
    {
        current_task->flag_finish=1;
        flags[switcher_status]=stop;
        cmd_feedback[3]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : catapult_activate
  *@  function  : NULL
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int catapult_activate(mission_queue *current_task)
{
    
    return 0;
}

/*********************************************************************************
  *@  name      : pos_regulator_pos_set
  *@  function  : NULL
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int pos_regulator_pos_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[regulator_status]==stop)
    {
        flags[regulator_status]=moving;
//        if(current_task->info.x!=-1)
//            flags[regulator_vertical_pos]=current_task->info.x;
//        if(current_task->info.y!=-1)
//            flags[regulator_horizontal_pos]=current_task->info.y;
//        if(current_task->info.z!=-1)
//            flags[regulator_catapult_pos]=current_task->info.z;
        if(current_task->info.z==sweep)
        {
            flags[regulator_R_pos]=sweep;
        }
        else
        {
            flags[regulator_R_pos]=current_task->info.z;
            flags[regulator_L_pos]=current_task->info.z;
        }
        if(current_task->info.z!=-1)
        {
            can_msg[5]=current_task->info.z+1;
        }
//        else
//        {
//            if(current_task->info.x==down)
//            {
//                can_msg[4]=1;
//                
//            }
//            else
//            {
//                can_msg[4]=2;
//            }
//        }
        
        fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[4]==1||cmd_feedback[5]==1)
    {
        current_task->flag_finish=1;
        flags[regulator_status]=stop;
        if(cmd_feedback[4]==1)
            cmd_feedback[4]=0;
        else
            cmd_feedback[5]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : pick_up_activator_pos_set
  *@  function  : NULL
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int pick_up_activator_pos_set(mission_queue *current_task)
{
    uint8_t can_msg[8]={0};
    if(flags[activator_status]==stop)
    {
        flags[activator_status]=moving;
        if(current_task->info.x>6)
           current_task->info.x-=5;
        flags[activator_pos]=current_task->info.x;
        can_msg[6]=7-current_task->info.x;
        
        fdcan_add_msg_2_queue(&hfdcan1,can_msg,0x114,8);
    }
    if(cmd_feedback[6]==1)
    {
        current_task->flag_finish=1;
        flags[activator_status]=stop;
        cmd_feedback[6]=0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : pick_up
  *@  function  : 抓取塔块流程函数
  *@  input     : 塔块朝向
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void pick_up(uint8_t pos,uint8_t mode,uint8_t flag_sensor_mode)
{
    
    uint8_t set_flags[20]={either};
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    Ort info={.x=0,.y=0,.z=0};
    if(mode==automode)
    {
        if(pos==forward||pos==backward)
        {
            info.x=block_num+5;
            if(block_num>6)
              info.x-=5;
            add_mission(GRABPOSSET,set_flags,0,&info);
            info.x=tower_block_5;
            add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
            info.x=forward;
            add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
            info.z=regulate;
            set_flags[grab_status]=stop;
            add_mission(POSREGULATORPOSSET,set_flags,0,&info);
        }
        else
        {
            info.x=up;
            add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
            info.x=tower_bottom;
            add_mission(GRABPOSSET,set_flags,0,&info);
            info.x=block_num;
            add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
            info.z=standby;
            set_flags[grab_status]=stop;
            add_mission(POSREGULATORPOSSET,set_flags,0,&info);
        }
        info.x=0;
        info.y=0;
        info.z=0;
        set_flags[auto_drive_status]=moving_partially_complete1;
        add_mission(HOOKGRASP,set_flags,0,&info);
        
        set_flags[hook_status]=stop;
        set_flags[hook_pos]=grasp;
        
        info.z=standby;
        add_mission(POSREGULATORPOSSET,set_flags,0,&info);
        info.x=tower_bottom;
        add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
        info.x=block_num;
        if(block_num>6)
            info.x-=5;
        add_mission(GRABPOSSET,set_flags,0,&info);
        info.x=0;
        info.y=1;
        add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
        
    }
    else if(flag_sensor_mode==0)
    {
        if(mode==automode)
        {
            add_mission(HOOKRELEASE,set_flags,1,&info);
            info.x=-1;
            info.y=-1;
            info.z=release;
            add_mission(POSREGULATORPOSSET,set_flags,0,&info);
            info.x=0;
            info.y=0;
            info.z=0;
        }
        else if(pos==forward||pos==backward)
        {
            info.x=400;
            flags[delay_status]=stop;
            add_mission(TASKQUEUEDELAY,set_flags,1,&info);
            info.x=0;
        }
        
        set_flags[hook_status]=stop;
        if(pos==up||pos==down)
        {
            if(flags[switcher_pos]!=up&&flags[switcher_pos]!=down)
                info.x=up;
            else
                info.x=flags[switcher_pos];
        }
        else if(mode==automode)
        {
            info.x=forward;
            set_flags[hook_status]=either;
            add_mission(SWITCHERDIRECTIONSET,set_flags,1,&info);
            info.x=tower_block_5;
            add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
            set_flags[hook_status]=stop;
        }
            
        info.x=0;
        set_flags[hook_status]=either;
        
        if((pos==forward||pos==backward)&&mode==automode)
        {
            set_flags[grab_status]=stop;
            set_flags[switcher_status]=stop;
            info.x=block_num+5;
            if(block_num>6)
                info.x-=5;
            add_mission(GRABPOSSET,set_flags,1,&info);
            set_flags[grab_status]=either;
            info.x=0;
        }
        else
        {
            set_flags[grab_status]=stop;
            info.x=tower_bottom;
            add_mission(GRABPOSSET,set_flags,1,&info);
            set_flags[grab_status]=either;
            info.x=0;
        }

        set_flags[hook_pos]=release;
        set_flags[hook_status]=stop;
        set_flags[grab_status]=stop;
        set_flags[switcher_status]=stop;
        
        if(mode==automode)
            set_flags[auto_drive_status]=moving_partially_complete1;
        else if(pos==forward||pos==backward)
            set_flags[delay_status]=delay_complete;
        add_mission(HOOKGRASP,set_flags,1,&info);
        
        for(int i=0;i<total_flags;i++)
        {
            set_flags[i]=either;
        }
        
        set_flags[hook_status]=stop;
        info.x=tower_bottom;
        add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
        info.x=0;
        
//        if(1)
//        {
            set_flags[grab_status]=stop;
            
            set_flags[hook_status]=stop;
            if(mode==automode)
                set_flags[auto_drive_status]=moving_partially_complete1;
            info.x=block_num;
            if(block_num>6)
                info.x-=5;
            add_mission(GRABPOSSET,set_flags,1,&info);
            
            set_flags[grab_status]=either;
            info.x=tower_bottom;
            add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
            info.x=-1;
            info.y=-1;
            info.z=release;
            add_mission(POSREGULATORPOSSET,set_flags,0,&info);
            info.y=0;
            info.z=0;
            info.x=0;
            
            for(int i=0;i<total_flags;i++)
            {
                set_flags[i]=either;
            }
            info.x=0;
//            if(block_num!=6)
//                set_flags[grab_status]=stop;        
            set_flags[hook_status]=stop;
            set_flags[regulator_status]=stop;
            if(block_color==down&&mode==manualmode)
            {
    //            if(flags[switcher_pos]==up)
    //                info.x=down;
    //            else
    //                info.x=up;
                info.y=1;
            }
            else if(pos==down&&mode==automode)
            {
                if(flags[switcher_pos]==up)
                    info.x=down;
                else
                    info.x=up;
            }
            else if(pos==forward&&mode==automode)
            {
                info.x=up;
            }
            else if(pos==backward&&mode==automode)
            {
                info.x=down;
            }
            info.y=1;
            add_mission(SWITCHERDIRECTIONSET,set_flags,0,&info);
            info.x=0;
            set_flags[hook_status]=either;
//        }
//        else
//        {
//            
//            set_flags[hook_status]=stop;
//            info.x=block_num;
//            add_mission(GRABPOSSET,set_flags,1,&info);
//            
//            info.x=tower_bottom;
//            add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
//            info.x=0;
//            
//            info.x=-1;
//            info.y=-1;
//            info.z=release;
//            add_mission(POSREGULATORPOSSET,set_flags,0,&info);
//        }
    }
    else
    {
        info.x=400;//光电门触发后夹块延迟
        flags[delay_status]=stop;
        add_mission(TASKQUEUEDELAY,set_flags,1,&info);
        info.x=0;
        
        if(pos==forward)
            info.x=up;
        else
            info.x=down;
        info.y=1;
        add_mission(SWITCHERDIRECTIONSET,set_flags,1,&info);
        
        set_flags[grab_status]=stop;
        set_flags[switcher_status]=stop;
        set_flags[delay_status]=delay_complete;
        add_mission(HOOKGRASP,set_flags,1,&info);
        for(int i=0;i<total_flags;i++)
        {
            set_flags[i]=either;
        }
        
        set_flags[hook_status]=stop;
        set_flags[hook_pos]=grasp;
        
        info.z=standby;
        add_mission(POSREGULATORPOSSET,set_flags,0,&info);
        info.z=0;
        
        info.x=block_num;
        if(block_num>6)
                info.x-=5;
        info.y=0;
        add_mission(GRABPOSSET,set_flags,1,&info);
        
        info.x=tower_bottom;
        add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&info);
        

        set_flags[grab_status]=either;
        
    }
    return;
}

/*********************************************************************************
  *@  name      : auto_pick_up
  *@  function  : 自动抓取塔块
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int auto_pick_up(mission_queue *current_task)
{
    static uint8_t flag_running,count=0;
    barrier *target;
    static uint8_t direction;
    Ort grasp_pos;
    double distance,distance_2_move;
    uint8_t set_flags[20];
    instruction_refresh();
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    if(flag_running==0)
    {         
        flags[lock_mode_status]=moving;
        flag_running=1;
        flags[auto_drive_status]=stop;
        focus_mode=0;
    }

    if(count==0)
    {
        target=find_barrier(block_num);
        if(target==NULL)
        {
            flags[lock_mode_status]=stop;
            current_task->flag_finish=1;
            flag_running=0;
            count=0;
            return 0;
        }
        
        target_pos.x=target->location.x;
        target_pos.y=target->location.y;
        target_pos.z=atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        dZ=-target_pos.z;
        count=1;
    }
    if(Read_Button(13)==1)
        count=0;
        
    if(flags[auto_drive_status]==stop&&count==1)
    {
        short_drive_deadzone=0.1f;
        target_pos.z=atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        dZ=-target_pos.z;
        distance=sqrtf((current_pos.x-target_pos.x)*(current_pos.x-target_pos.x)+(current_pos.y-target_pos.y)*(current_pos.y-target_pos.y));
        distance_2_move=distance-0.676f;
        grasp_pos.x=current_pos.x+(target_pos.x-current_pos.x)*distance_2_move/distance;
        grasp_pos.y=current_pos.y+(target_pos.y-current_pos.y)*distance_2_move/distance;
        grasp_pos.z=moving_partially_complete2;
        flags[auto_drive_status]=moving;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&grasp_pos);
        set_flags[auto_drive_status]=moving_partially_complete2;
        grasp_pos.x=0;
        grasp_pos.y=0;
        grasp_pos.z=moving_partially_complete1;
        add_mission(MOVEFORWARD2,set_flags,0,&grasp_pos);
        set_flags[auto_drive_status]=either;
        direction=(uint8_t)target->location.z;
        pick_up(target->location.z,automode,0);
    }
    if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_ORE) != RESET) //如果发生了上溢错误，就将标志位清零，并重新开始接收头帧
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
    }
    if(flags[auto_drive_status]==moving_partially_complete1&&flags[hook_status]==stop)
    {
        remove_barrier(block_num);
//        if(direction==up||direction==down)
//        {
//            grasp_pos=evaluate_approach_pos(1,1.5f);
//            if(grasp_pos.x>0)
//            {
//              add_mission(AUTODRIVELONGDISTANCE,set_flags,1,&grasp_pos);
//            }
//        }
        
        
        
        flags[lock_mode_status]=stop;
        current_task->flag_finish=1;
        flag_running=0;
        count=0;
        return 0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : auto_place
  *@  function  : 自动搭塔
  *@  input     : current_task
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
int auto_place(mission_queue *current_task)
{
    static uint8_t flag_running=0,flag_correct=0;
    static Ort p_base_pos,p_correction_value;
    Ort release_pos,stop_pos,offset_vector;
    double distance,distance_2_move;
    int deg;
    uint8_t set_flags[20];
    
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    if(flag_running==0)
    {         
        flag_running=1;
        flags[lock_mode_status]=moving;
        p_correction_value=correction_value;
        
        if(flags[auto_drive_status]!=moving)
            flags[auto_drive_status]=stop;
    }
    if (block_num<7)
    {
        target_pos=find_barrier(1)->location;
        deg=(int)(-atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f)%45;
        while((abs(deg)<5||abs(deg)>40)&&my_sqrt(pow(current_speed.x,2)+pow(current_speed.y,2))>1.0f)
        {
            osDelay(5);
            target_pos=find_barrier(1)->location;
            deg=(int)(-atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f)%45;
        }
        stop_pos=evaluate_place_pos(1,1.2f);
    }
    else
    {
        target_pos=find_barrier(12)->location;
        deg=(int)(-atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f)%45;
        while((abs(deg)<5||abs(deg)>40)&&my_sqrt(pow(current_speed.x,2)+pow(current_speed.y,2))>1.0f)
        {
            osDelay(5);
            target_pos=find_barrier(12)->location;
            deg=(int)(-atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f)%45;
        }
        stop_pos=evaluate_place_pos(12,1.2f);
    }
    dZ=stop_pos.z;
    if(flags[auto_drive_status]==moving_partially_complete2)
    {
        flag_correct=0;
    }

    if(flag_correct)
    {
        offset_vector.x=target_pos.x-p_base_pos.x;
        offset_vector.y=target_pos.y-p_base_pos.y;
        correction_value.x=p_correction_value.x-offset_vector.x;
        correction_value.y=p_correction_value.y-offset_vector.y;
    }

    if(flags[auto_drive_status]==stop&&flag_running==1)
    {
        flags[auto_drive_status]=moving;
        short_drive_deadzone=0.10f;
        flag_correct=1;
        p_base_pos=target_pos;
        p_correction_value=correction_value;
//        dZ=-atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        flag_running=2;
        stop_pos.z=moving_partially_complete1;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&stop_pos);
        distance=sqrtf((stop_pos.x-target_pos.x)*(stop_pos.x-target_pos.x)+(stop_pos.y-target_pos.y)*(stop_pos.y-target_pos.y));
        //distance=sqrtf((current_pos.x-target_pos.x)*(current_pos.x-target_pos.x)+(current_pos.y-target_pos.y)*(current_pos.y-target_pos.y));
        distance_2_move=distance-0.65f;//0.515
        release_pos.x=stop_pos.x+(target_pos.x-stop_pos.x)*distance_2_move/distance;
        release_pos.y=stop_pos.y+(target_pos.y-stop_pos.y)*distance_2_move/distance;
//        release_pos.x=current_pos.x+(target_pos.x-current_pos.x)*distance_2_move/distance;
//        release_pos.y=current_pos.y+(target_pos.y-current_pos.y)*distance_2_move/distance;

        
        release_pos.z=moving_partially_complete2;
        flags[auto_drive_status]=moving;
        
        set_flags[grab_status]=stop;
        
        set_flags[auto_drive_status]=moving_partially_complete1;
        set_flags[deg_offset]=60;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&release_pos);
        set_flags[deg_offset]=either;
        
        release_pos.z=moving_place_block;
        set_flags[auto_drive_status]=moving_partially_complete2;
        add_mission(MOVEFORWARD,set_flags,1,&release_pos);
        
        if(block_num<7)
            place_block(block_num);
        else
            place_block(block_num-5);
        set_flags[hook_status]=stop;
        set_flags[auto_drive_status]=moving_place_block;
        
        if(block_num<7)
        {
            release_pos.x=8.0f;
            release_pos.y=6.0f;
        }
        else
        {
            release_pos.x=4.0f;
            release_pos.y=6.0f;
        }
        float temp=-atan2f(target_pos.x-current_pos.x,target_pos.y-current_pos.y)*180.0f/3.1415926f;
        if(temp>=-135&&temp<-45)
        {
            release_pos.x=release_pos.x-1;
            release_pos.y=release_pos.y;
        }
        else if(temp>=-45&&temp<45)
        {
            release_pos.x=release_pos.x;
            release_pos.y=release_pos.y-1;
        }
        else if(temp>=45&&temp<135)
        {
            release_pos.x=release_pos.x+1;
            release_pos.y=release_pos.y;
        }
        else
        {
            release_pos.x=release_pos.x;
            release_pos.y=release_pos.y+1;
        }
        
        release_pos.z=moving_partially_complete3;
        add_mission(AUTODRIVESHORTDISTANCE,set_flags,1,&release_pos);
        if(block_num!=tower_block_5&&block_num!=tower_block_5+5)
            release_pos.x=block_num+1;
        else
            release_pos.x=block_num;
        if(release_pos.x>6)
            release_pos.x-=5;
        set_flags[auto_drive_status]=either;
        add_mission(GRABPOSSET,set_flags,1,&release_pos);
        
        release_pos.x=tower_bottom;
        set_flags[hook_status]=stop;
        set_flags[auto_drive_status]=moving_partially_complete3;
//        add_mission(HOOKGRASP,set_flags,0,&release_pos);
        set_flags[grab_status]=stop;
        add_mission(GRABPOSSET,set_flags,1,&release_pos);
        set_flags[grab_pos]=tower_bottom;
//        add_mission(HOOKRELEASE,set_flags,0,&release_pos);

    }
    if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_ORE) != RESET) //如果发生了上溢错误，就将标志位清零，并重新开始接收头帧
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
    }
//    if(flags[auto_drive_status]==moving_place_block&&flags[hook_pos]==release&&deg_pid_disable==1)
//    {
//        dZ=-current_pos.z;
//        deg_pid_disable=0;
//    }
    if(flags[auto_drive_status]==moving_partially_complete3)
    {
        get_block_flag=0;
        remove_barrier(block_num);
        pos_reset=1;
        current_task->flag_finish=1;
        block_num++;
        
        for(int i=0;i<total_flags;i++)
        {
            set_flags[i]=either;
        }
        set_flags[grab_status]=stop;
        release_pos.x=block_num;
        add_mission(PICKUPACTIVATORPOSSET,set_flags,0,&release_pos);
        
        release_pos.x=-1;
        release_pos.y=-1;
        release_pos.z=grasp;
        //add_mission(POSREGULATORPOSSET,set_flags,0,&release_pos);
        
//
        
//        if(block_num==4)
//        {
//            release_pos.x=down;
//            release_pos.y=backward;
//            release_pos.z=-1;
//            add_mission(POSREGULATORPOSSET,set_flags,0,&release_pos);
//        }
        
        focus_mode=1;
        flags[lock_mode_status]=stop;
        flag_running=0;
        deg_pid_disable=0;
        return 0;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : place_block
  *@  function  : 搭塔流程函数
  *@  input     : 塔块编号
  *@  output    : NULL
  *@  note      : NULL
*********************************************************************************/
void place_block(uint8_t tower_num)
{
    uint8_t set_flags[20]={either};
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    Ort info={.x=0,.y=0,.z=0};
    
    set_flags[auto_drive_status]=moving_place_block;
    info.x=tower_num+10;
    add_mission(GRABPOSSET,set_flags,0,&info);
    
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    
    set_flags[grab_status]=stop;
    set_flags[auto_drive_status]=moving_place_block;
    
    add_mission(HOOKRELEASE,set_flags,0,&info);
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    
    return;
}

int task_queue_delay(mission_queue *current_task)
{
    flags[delay_status]=delaying;
    osDelay(current_task->info.x);
    current_task->flag_finish=1;
    flags[delay_status]=delay_complete;
    return 0;
}

/*********************************************************************************
  *@  name      : move_forward
  *@  function  : 搭塔流程函数
  *@  input     : 塔块编号
  *@  output    : NULL
  *@  note      : 
*********************************************************************************/
int move_forward(mission_queue *current_task)
{
    static uint8_t PA0_triggered_time=8,PA1_triggered_time=8;
    static int timer=0;
    NNlearn=0;
        
    if(PA0_triggered_time<8)
        PA0_triggered_time++;
    if(PA1_triggered_time<8)
        PA1_triggered_time++;
    
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==1)
        PA1_triggered_time=0;
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1)
        PA0_triggered_time=0;
    
    if(PA0_triggered_time==8&&PA1_triggered_time!=8)
    {
        open_loop_velocity.y=0.4f;
        open_loop_velocity.z=15;
        deg_pid_disable=1;
    }
    else if(PA0_triggered_time!=8&&PA1_triggered_time==8)
    {
        open_loop_velocity.y=0.4f;
        open_loop_velocity.z=-15;
        deg_pid_disable=1;
    }
    else
    {
        open_loop_velocity.y=0.4f;
        open_loop_velocity.z=0;
    }
    
    if(timer<1000)
        timer++;
    
    if((PA0_triggered_time!=8&&PA1_triggered_time!=8)||Read_Button(27)==1||timer>900)
    {
        Ort old_pos=current_pos;
        open_loop_velocity.y=0.3f;
        flags[auto_drive_status]=current_task->info.z;
        while(flags[hook_pos]!=release||flags[hook_status]==moving)
        {
            osDelay(10);
        }
        PA0_triggered_time=8;
        PA1_triggered_time=8;
        open_loop_velocity.y=0;
        open_loop_velocity.z=0;
        timer=0;
        NNlearn=1;
        if(block_num<7)
        {
            correction_value.x=(8.0f+arm_sin_f32(-current_pos.z*3.1415926f/180.0f)*0.555f)-(float)gyro.x/1000.0f;
            correction_value.y=(6.0f-arm_cos_f32(-current_pos.z*3.1415926f/180.0f)*0.555f)-(float)gyro.y/1000.0f;
        }
        else
        {
            correction_value.x=(4.0f+arm_sin_f32(-current_pos.z*3.1415926f/180.0f)*0.555f)-(float)gyro.x/1000.0f;
            correction_value.y=(6.0f-arm_cos_f32(-current_pos.z*3.1415926f/180.0f)*0.555f)-(float)gyro.y/1000.0f;

        }
        r_correction_value=correction_value;
        correct_brrier(old_pos,current_pos);
        current_task->flag_finish=1;
        
    }
    return 0;
}
PID_T block_deg_pid={.KP=0.1,.KI=0,.KD=0,.PID_MAX=4,.Dead_Zone=0.5f,.I_Limit=15,.I_MAX=70};
PID_T block_dist_pid={.KP=2,.KI=0,.KD=0,.PID_MAX=3,.Dead_Zone=0.05f,.I_Limit=15,.I_MAX=70};
/**********************************************************************************
  *@  name      : fuck_block
  *@  function  : 搭塔流程函数
  *@  input     : 塔块编号
  *@  output    : NULL
  *@  note      : 
  ********************************************************************************/
int fuck_block(mission_queue *current_task)
{
    static uint8_t flag_init=0;
    static float s_distance;
    float distance;
    uint8_t set_flags[total_flags]={0};
    float deg,len;
    static uint32_t clock=0;
    static barrier *target;
    Ort info;
    for (int i = 0; i < total_flags; i++)
    {
        set_flags[i]=either;
    }
    info.x=0;
    info.y=0;
    info.z=0;
    
    if(flag_init==0)
    {
        flag_init=1;
        focus_mode=1;
        target=find_barrier(block_num);
        if(target==NULL||target->location.z==up||target->location.z==down)
        {
            flag_init=0;
            current_task->flag_finish=1;
            clock=0;
            flags[auto_drive_status]=current_task->info.z;
            return 0;
        }
        s_distance=cal_distance(current_pos,target->location);
    }
    distance=cal_distance(current_pos,target->location);
    if(target->deg==361)
    {
        deg=60;
    }
    else
    {
        deg=-target->deg+current_pos.z;
        if(deg>90)
            deg-=180;
        if(deg<-90)
            deg+=180;
    }
    //len=2.2f*(my_sqrt((abs(deg)+3)/60.0f)-0.223606797f);
    //len=0.7f*arm_sin_f32(3.1415926f*fabs(deg)/60.0f);
    len=Pid_Run(&block_deg_pid,0,-fabs(deg))/3.0f;
    dX=judge_sign(deg)*len*arm_cos_f32(current_pos.z*3.1415926f/180.0f);
    dY=-judge_sign(deg)*len*arm_sin_f32(current_pos.z*3.1415926f/180.0f);
    distance=Pid_Run(&block_dist_pid,s_distance,distance);
    dX+=-distance/90.0f*arm_sin_f32(current_pos.z*3.1415926f/180.0f);
    dY+=-distance/90.0f*arm_cos_f32(current_pos.z*3.1415926f/180.0f);
    
    if(clock>1000||Read_Button(27)==1)
    {
        flag_init=0;
        current_task->flag_finish=1;
        clock=0;
        flags[auto_drive_status]=current_task->info.z;
        dX=0;
        dY=0;
        return 0;
    }
    if(fabs(deg)<5)
    {
        flag_init=0;
        focus_mode=0;
        current_task->flag_finish=1;
        flags[auto_drive_status]=current_task->info.z;
        info.x=block_num;
        get_block_flag=1;
        clock=0;
        add_mission(AUTOPICKUP,set_flags,0,&info);
        return 0;
        
    }

    clock++;
    return 0;
}

int auto_pick_up_long(mission_queue *current_task)
{
    static uint8_t flag_init=0;
    static float deg_tmp;
    int target=current_task->info.x;
    barrier *tar=find_barrier(target);
    uint8_t set_flags[20];
    Ort info,point;
    for(int i=0;i<total_flags;i++)
    {
        set_flags[i]=either;
    }
    if(tar==NULL)
    {
        flag_init=0;
        current_task->flag_finish=1;
        return 0;
    }
    if(flag_init==0)
    {
        flag_init=1;
        //if(check_barrier(current_pos,tar->location,0.35f)==0)
        if(1)
        {
            info=evaluate_place_pos(target,0.9f);
            dZ=info.z;
            deg_tmp=dZ;
            info.z=moving_partially_complete2;
            short_drive_deadzone=0.15f;
            add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&info);
        }
        else
        {
            info=dynamic_path_planning(current_pos,tar->location,check_barrier(current_pos,tar->location,0.2f))->pos;
            info.z=moving_partially_complete1;
            short_drive_deadzone=0.15f;
            add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&info);
            info=evaluate_approach_pos(info,target,1.2f);
            dZ=info.z;
            deg_tmp=dZ;
            set_flags[auto_drive_status]=moving_partially_complete1;
            info.z=moving_partially_complete2;
            short_drive_deadzone=0.15f;
            add_mission(AUTODRIVESHORTDISTANCE,set_flags,0,&info);
        }
        
    }

    dZ=-atan2f(tar->location.x-current_pos.x,tar->location.y-current_pos.y)*180.0f/3.1415926f;
    //dZ=deg_tmp;

    if(cal_distance(tar->location,current_pos)<1.2f||flags[auto_drive_status]==moving_partially_complete2)
    {
        tar=find_barrier(target);
        if(tar!=NULL)
        {
            if(tar->location.z==up||tar->location.z==down)
            {
                info.x=block_num;
                get_block_flag=1;
                dZ=-atan2f(tar->location.x-current_pos.x,tar->location.y-current_pos.y)*180.0f/3.1415926f;
                set_flags[deg_offset]=30;
                add_mission(AUTOPICKUP,set_flags,0,&info);
                focus_mode=0;
            }
            else
            {
                set_flags[deg_offset]=5;
                dZ=-atan2f(tar->location.x-current_pos.x,tar->location.y-current_pos.y)*180.0f/3.1415926f;
                info.z=moving_complete1;
                add_mission(FUCKBLOCK,set_flags,1,&info);
            }
        }
        flag_init=0;
        current_task->flag_finish=1;
    }
    return 0;
}

/*********************************************************************************
  *@  name      : move_forward2
  *@  function  : 夹块流程函数
  *@  input     : 塔块编号
  *@  output    : NULL
  *@  note      : 
*********************************************************************************/
int move_forward2(mission_queue *current_task)
{
    static int timer=0;
    NNlearn=0;
    if(timer<1000)
        timer++;
    open_loop_velocity.y=0.35f;
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)||Read_Button(27)==1||timer>900)
    {
        open_loop_velocity.y=0.1f;
        flags[auto_drive_status]=current_task->info.z;
        while((flags[hook_pos]!=grasp||flags[hook_status]==moving)&&Read_Button(27)==0)
        {
            osDelay(10);
        }
        open_loop_velocity.y=0;
        timer=0;
        NNlearn=1;
        current_task->flag_finish=1;
    }
    return 0;
}

void fdcan_add_msg_2_queue(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t StdId, uint32_t Length)
{
    xSemaphoreTake(fdcan_queue_mutex,5);
    fdcan_msg_queue *tmp=malloc(sizeof(fdcan_msg_queue));
    fdcan_msg_queue *tmp2=fdcan_msg_queue_head;
    if(tmp==NULL)
        Error_Handler();
    tmp->fdcan=hfdcan;
    tmp->ID=StdId;
    tmp->next=NULL;
    tmp->len=Length;
    for (int i = 0; i < Length; i++)
    {
        tmp->msg[i]=TxData[i];
    }
    if(fdcan_msg_queue_head==NULL)
    {
        fdcan_msg_queue_head=tmp;       
    }
    else
    {
        while (tmp2->next!=NULL)    
        {
            tmp2=tmp2->next;
        }
        tmp2->next=tmp;
    }
    xSemaphoreGive(fdcan_queue_mutex);
    return;
}
