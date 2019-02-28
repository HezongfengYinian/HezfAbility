#include "mbus_app.h"

#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "stm32f2xx.h"
#include "usart.h"

#include "commontypes.h"
#include "mbusproto.h"
#include "cmd_type.h"
#include "3g_protocol.h"
#include "crc.h"
#include "devrange.h"

#include "msg_center.h"
#include "bootcfg.h"
#include "net_app.h"
#include "led_indicator.h"
#include "3g_thread.h"
#include "../../../../../version.h"

//#define LOG_DEBUG
#include "3g_log.h"
#include "datamanage.h"
#include "comm_def.h"
#include "lcd_spi.h"
#include "stm32_flash.h"
/*
** ip网络收发缓冲区
*/

struct rt_messagequeue mbus_rec_mq;

/*
 * 上报工控机数据消息队列定义
 */

struct rt_messagequeue mbus_reported_mq;


#define SENSOR_POOL_MAX   10240  			 // size of sensor message dequeue
struct rt_messagequeue sensor_mq;
static  rt_uint8_t sensor_mq_pool[SENSOR_POOL_MAX];

#define COORDINATED_POOL_MAX   10240  			 // size of sensor message dequeue
struct rt_messagequeue coordinated_mq;
static  rt_uint8_t coordinated_mq_pool[COORDINATED_POOL_MAX];


#define VERSION_REPORT_INTERVAL_SEC     (3600)


void init_mbus_msg_queue(void)
{

}



void sensor_thread_entry(void * param)
{
	rt_kprintf("sensor_thread_entry\n");	
	rt_mq_init(&sensor_mq, "SENSONMQ", sensor_mq_pool, MSG_MINITOR_SIZE,
               sizeof(sensor_mq_pool), RT_IPC_FLAG_FIFO);
	LCD_Initial();  
	gpio_test();
	Test();
	while(1)
	{		
		LCD_Show();
		feed_watchdog();
		time_show();
		station_status_monitoring();
		rt_thread_delay(100);
	}
}

void coordinate_thread_entry(void * param)
{
	rt_kprintf("coordinated_thread_entry\n");	
	
	rt_mq_init(&coordinated_mq, "COORDINATEDMQ", coordinated_mq_pool, MSG_MINITOR_SIZE,
               sizeof(coordinated_mq_pool), RT_IPC_FLAG_FIFO);
		
	while(1)
	{
		coordinated_recv();
		feed_watchdog();
		rt_thread_delay(800);
	}
}
static rt_thread_t sensor_thread = RT_NULL;
static rt_thread_t coordinated_thread = RT_NULL;

rt_bool_t start_modbus_work()
{
    sensor_thread = rt_thread_create("sensor", sensor_thread_entry,
                                    RT_NULL, 2048, 9, 10);
	
    coordinated_thread = rt_thread_create("coordinate", coordinate_thread_entry,
                                    RT_NULL, 2048, 9, 10);

    if (sensor_thread == RT_NULL)
    {
        ERROR_LOG("create modbus work sensor thread failed\n");
        return RT_FALSE;
    }

    if (coordinated_thread == RT_NULL)
    {
        ERROR_LOG("create modbus work coordinated thread failed\n");
        return RT_FALSE;
    }

    rt_thread_startup(sensor_thread);
    rt_thread_startup(coordinated_thread);

    return RT_TRUE;
}

void stop_modbus_work()
{
    rt_enter_critical();

    if (sensor_thread != RT_NULL && sensor_thread->stat != RT_THREAD_CLOSE)
    {
        rt_thread_delete(sensor_thread);
    }

    if (coordinated_thread != RT_NULL && coordinated_thread->stat != RT_THREAD_CLOSE)
    {
        rt_thread_delete(coordinated_thread);
    }
    rt_exit_critical();

    TIME_LOG(LOG_CRITICAL, "stop modbus thread\n");
}

