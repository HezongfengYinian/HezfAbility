/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f2xx.h"
#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_COMPONENTS_INIT
#include "components.h"
#endif
#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#include "net_app.h"
#include "bootcfg.h"
#include "led_indicator.h"
#include "3g_watchdog.h"

extern void rt_platform_init(void);
extern int dfs_mkfs(const char *fs_name, const char *device_name);

void init_spi_flash_filesystem()
{
    /* mount SPI flash as root directory */
    if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("flash0 mount to /\n");
    }
    else
    {
        if (dfs_mkfs("elm", "flash0") != 0)
            rt_kprintf("mkfs error\n");
        else
            rt_kprintf("mkfs ok\n");

        if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("flash0 mount to /\n");
        }
        else
            rt_kprintf("flash0 mount to / failed.\n");
    }
}

void rt_init_thread_entry(void *parameter)
{
    rt_platform_init();

#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

    init_spi_flash_filesystem();

    /* do some thing here. */
    bootsh_init();
    bootsh_set_device(RT_CONSOLE_DEVICE_NAME);

    /* 启动硬件看门狗喂狗定时器 */
    startup_hw_watchdog_feeding_timer();

    /* 初始化系统状态指示灯*/
    init_sys_status_indicator_lamps();
    startup_sys_running_state_lamp_timer();
    startup_iwdg_feeding_timer();

    rt_sem_init(&sys_option_sem, "opsem", 0, RT_IPC_FLAG_FIFO);
/*
    if (!start_boot_work())
    {
        rt_hw_board_reboot();
    }

    rt_sem_take(&sys_option_sem, RT_WAITING_FOREVER);

    if (!start_net_work())
    {
        rt_hw_board_reboot();
    }
*/
}

int rt_application_init()
{
    rt_thread_t init_thread;

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 10, 50);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
