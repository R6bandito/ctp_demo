/**
 * @file    stm32f1_rtos_demo.h
 * @brief   EasyCantp — FreeRTOS Multi-Thread Demo Header
 * @author  R6bandito
 * @date    2026-08
 *
 * Platform: STM32F103XE + FreeRTOS + CAN1 loopback
 * Prerequisites:
 *   1. FreeRTOS configured with > 2 tasks, queue support enabled
 *   2. ctp_os_port_freertos.c compiled into project
 *   3. CAN hardware + NVIC + SysTick hooks set up (same as bare-metal demo)
 *   4. configASSERT enabled for debug builds
 *
 * Quick start from main():
 *   rtos_demo_init();          // sets up CAN + CTP + OS layer + TP kernel thread
 *   rtos_demo_run();           // spawns sender tasks, waits for completion
 */
#ifndef __STM32F1_RTOS_DEMO_H__
#define __STM32F1_RTOS_DEMO_H__


#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ctp.h"
#include "ctp_os.h"


void rtos_demo_init( void );
void rtos_demo_run( void );
void rtos_demo_shared_conn( void );


#endif /* __STM32F1_RTOS_DEMO_H__ */
