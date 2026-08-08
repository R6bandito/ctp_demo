/**
 * @file    ctp_os_port_freertos.c
 * @brief   CAN TP OS Abstraction Layer — FreeRTOS Port
 * @author  R6bandito
 * @date    2026-08
 *
 * Provides concrete OS primitives required by @ref ctp_os.h, mapped onto
 * the FreeRTOS API.
 *
 *
 * API mapping:
 *
 * | Abstract function              | FreeRTOS primitive            |
 * |------------------------------- |-------------------------------|
 * | Cus_Cantp_OS_ThreadNew()       | xTaskCreate()                 |
 * | Cus_Cantp_OS_MailboxCreate()   | xQueueCreate()                |
 * | Cus_Cantp_OS_MailboxSend()     | xQueueSendToBack()            |
 * | Cus_Cantp_OS_MailboxFetch()    | xQueueReceive()               |
 * | Cus_Cantp_OS_Delay()           | vTaskDelay()                  |
 * | Cus_Cantp_OS_EnterCritical()   | taskENTER_CRITICAL()          |
 * | Cus_Cantp_OS_ExitCritical()    | taskEXIT_CRITICAL()           |
 *
 */

#include "ctp_os.h"


#if defined(CUS_CANTP_OS)

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

TaskHandle_t g_ctpKernel;

/* *********************************************************************** */
int8_t 
Cus_Cantp_OS_ThreadNew( Cus_CANTP_Thread_t pThread, void *pArg, uint32_t stackSize, uint32_t priority )
{
	BaseType_t Return = xTaskCreate( (TaskFunction_t)pThread, "ctp_kernel", stackSize, pArg, priority, &g_ctpKernel );
	if ( Return != pdPASS )
		return -1;

	return 0;
}


int8_t 
Cus_Cantp_OS_MailboxCreate( uint32_t itemCount, uint32_t itemSize, void **pBox )
{
	*pBox = xQueueCreate( itemCount, itemSize );
	if ( !(*pBox) )
		return -1;

	return 0;
}


int8_t 
Cus_Cantp_OS_MailboxSend( void *mailBox, const void *item, uint32_t timeoutMs )
{
	TickType_t ticks = (timeoutMs == 0xFFFFFFFF) ? portMAX_DELAY: pdMS_TO_TICKS( timeoutMs );
	BaseType_t Return = xQueueSendToBack( (QueueHandle_t)mailBox, item, ticks );
	if ( Return != pdTRUE )
		return -1;

	return 0;
}


int8_t 
Cus_Cantp_OS_MailboxFetch( void *mailBox, void *item, uint32_t timeoutTicks )
{
	/* NOTE: internal use only (kernel thread).  The timeout is interpreted
	 * directly in ticks, NOT milliseconds — the kernel thread's MainFunction
	 * driving period therefore scales with configTICK_RATE_HZ: raising the
	 * tick rate compresses the CF frame interval. */
	TickType_t ticks = (timeoutTicks == 0xFFFFFFFF) ? portMAX_DELAY: (TickType_t)timeoutTicks;
	BaseType_t Return = xQueueReceive( (QueueHandle_t)mailBox, item, ticks );
	if ( Return != pdTRUE )
		return -1;

	return 0;
}


void 
Cus_Cantp_OS_Delay( uint32_t delayMs )
{
	TickType_t ticks = (delayMs == 0xFFFFFFFF) ? portMAX_DELAY: pdMS_TO_TICKS( delayMs );
	vTaskDelay( ticks );
}


void 
Cus_Cantp_OS_EnterCritical( void )
{
	taskENTER_CRITICAL();
}


void 
Cus_Cantp_OS_ExitCritical( void )
{
	taskEXIT_CRITICAL();
}

#endif /* CUS_CANTP_OS */
