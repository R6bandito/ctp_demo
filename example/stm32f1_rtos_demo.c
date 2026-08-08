/**
 * @file    stm32f1_rtos_demo.c
 * @brief   EasyCantp — FreeRTOS Multi-Thread Demo
 * @author  R6bandito
 * @date    2026-08
 *
 * Verifies the OS safety layer by running two sender tasks that
 * concurrently submit CAN TP transmissions through the mailbox
 * to the single TP kernel thread.
 *
 * ── Architecture ──
 *   Task "SenderA" ─┐
 *                   ├─ Mailbox ─► TP Kernel Thread (Cus_Cantp_MainFunction)
 *   Task "SenderB" ─┘
 *
 * SenderA transmits a 35-byte First Frame (multi-frame).
 * SenderB transmits a 7-byte Single Frame on a different TA.
 * Both completions are tracked via DataInd callbacks.
 *
 * ── Expected console output ──
 *   --- RTOS MULTI-TASK DEMO ---
 *   SenderA FF start: OK
 *   SenderB SF start: OK
 *   DataInds: 2/2 PASS
 *
 * ── ISR wiring (same as bare-metal demo) ──
 *   SysTick_Handler           → Cus_Cantp_TimerTickInc()
 *   HAL_CAN_RxFifo0MsgPending → Cus_Cantp_FeedFrame()
 *   HAL_CAN_TxMailboxComplete  → Cus_Cantp_TxConfirm()
 */
#include "stm32f1_rtos_demo.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>


/*============================================================================
 * Platform setup (identical to bare-metal loopback demo)
 *============================================================================*/
#define DEMO_CAN_PORT              (GPIOA)
#define DEMO_CAN_TX                (GPIO_PIN_12)
#define DEMO_CAN_RX                (GPIO_PIN_11)
#define DEMO_CAN_CLK_EN()          do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while(0)

CAN_HandleTypeDef  hcan;

static int8_t demoSend( void *ctx, uint32_t canId, const uint8_t *data, uint8_t dlc );
static void demoErr( void *conn, Cus_CANTP_ErrCode_t err );


/*============================================================================
 * Shared test state
 *============================================================================*/
static volatile uint8_t g_done_count;
static SemaphoreHandle_t g_done_sem;
static const uint8_t g_sf_data[7] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0x00 };
static uint8_t       g_ff_data[35];       /* filled at init */


/* ── DataInd callback shared by all RxConns ── */
static void demoRxCb( void *rxConn, const uint8_t *data, uint32_t len )
{
    (void)rxConn; (void)data; (void)len;

    taskENTER_CRITICAL();
    g_done_count++;
    taskEXIT_CRITICAL();

    /* DataInd runs in Phase 2 (task context, not ISR) */
    xSemaphoreGive( g_done_sem );
}


/*============================================================================
 * CAN send callback — mailbox write (runs inside TP thread → critical section)
 *============================================================================*/
static int8_t demoSend( void *ctx, uint32_t canId, const uint8_t *data, uint8_t dlc )
{
    (void)ctx;
    CAN_TxHeaderTypeDef header = {
        .DLC    = dlc,
        .IDE    = CAN_ID_STD,
        .RTR    = CAN_RTR_DATA,
        .StdId  = canId
    };
    uint32_t mailBox = 0;

    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS( 50 );
    while ( HAL_CAN_AddTxMessage( &hcan, &header, (uint8_t *)data, &mailBox ) != HAL_OK )
    {
        if ( (int32_t)( xTaskGetTickCount() - deadline ) >= 0 )
            return -1;            /* mailbox stuck — give up */
    }
    return (int8_t)mailBox;
}


/*============================================================================
 * Error callback
 *============================================================================*/
static void demoErr( void *conn, Cus_CANTP_ErrCode_t err )
{
    (void)conn;
    printf( "[ERR] code=%d\n", (int)err );
}


/*============================================================================
 * CAN hardware init (loopback mode)
 *============================================================================*/
static void rtos_can_init( void )
{
    /* GPIO */
    DEMO_CAN_CLK_EN();
    GPIO_InitTypeDef gpio = { .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL,
                               .Speed = GPIO_SPEED_FREQ_HIGH };

    gpio.Pin = DEMO_CAN_TX;
    HAL_GPIO_Init( DEMO_CAN_PORT, &gpio );
    gpio.Mode = GPIO_MODE_AF_INPUT;
    gpio.Pin  = DEMO_CAN_RX;
    HAL_GPIO_Init( DEMO_CAN_PORT, &gpio );

    /* CAN peripheral */
    __HAL_RCC_CAN1_CLK_ENABLE();
    hcan.Instance = CAN1;
    hcan.Init.AutoBusOff           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.Mode                 = CAN_MODE_LOOPBACK;
    hcan.Init.Prescaler            = 9;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.SyncJumpWidth        = CAN_SJW_2TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_5TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.TransmitFifoPriority = ENABLE;
    HAL_CAN_Init( &hcan );

    /* Filter — accept all */
    CAN_FilterTypeDef fcan = {
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterBank           = 0,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterIdHigh         = 0, .FilterIdLow     = 0,
        .FilterMaskIdHigh     = 0, .FilterMaskIdLow = 0,
        .FilterActivation     = CAN_FILTER_ENABLE
    };
    HAL_CAN_ConfigFilter( &hcan, &fcan );
    HAL_CAN_Start( &hcan );

    HAL_CAN_ActivateNotification( &hcan, CAN_IT_RX_FIFO0_MSG_PENDING
                                        | CAN_IT_RX_FIFO1_MSG_PENDING
                                        | CAN_IT_TX_MAILBOX_EMPTY );

    HAL_NVIC_SetPriority( CAN1_RX0_IRQn, 6, 0 );
    HAL_NVIC_SetPriority( CAN1_RX1_IRQn, 6, 0 );
    HAL_NVIC_SetPriority( CAN1_TX_IRQn,  6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX0_IRQn );
    HAL_NVIC_EnableIRQ( CAN1_RX1_IRQn );
    HAL_NVIC_EnableIRQ( CAN1_TX_IRQn );
}


/*============================================================================
 * Sender Task A — FF 35 bytes on TA=0x03
 *============================================================================*/
static void TaskSenderA( void *arg )
{
    (void)arg;

    const Cus_CANTP_ChannelCfg_t chTx = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL, .SA = 0x01, .TA = 0x03,
        .TAType = CUS_CANTP_TA_TYPE_PHYSICAL, .fSize = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_ChannelCfg_t chRx = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL, .SA = 0x00, .TA = 0x03,
        .TAType = CUS_CANTP_TA_TYPE_PHYSICAL, .fSize = CUS_CANTP_SIZE_8, .funcId = 0
    };

    /* create conns through the OS-safe API (synchronous — blocks until TP
     * thread processes the request) */
    const Cus_CANTP_TxConn_t *tx = NULL;
    const Cus_CANTP_RxConn_t *rx = NULL;

    static uint8_t rx_buf_a[256];
    Cus_Cantp_OS_CreateTxConn( chTx, (void *)CAN1, NULL, demoSend, demoErr,
                                5000, &tx );

    Cus_Cantp_OS_CreateRxConn_Bind( chRx, (void *)CAN1, NULL, demoSend, demoErr,
                                     demoRxCb, rx_buf_a, sizeof(rx_buf_a),
                                     0, 0, 5000, &rx );

    /* submit FF send */
    int8_t r = Cus_Cantp_OS_StartTransmit( (Cus_CANTP_TxConn_t *)tx,
                                            g_ff_data, sizeof(g_ff_data), 5000 );
    printf( "  SenderA (FF %uB): %s\n", (unsigned)sizeof(g_ff_data),
            (r == 0) ? "OK" : "FAIL" );

    /* cleanup is handled by demo teardown; just suspend */
    vTaskSuspend( NULL );
}


/*============================================================================
 * Sender Task B — SF 7 bytes on TA=0x04
 *
 * Small delay before sending → its SF overlaps with Task A's FF
 * multi-frame sequence inside the TP thread.
 *============================================================================*/
static void TaskSenderB( void *arg )
{
    (void)arg;

    /* let Task A get its FF out first */
    vTaskDelay( pdMS_TO_TICKS( 10 ) );

    const Cus_CANTP_ChannelCfg_t chTx = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL, .SA = 0x02, .TA = 0x04,
        .TAType = CUS_CANTP_TA_TYPE_PHYSICAL, .fSize = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_ChannelCfg_t chRx = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL, .SA = 0x00, .TA = 0x04,
        .TAType = CUS_CANTP_TA_TYPE_PHYSICAL, .fSize = CUS_CANTP_SIZE_8, .funcId = 0
    };

    const Cus_CANTP_TxConn_t *tx = NULL;
    const Cus_CANTP_RxConn_t *rx = NULL;

    static uint8_t rx_buf_b[256];
    Cus_Cantp_OS_CreateTxConn( chTx, (void *)CAN1, NULL, demoSend, demoErr,
                                5000, &tx );

    Cus_Cantp_OS_CreateRxConn_Bind( chRx, (void *)CAN1, NULL, demoSend, demoErr,
                                     demoRxCb, rx_buf_b, sizeof(rx_buf_b),
                                     0, 0, 5000, &rx );

    int8_t r = Cus_Cantp_OS_StartTransmit( (Cus_CANTP_TxConn_t *)tx,
                                            g_sf_data, sizeof(g_sf_data), 5000 );
    printf( "  SenderB (SF %uB): %s\n", (unsigned)sizeof(g_sf_data),
            (r == 0) ? "OK" : "FAIL" );

    vTaskSuspend( NULL );
}


/*============================================================================
 * Watcher task — waits for both DataInd callbacks, then prints verdict
 *============================================================================*/
static void TaskWatcher( void *arg )
{
    (void)arg;

    /* wait until two DataInds fire or 10 s timeout */
    for ( uint32_t i = 0; i < 20; i++ )
    {
        if ( xSemaphoreTake( g_done_sem, pdMS_TO_TICKS( 500 ) ) != pdTRUE )
            break;                                    /* timeout */
        if ( g_done_count >= 2 )
            break;
    }

    printf( "  DataInds fired: %u  %s\n",
            (unsigned)g_done_count,
            (g_done_count == 2) ? "PASS" : "FAIL" );

    vTaskSuspend( NULL );
}


/*============================================================================
 * Shared-Conn Priority Test helpers
 *============================================================================*/

static SemaphoreHandle_t   g_shared_done_sem;
static volatile uint8_t    g_shared_done_count;
static uint8_t             g_shared_rx_buf[256];
static const Cus_CANTP_RxConn_t *g_shared_rx;
static const Cus_CANTP_TxConn_t *g_shared_tx;


/* ── callback for shared-conn test ── */
static void sharedRxCb( void *rxConn, const uint8_t *data, uint32_t len )
{
    (void)rxConn; (void)data; (void)len;

    taskENTER_CRITICAL();
    g_shared_done_count++;
    taskEXIT_CRITICAL();

    xSemaphoreGive( g_shared_done_sem );
}


/* ── High-priority sender (prio=3) ── */
static void TaskSharedHi( void *arg )
{
    (void)arg;

    /* wait for the shared conn to be free (initially available) */
    uint8_t data[6] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    printf( "  [HI ] submitting (SF %uB)...\n", (unsigned)sizeof(data) );
    int8_t r = Cus_Cantp_OS_StartTransmit( (Cus_CANTP_TxConn_t *)g_shared_tx,
                                            data, sizeof(data), 5000 );
    printf( "  [HI ] submit %s\n", (r == 0) ? "OK" : "FAIL" );

    /* wait for our transmission to complete */
    uint8_t target = g_shared_done_count + 1;
    while ( g_shared_done_count < target )
        vTaskDelay( pdMS_TO_TICKS( 1 ) );

    printf( "  [HI ] done (total=%u)\n", (unsigned)g_shared_done_count );

    /* yield to mid-priority task */
    vTaskSuspend( NULL );
}


/* ── Mid-priority sender (prio=2) ── */
static void TaskSharedMid( void *arg )
{
    (void)arg;

    /* HI must run first (higher prio); small delay to guarantee ordering */
    vTaskDelay( pdMS_TO_TICKS( 5 ) );

    uint8_t data[6] = { 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB };

    printf( "  [MID] submitting (SF %uB)...\n", (unsigned)sizeof(data) );
    int8_t r = Cus_Cantp_OS_StartTransmit( (Cus_CANTP_TxConn_t *)g_shared_tx,
                                            data, sizeof(data), 5000 );
    printf( "  [MID] submit %s\n", (r == 0) ? "OK" : "FAIL" );

    uint8_t target = g_shared_done_count + 1;
    while ( g_shared_done_count < target )
        vTaskDelay( pdMS_TO_TICKS( 1 ) );

    printf( "  [MID] done (total=%u)\n", (unsigned)g_shared_done_count );

    vTaskSuspend( NULL );
}


/* ── Low-priority sender (prio=1) ── */
static void TaskSharedLo( void *arg )
{
    (void)arg;

    /* HI and MID must have finished before we run */
    vTaskDelay( pdMS_TO_TICKS( 10 ) );

    uint8_t data[6] = { 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };

    printf( "  [LO ] submitting (SF %uB)...\n", (unsigned)sizeof(data) );
    int8_t r = Cus_Cantp_OS_StartTransmit( (Cus_CANTP_TxConn_t *)g_shared_tx,
                                            data, sizeof(data), 5000 );
    printf( "  [LO ] submit %s\n", (r == 0) ? "OK" : "FAIL" );

    uint8_t target = g_shared_done_count + 1;
    while ( g_shared_done_count < target )
        vTaskDelay( pdMS_TO_TICKS( 1 ) );

    printf( "  [LO ] done (total=%u)\n", (unsigned)g_shared_done_count );

    /* signal the shared watcher that all three are done */
    xSemaphoreGive( g_shared_done_sem );

    vTaskSuspend( NULL );
}


/* ── Watcher for shared-conn test ── */
static void TaskSharedWatch( void *arg )
{
    (void)arg;

    /* poll until 3 DataInds or 10 s timeout; already got one semGive from LO */
    for ( uint32_t i = 0; i < 25; i++ )
    {
        if ( xSemaphoreTake( g_shared_done_sem, pdMS_TO_TICKS( 500 ) ) != pdTRUE )
            break;
        if ( g_shared_done_count >= 3 )
            break;
    }

    printf( "  Shared-conn DataInds: %u/3  %s\n",
            (unsigned)g_shared_done_count,
            (g_shared_done_count == 3) ? "PASS" : "FAIL" );

    vTaskSuspend( NULL );
}


/*============================================================================
 * Public API
 *============================================================================*/

void rtos_demo_init( void )
{
    /* fill FF test pattern */
    for ( uint8_t i = 0; i < sizeof(g_ff_data); i++ )
        g_ff_data[i] = i;

    /* shared semaphore for callback → watcher signalling */
    g_done_sem = xSemaphoreCreateBinary();
    configASSERT( g_done_sem );

    /* hardware + protocol core */
    rtos_can_init();
    Cus_Cantp_ConnPoolInit();
    Cus_Cantp_TimerInit();

    /* OS layer — creates mailbox + TP kernel thread */
    Cus_CANTP_OS_Init();
}


void rtos_demo_run( void )
{
    printf( "\n--- RTOS MULTI-TASK DEMO (FF 35B + SF 7B) ---\n" );

    g_done_count = 0;

    /* spawn two sender tasks + one watcher */
    xTaskCreate( TaskSenderA, "SndA", 512, NULL, 2, NULL );
    xTaskCreate( TaskSenderB, "SndB", 512, NULL, 2, NULL );
    xTaskCreate( TaskWatcher, "Watch", 256, NULL, 1, NULL );
}


/* ── Shared-conn init task (runs after scheduler started) ── */
static void TaskSharedInit( void *arg )
{
    (void)arg;

    printf( "\n--- RTOS SHARED-CONN PRIORITY DEMO (same conn, 3 threads) ---\n" );

    g_shared_done_count = 0;

    /* create semaphore for callback → watcher signalling */
    g_shared_done_sem = xSemaphoreCreateBinary();
    configASSERT( g_shared_done_sem );

    /* create ONE shared Rx + Tx conn pair (TA=0x05) */
    const Cus_CANTP_ChannelCfg_t chRx = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL, .SA = 0x00, .TA = 0x05,
        .TAType = CUS_CANTP_TA_TYPE_PHYSICAL, .fSize = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_ChannelCfg_t chTx = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL, .SA = 0x01, .TA = 0x05,
        .TAType = CUS_CANTP_TA_TYPE_PHYSICAL, .fSize = CUS_CANTP_SIZE_8, .funcId = 0
    };

    Cus_Cantp_OS_CreateRxConn_Bind( chRx, (void *)CAN1, NULL, demoSend, demoErr,
                                     sharedRxCb, g_shared_rx_buf,
                                     sizeof(g_shared_rx_buf),
                                     0, 0, 5000, &g_shared_rx );

    Cus_Cantp_OS_CreateTxConn( chTx, (void *)CAN1, NULL, demoSend, demoErr,
                                5000, &g_shared_tx );

    /*
     * Spawn three sender threads + watcher.
     * HI (prio=3) → MID (prio=2) → LO (prio=1).
     */
    xTaskCreate( TaskSharedHi,    "ShHi",  512, NULL, 3, NULL );
    xTaskCreate( TaskSharedMid,   "ShMid", 512, NULL, 2, NULL );
    xTaskCreate( TaskSharedLo,    "ShLo",  512, NULL, 1, NULL );
    xTaskCreate( TaskSharedWatch, "ShWtch", 256, NULL, 1, NULL );

    /* Init done — self-delete. */
    vTaskDelete( NULL );
}

void rtos_demo_shared_conn( void )
{
    /* Defer to a task so OS APIs see a running scheduler. */
    xTaskCreate( TaskSharedInit, "ShInit", 512, NULL,
                 configMAX_PRIORITIES - 1, NULL );
}
