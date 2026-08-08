/**
 * @file    stm32f1_loopback_demo.c
 * @brief   EasyCantp — STM32F1 CAN Loopback Demo (HAL only, no CANCUS)
 * @author  R6bandito
 * @date    2026-08
 *
 * This file demonstrates how to integrate EasyCantp with STM32 HAL CAN
 * without depending on the CANCUS library. All test cases run in CAN
 * loopback mode — self-send / self-receive, zero external hardware.
 *
 * Quick start:
 *   1. Call test_env_init() from main() to set up CAN + CTP
 *   2. Call test_run_all() or individual test_xxx() functions
 *   3. Callbacks auto-validate data and print PASS/FAIL over UART
 *
 * Interrupt wiring required in stm32f1xx_it.c:
 *   SysTick_Handler          → Cus_Cantp_TimerTickInc()   (1 ms tick)
 *   CAN1_RX0_IRQHandler      → HAL_CAN_IRQHandler(&hcan)
 *   CAN1_TX_IRQHandler       → HAL_CAN_IRQHandler(&hcan)
 *   RxFifo0MsgPendingCallback → Cus_Cantp_FeedFrame()
 *   TxMailboxNCompleteCallback → Cus_Cantp_TxConfirm()
 *
 * Notes:
 *   - hcan is global (non-static), referenced via extern by stm32f1xx_it.c
 *   - TimeSeg1 / TimeSeg2 must use HAL macros (CAN_BS1_xTQ), never raw numbers
 *   - TransmitFifoPriority = ENABLE avoids CAN-ID arbitration reordering
 *     across multiple concurrent connections
 */
#include "stm32f1_loopback_demo.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "ctp.h"

CAN_HandleTypeDef hcan;
static CAN_FilterTypeDef fcan;

static const uint8_t *g_expect_data;
static uint32_t g_expect_len;
static uint8_t g_isOK;               /* set to 1 by dataReceiveCallBack on PASS */

static uint8_t rx_buf[4096];

/* **************************************************** */
static int8_t sendCallBack( void *ctx, uint32_t canId, const uint8_t *data, uint8_t dlc ); 
static void dataReceiveCallBack( void *rxConn, const uint8_t *data, uint32_t len );
static void errorCallBack( void *conn, Cus_CANTP_ErrCode_t err ); 
static void demo_deviceInit( void );
/* **************************************************** */


static void 
print( uint8_t code )
{
    printf( "[ERROR] Code: %d\n", code );
}


static void 
demo_setExpectData( const uint8_t *data, uint32_t len )
{
    g_expect_data = data;
    g_expect_len  = len;
}


static void 
demo_deviceInit( void )
{
    {
        /* GPIO Init. */
        DEMO_CAN_CLK_EN();

        GPIO_InitTypeDef gpio_initstructure;
        gpio_initstructure.Mode     = GPIO_MODE_AF_PP;
        gpio_initstructure.Pin      = DEMO_CAN_TX;
        gpio_initstructure.Pull     = GPIO_NOPULL;
        gpio_initstructure.Speed    = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init( DEMO_CAN_PORT, &gpio_initstructure );

        gpio_initstructure.Mode     = GPIO_MODE_AF_INPUT;
        gpio_initstructure.Pin      = DEMO_CAN_RX;
        HAL_GPIO_Init( DEMO_CAN_PORT, &gpio_initstructure );
    }

    /* CAN device config. */
    __HAL_RCC_CAN1_CLK_ENABLE();
    hcan.Instance = CAN1;
    hcan.Init.AutoBusOff            = DISABLE;
    hcan.Init.AutoRetransmission    = ENABLE;
    hcan.Init.AutoWakeUp            = DISABLE;
    hcan.Init.Mode                  = CAN_MODE_LOOPBACK;
    hcan.Init.Prescaler             = 9;
    hcan.Init.ReceiveFifoLocked     = DISABLE;
    hcan.Init.SyncJumpWidth         = CAN_SJW_2TQ;
    hcan.Init.TimeSeg1              = CAN_BS1_5TQ;
    hcan.Init.TimeSeg2              = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode     = DISABLE;
    hcan.Init.TransmitFifoPriority  = ENABLE;   /* for Cantp, this mode is better. */
    HAL_CAN_Init( &hcan );

    fcan.FilterMode                 = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale                = CAN_FILTERSCALE_32BIT;
    fcan.FilterBank                 = 0;
    fcan.FilterFIFOAssignment       = CAN_FILTER_FIFO0;
    fcan.FilterIdHigh               = 0;
    fcan.FilterIdLow                = 0;
    fcan.FilterMaskIdHigh           = 0;
    fcan.FilterMaskIdLow            = 0;
    fcan.FilterActivation           = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter( &hcan, &fcan );

    HAL_CAN_Start( &hcan );

    HAL_CAN_ActivateNotification( &hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY );

    HAL_NVIC_SetPriority( CAN1_RX0_IRQn, 6, 0 );
    HAL_NVIC_SetPriority( CAN1_RX1_IRQn, 6, 0 );
    HAL_NVIC_SetPriority( CAN1_TX_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX0_IRQn );
    HAL_NVIC_EnableIRQ( CAN1_RX1_IRQn );
    HAL_NVIC_EnableIRQ( CAN1_TX_IRQn );
}


void 
test_env_init( void )
{
    demo_deviceInit();
    Cus_Cantp_ConnPoolInit();
    Cus_Cantp_TimerInit();
}


static int8_t 
sendCallBack( void *ctx, uint32_t canId, const uint8_t *data, uint8_t dlc )
{
    CAN_TxHeaderTypeDef header;
	header.DLC = dlc;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.StdId = canId;

    uint32_t mailBox = 0;
    
    /* Blocking send with 50-ms retry: spin-waits an empty mailbox
     * instead of returning -1 and aborting the transmission. */
    uint32_t timeout = Cus_Cantp_TimerNow();
    while ( (HAL_CAN_AddTxMessage( &hcan, &header, (uint8_t *)data, &mailBox ) != HAL_OK) 
                && (Cus_Cantp_TimerNow() - timeout) < 50 )
    {
        /* Tx request failed. Mailbox full. Wait and retry. */
        __nop();
        __nop();
        __nop();
    }
    
    return mailBox;
}


static void 
dataReceiveCallBack( void *rxConn, const uint8_t *data, uint32_t len )
{
    Cus_CANTP_RxConn_t *rx = (Cus_CANTP_RxConn_t *)rxConn;
    const char *mode = (rx->channel.addrMode == CUS_CANTP_ADDR_MODE_NORMAL) ? "NORMAL" : "EXT";
    uint8_t sf_max = (rx->channel.addrMode == CUS_CANTP_ADDR_MODE_NORMAL) ? 7 : 6;
    const char *type   = (len <= sf_max) ? "SF" : "FF";

    bool pass = (g_expect_data != NULL) &&
                (len == g_expect_len) &&
                (memcmp( data, g_expect_data, len ) == 0);

    printf( "[%s %s] RX %luB  %s", mode, type, len, pass ? "PASS" : "FAIL" );

    uint32_t n = (len <= 32) ? len : 16;
    printf( "  [" );
    for ( uint32_t i = 0; i < n; i++ ) printf( " %02X", data[i] );
    if ( len > 16 ) printf( " ..." );
    printf( " ]\n" );

    if ( !pass && g_expect_data )
    {
        printf( "      exp: [" );
        for ( uint32_t i = 0; i < n; i++ ) printf( " %02X", g_expect_data[i] );
        if ( g_expect_len > 16 ) printf( " ..." );
        printf( " ]\n" );
    }

    if ( pass )     g_isOK = 1;
}


static void 
errorCallBack( void *conn, Cus_CANTP_ErrCode_t err )
{
    (void)conn;
    print( (uint8_t)err );
}


/* *************************************** TEST *************************************** */
void 
test_run_all( void )
{
    test_normal_sf_7b();
    test_normal_ff_35b_bs0();
    test_normal_ff_35b_bs2();
    test_normal_ff_4095();
    test_ext_sf_6b();
    test_ext_ff_4095b();
    test_func_sf_broadcast();
}


void 
test_normal_sf_7b( void )
{
    uint8_t data[7] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xAA };

    const Cus_CANTP_ChannelCfg_t rxCh = {
		.addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
		.SA       = 0x00,          
		.TA       = 0x02,          
		.TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
		.fSize    = CUS_CANTP_SIZE_8,
		.funcId   = 0
	};

	const Cus_CANTP_RxConn_t *rxConn = Cus_Cantp_CreateRxConn(
		rxCh,
		(void *)CAN1,
		NULL,
		sendCallBack,
		errorCallBack
	);
	Cus_Cantp_RxConnBindBuf( rxConn, rx_buf, sizeof(rx_buf), 0, 0, dataReceiveCallBack );

	/* ── Create Tx Conn ── */
	const Cus_CANTP_ChannelCfg_t txCh = {
		.addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
		.SA       = 0x01,
		.TA       = 0x02,
		.TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
		.fSize    = CUS_CANTP_SIZE_8,
		.funcId   = 0
	};

	const Cus_CANTP_TxConn_t *txConn = Cus_Cantp_CreateTxConn(
		txCh,
		(void *)CAN1,
		NULL,
		sendCallBack,
		errorCallBack
	);

    demo_setExpectData( data, sizeof(data) );
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)txConn, data, sizeof(data) );

    while( !g_isOK )
        Cus_Cantp_MainFunction();

    Cus_Cantp_ReleaseConn( (void *)txConn, CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rxConn, CUS_CANTP_CONN_TYPE_RX );

    g_isOK = 0;
}


void 
test_normal_ff_35b_bs0( void )
{
    uint8_t data[35] = { 0 };
    for( uint8_t i = 0; i < sizeof(data); i++ )
        data[i] = i;

    const Cus_CANTP_ChannelCfg_t rxCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x00,          
        .TA       = 0x02,          
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = 0
    };

    const Cus_CANTP_RxConn_t *rxConn = Cus_Cantp_CreateRxConn(
        rxCh,
        (void *)CAN1,
        NULL,
        sendCallBack,
        errorCallBack
    );
    Cus_Cantp_RxConnBindBuf( rxConn, rx_buf, sizeof(rx_buf), 0, 0, dataReceiveCallBack );

    /* ── Create Tx Conn ── */
    const Cus_CANTP_ChannelCfg_t txCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x01,
        .TA       = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = 0
    };

    const Cus_CANTP_TxConn_t *txConn = Cus_Cantp_CreateTxConn(
        txCh,
        (void *)CAN1,
        NULL,
        sendCallBack,
        errorCallBack
    );

    demo_setExpectData( data, sizeof(data) );
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)txConn, data, sizeof(data) );

    while( !g_isOK )
        Cus_Cantp_MainFunction();

    Cus_Cantp_ReleaseConn( (void *)txConn, CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rxConn, CUS_CANTP_CONN_TYPE_RX );

    g_isOK = 0;
}


void 
test_normal_ff_35b_bs2( void )
{
    uint8_t data[35] = { 0 };
    for( uint8_t i = 0; i < sizeof(data); i++ )
        data[i] = i;

    const Cus_CANTP_ChannelCfg_t rxCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x00,          
        .TA       = 0x02,          
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = 0
    };

    const Cus_CANTP_RxConn_t *rxConn = Cus_Cantp_CreateRxConn(
        rxCh,
        (void *)CAN1,
        NULL,
        sendCallBack,
        errorCallBack
    );
    Cus_Cantp_RxConnBindBuf( rxConn, rx_buf, sizeof(rx_buf), 2, 0, dataReceiveCallBack );

    /* ── Create Tx Conn ── */
    const Cus_CANTP_ChannelCfg_t txCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x01,
        .TA       = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = 0
    };

    const Cus_CANTP_TxConn_t *txConn = Cus_Cantp_CreateTxConn(
        txCh,
        (void *)CAN1,
        NULL,
        sendCallBack,
        errorCallBack
    );

    demo_setExpectData( data, sizeof(data) );
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)txConn, data, sizeof(data) );

    while( !g_isOK )
        Cus_Cantp_MainFunction();

    Cus_Cantp_ReleaseConn( (void *)txConn, CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rxConn, CUS_CANTP_CONN_TYPE_RX );

    g_isOK = 0;
}


void 
test_normal_ff_4095( void )
{
    static uint8_t data[4095] = { 0 };
    for( uint16_t i = 0; i < sizeof(data); i++ )
        data[i] = i;

    const Cus_CANTP_ChannelCfg_t rxCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x00,          
        .TA       = 0x02,          
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = 0
    };

    const Cus_CANTP_RxConn_t *rxConn = Cus_Cantp_CreateRxConn(
        rxCh,
        (void *)CAN1,
        NULL,
        sendCallBack,
        errorCallBack
    );
    Cus_Cantp_RxConnBindBuf( rxConn, rx_buf, sizeof(rx_buf), 0, 0, dataReceiveCallBack );

    /* ── Create Tx Conn ── */
    const Cus_CANTP_ChannelCfg_t txCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x01,
        .TA       = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = 0
    };

    const Cus_CANTP_TxConn_t *txConn = Cus_Cantp_CreateTxConn(
        txCh,
        (void *)CAN1,
        NULL,
        sendCallBack,
        errorCallBack
    );

    demo_setExpectData( data, sizeof(data) );
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)txConn, data, sizeof(data) );

    while( !g_isOK )
        Cus_Cantp_MainFunction();

    Cus_Cantp_ReleaseConn( (void *)txConn, CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rxConn, CUS_CANTP_CONN_TYPE_RX );

    g_isOK = 0;
}


void 
test_ext_sf_6b( void )
{
    uint8_t data[6] = { 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6 };

    const Cus_CANTP_ChannelCfg_t rxCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_EXT,
        .SA       = 0x00, .TA = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_RxConn_t *rxConn = Cus_Cantp_CreateRxConn(
        rxCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );
    Cus_Cantp_RxConnBindBuf( rxConn, rx_buf, sizeof(rx_buf), 0, 0, dataReceiveCallBack );

    const Cus_CANTP_ChannelCfg_t txCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_EXT,
        .SA = 0x01, .TA = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_TxConn_t *txConn = Cus_Cantp_CreateTxConn(
        txCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );

    demo_setExpectData( data, sizeof(data) );
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)txConn, data, sizeof(data) );

    while( !g_isOK )  
        Cus_Cantp_MainFunction();

    Cus_Cantp_ReleaseConn( (void *)txConn, CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rxConn, CUS_CANTP_CONN_TYPE_RX );

    g_isOK = 0;
}


void 
test_ext_ff_4095b( void )
{
    static uint8_t data[4095] = { 0 };
    for( uint16_t i = 0; i < sizeof(data); i++ )
        data[i] = (uint8_t)(i & 0xFF);

    const Cus_CANTP_ChannelCfg_t rxCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_EXT,
        .SA = 0x00, .TA = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_RxConn_t *rxConn = Cus_Cantp_CreateRxConn(
        rxCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );
    Cus_Cantp_RxConnBindBuf( rxConn, rx_buf, sizeof(rx_buf), 0, 0, dataReceiveCallBack );

    const Cus_CANTP_ChannelCfg_t txCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_EXT,
        .SA = 0x01, .TA = 0x02,
        .TAType   = CUS_CANTP_TA_TYPE_PHYSICAL,
        .fSize    = CUS_CANTP_SIZE_8, .funcId = 0
    };
    const Cus_CANTP_TxConn_t *txConn = Cus_Cantp_CreateTxConn(
        txCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );

    demo_setExpectData( data, sizeof(data) );
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)txConn, data, sizeof(data) );

    while( !g_isOK )  
        Cus_Cantp_MainFunction();

    Cus_Cantp_ReleaseConn( (void *)txConn, CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rxConn, CUS_CANTP_CONN_TYPE_RX );

    g_isOK = 0;
}


/* ==================================================================
 * Test: Functional SF broadcast — one Tx, two Rx both listen on funcId
 *
 * Verifies:
 *   1. Functional SF is routed to ALL RxConns with matching funcId
 *   2. Physical FindRxById correctly returns NULL for functional frames
 *   3. DataInd fires on every matching RxConn (not just the first)
 *   4. SA == 0 wildcard works for functional as well
 * ================================================================== */

/* per-test state for the functional-broadcast callback */
static uint8_t  g_func_rx_count;
static uint8_t  g_func_buf1[256];
static uint32_t g_func_len1;
static uint8_t  g_func_buf2[256];
static uint32_t g_func_len2;

static void func_broadcast_cb( void *rxConn, const uint8_t *data, uint32_t len )
{
    (void)rxConn;
    if ( g_func_rx_count == 0 )
    {
        g_func_len1 = (len < sizeof(g_func_buf1)) ? len : sizeof(g_func_buf1);
        memcpy( g_func_buf1, data, g_func_len1 );
    }
    else
    {
        g_func_len2 = (len < sizeof(g_func_buf2)) ? len : sizeof(g_func_buf2);
        memcpy( g_func_buf2, data, g_func_len2 );
    }
    g_func_rx_count++;
}

void 
test_func_sf_broadcast( void )
{
    uint8_t data[7] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
    const uint32_t funcId = 0x7DF;   /* OBD functional request */

    printf( "\n--- FUNC SF BROADCAST (funcId=0x%03lX) ---\n", funcId );

    /* ── Create TWO RxConns both listening on the same functional ID ── */
    const Cus_CANTP_ChannelCfg_t rxCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x00,           /* wildcard: accept any sender          */
        .TA       = 0x00,           /* unused for functional (funcId wins)  */
        .TAType   = CUS_CANTP_TA_TYPE_FUNCTIONAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = funcId
    };

    const Cus_CANTP_RxConn_t *rx1 = Cus_Cantp_CreateRxConn(
        rxCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );
    Cus_Cantp_RxConnBindBuf( rx1, g_func_buf1, sizeof(g_func_buf1),
                              0, 0, func_broadcast_cb );

    const Cus_CANTP_RxConn_t *rx2 = Cus_Cantp_CreateRxConn(
        rxCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );
    Cus_Cantp_RxConnBindBuf( rx2, g_func_buf2, sizeof(g_func_buf2),
                              0, 0, func_broadcast_cb );

    /* ── Create functional TxConn ── */
    const Cus_CANTP_ChannelCfg_t txCh = {
        .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,
        .SA       = 0x01,           /* source address (for reference)       */
        .TA       = 0x00,           /* unused for functional                */
        .TAType   = CUS_CANTP_TA_TYPE_FUNCTIONAL,
        .fSize    = CUS_CANTP_SIZE_8,
        .funcId   = funcId
    };
    const Cus_CANTP_TxConn_t *tx = Cus_Cantp_CreateTxConn(
        txCh, (void *)CAN1, NULL, sendCallBack, errorCallBack );

    /* ── Send & poll until both RxConns fire DataInd ── */
    g_func_rx_count = 0;
    Cus_Cantp_StartTransmit( (Cus_CANTP_TxConn_t *)tx, data, sizeof(data) );

    uint32_t timeout = Cus_Cantp_TimerNow() + 2000;
    while ( g_func_rx_count < 2 && (int32_t)(Cus_Cantp_TimerNow() - timeout) < 0 )
        Cus_Cantp_MainFunction();

    /* ── Validate ── */
    bool ok1 = (g_func_rx_count >= 1)
            && (g_func_len1 == sizeof(data))
            && (memcmp(g_func_buf1, data, sizeof(data)) == 0);
    bool ok2 = (g_func_rx_count >= 2)
            && (g_func_len2 == sizeof(data))
            && (memcmp(g_func_buf2, data, sizeof(data)) == 0);

    printf( "  Rx1 received: %luB  %s\n", g_func_len1, ok1 ? "PASS" : "FAIL" );
    printf( "  Rx2 received: %luB  %s\n", g_func_len2, ok2 ? "PASS" : "FAIL" );
    printf( "  Total callbacks fired: %u  %s\n",
            g_func_rx_count, (g_func_rx_count == 2) ? "PASS" : "FAIL" );

    /* ── Clean up ── */
    Cus_Cantp_ReleaseConn( (void *)tx,  CUS_CANTP_CONN_TYPE_TX );
    Cus_Cantp_ReleaseConn( (void *)rx1, CUS_CANTP_CONN_TYPE_RX );
    Cus_Cantp_ReleaseConn( (void *)rx2, CUS_CANTP_CONN_TYPE_RX );
}
