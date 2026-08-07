/**
 * @file ctp_conn.h
 * @brief CAN TP Connection Management — Pool, Create, Release, Lookup
 * @author R6bandito
 * @date 2026-8
 *
 * Manages two static pools of Tx/Rx connection control blocks.
 * Provides allocation (Create), deallocation (Release), and
 * routing (Find*) APIs for the FSM layer and ISR dispatch.
 */
#ifndef __CANTP_CONN_H__
#define __CANTP_CONN_H__


#include "ctp_types.h"
#include "ctp_channel.h"
#include "ctp_timer.h"


/*============================================================================
 * Pool Sizing (user-overridable)
 *============================================================================*/
#ifndef CUS_CANTP_MAX_TX
	#define CUS_CANTP_MAX_TX		(2)
#endif

#ifndef CUS_CANTP_MAX_RX
	#define CUS_CANTP_MAX_RX		(2)
#endif


/*============================================================================
 * Connection Control Blocks
 *============================================================================*/

/*
 * TxConn — Transmit Connection
 */
struct Cus_CANTP_TxConn
{
    /* ---- Protocol state ---- */
    Cus_CANTP_State_t       state;      /* current FSM state                  */
    Cus_CANTP_ChannelCfg_t  channel;    /* embedded addressing config (copy)  */
    uint8_t                 sn;         /* next CF sequence number (0-15)     */
    int8_t                  index;      /* pool index, -1 = slot free         */

    /* ---- Send buffer context ---- */
    const uint8_t          *tx_data;    /* source data to transmit            */
    uint32_t                tot_len;    /* total message length               */
    uint32_t                pos;        /* current send offset in tx_data     */

    /* ---- Timers ---- */
    Cus_CANTP_Timer_t       t_n_as;     /* N_As: send confirmation (async)    */
    Cus_CANTP_Timer_t       t_n_bs;     /* N_Bs: wait for Flow Control        */
    Cus_CANTP_Timer_t       t_stmin;    /* STmin: inter-frame separation      */

    /* ---- Flow control (received from peer) ---- */
    uint8_t                 bs;         /* block size                         */
    uint8_t                 stmin;      /* separation time minimum            */
    uint16_t                bs_rem;     /* remaining CFs in current block     */

    /* ---- Callbacks ---- */
    Cus_Cantp_SendFunc_t    send;       /* CAN frame transmit                 */
    Cus_Cantp_ErrCb_t       err;        /* error notification                 */

    /* ---- Deferred callback (Phase 1 sets flags, Phase 2 dispatches) ---- */
    uint8_t                 cb_pending;  /* bit0: error callback is pending    */
    uint8_t                 cb_err_code; /* cached error code for Phase 2      */

    /*
     * Platform adapter fields — NOT interpreted by CANTP.
     * Set by the application during connection creation or inside callbacks.
     * Used for opaque matching in TxConfirm / FeedFrame routing.
     * ------------------------------------------------------------------
     * user_ctx : passed through to send / err callbacks (e.g. CAN handle,
     *            queue handle, application session pointer).
     * bind_dev : opaque device handle used by TxConfirm to perform
     *            pointer-comparison routing (e.g. &hcan1 on STM32).
     * tx_tag   : opaque tag set by the application's SendFunc.  CANTP
     *            compares it in TxConfirm to match the completed
     *            mailbox / queue entry back to this connection.
     *            On STM32 this is typically the CAN Tx mailbox number
     *            (0-2); on other platforms it may be a queue index or 0.
     */
    void                   *user_ctx;
    void                   *bind_dev;
    uint32_t                tx_tag;
};


/*
 * RxConn — Receive Connection
 */
struct Cus_CANTP_RxConn
{
    /* ---- Protocol state ---- */
    Cus_CANTP_State_t       state;
    Cus_CANTP_ChannelCfg_t  channel;
    uint8_t                 sn;         /* expected next CF sequence number   */
    int8_t                  index;      /* pool index, -1 = slot free         */

    /* ---- Receive buffer context ---- */
    uint8_t                *recv_buf;   /* user-provided reassembly buffer    */
    uint32_t                recv_size;  /* size of recv_buf                   */
    uint32_t                tot_len;    /* total message length (from FF)     */
    uint32_t                pos;        /* next write offset in recv_buf      */

    /* ---- Flow control (local config) ---- */
    uint8_t                 bs;         /* block size                         */
    uint8_t                 stmin;      /* separation time minimum            */
    uint16_t                bs_rem;     /* remaining CFs before next FC       */

    /* ---- Timers ---- */
    Cus_CANTP_Timer_t       t_n_ar;     /* N_Ar: FC send confirmation         */
    Cus_CANTP_Timer_t       t_n_cr;     /* N_Cr: wait for Consecutive Frame   */

    /* ---- Retry state ---- */
    uint8_t                 fc_pending; /* 1 = FC send deferred (mailbox busy),
                                           retried in MainFunction            */

    /* ---- Callbacks ---- */
    Cus_Cantp_SendFunc_t    send;       /* for sending Flow Control frames    */
    Cus_Cantp_DataInd_t     data_ind;   /* reassembly complete notification   */
    Cus_Cantp_ErrCb_t       err;        /* error notification                 */

    /* ---- Deferred callback (Phase 1 sets flags, Phase 2 dispatches) ---- */
    uint8_t                 cb_pending;  /* bit0: err, bit1: DataInd pending   */
    uint8_t                 cb_err_code; /* cached error code for err callback */

    /*
     * Platform adapter fields — see TxConn for detailed explanation.
     */
    void                   *user_ctx;   /* passed through to callbacks        */
    void                   *bind_dev;   /* CAN device for TxConfirm routing   */
    uint32_t                fc_tag;     /* FC mailbox tag (see tx_tag above)  */

    /* ---- Peer address ---- */
    uint8_t                 peer_sa;    /* sender's SA, used as TA when
                                           constructing FC CAN ID             */
};


/*============================================================================
 * Pool Initialization
 *============================================================================*/
void Cus_Cantp_ConnPoolInit( void );


/*============================================================================
 * Connection Lifecycle
 *============================================================================*/
const Cus_CANTP_RxConn_t *Cus_Cantp_CreateRxConn( const Cus_CANTP_ChannelCfg_t channel,
                                                   void *bind_dev, void *user_ctx,
                                                   Cus_Cantp_SendFunc_t sendFn,
                                                   Cus_Cantp_ErrCb_t errFn );

const Cus_CANTP_TxConn_t *Cus_Cantp_CreateTxConn( const Cus_CANTP_ChannelCfg_t channel,
                                                   void *bind_dev, void *user_ctx,
                                                   Cus_Cantp_SendFunc_t sendFn,
                                                   Cus_Cantp_ErrCb_t errFn );

void Cus_Cantp_RxConnBindBuf( const Cus_CANTP_RxConn_t *conn, uint8_t *pbuf,
                               uint32_t size, uint8_t bs, uint8_t stmin,
                               Cus_Cantp_DataInd_t commit );

void Cus_Cantp_ReleaseConn( void *conn, Cus_CANTP_ConnType_t type );


/*============================================================================
 * ISR Routing — Tag-based (TxConfirm)
 *============================================================================*/
Cus_CANTP_TxConn_t *Cus_Cantp_FindTxByTag( void *bind_dev, uint32_t tag );
Cus_CANTP_RxConn_t *Cus_Cantp_FindRxByTag( void *bind_dev, uint32_t tag );


/*============================================================================
 * ISR Routing — CAN-ID-based (FeedFrame)
 *============================================================================*/
Cus_CANTP_TxConn_t *Cus_Cantp_FindTxById( uint32_t canId, uint8_t *frame );
Cus_CANTP_RxConn_t *Cus_Cantp_FindRxById( uint32_t canId, uint8_t *frame );


#endif /* __CANTP_CONN_H__ */
