/**
 * @file ctp_fsm.h
 * @brief CAN TP State Machine — Public Interface
 * @author R6bandito
 * @date 2026-8
 *
 * Public entry points for the CAN TP finite state machine.
 * ISR callbacks (FeedFrame, TxConfirm) run inline; all
 * user callbacks are deferred to MainFunction (Task context).
 */
#ifndef __CANTP_FSM_H__
#define __CANTP_FSM_H__


#include "ctp_types.h"
#include "ctp_frame.h"
#include "ctp_conn.h"


/**
 * @brief  Initiate a CAN TP transmission.
 *
 * Automatically chooses Single Frame or First Frame based on
 * the payload size and connection parameters.
 *
 * @param conn   TxConn created by Cus_Cantp_CreateTxConn()
 * @param data   Source data (must remain valid until state == IDLE)
 * @param len    Total byte count (1 ~ 4095)
 * @return  1 = started,  0 = bad arg, -1 = busy,
 *         -2 = SendFunc failed, -3 = channel config error
 */
int8_t Cus_Cantp_StartTransmit( Cus_CANTP_TxConn_t *conn,
                                const uint8_t *data, uint32_t len );

                                
/**
 * @brief  CAN TX completion callback — call from CAN TX ISR.
 *
 * @param bind_dev   CAN device handle (must match conn->bind_dev)
 * @param tag        Opaque tag returned by the application's SendFunc
 */
void Cus_Cantp_TxConfirm( void *bind_dev, uint32_t tag );


/**
 * @brief  Feed a received CAN frame into the protocol stack.
 *
 * Call from the CAN RX interrupt handler.  Frame processing
 * (parse, copy, state update) is done inline; user callbacks
 * are deferred to Cus_Cantp_MainFunction().
 *
 * @param canId    CAN identifier
 * @param data     Raw CAN data field
 * @param dlc      Data Length Code from the hardware register (0-15)
 */
void Cus_Cantp_FeedFrame( uint32_t canId, const uint8_t *data, uint8_t dlc );


/**
 * @brief  Main state machine driver — call periodically from Task context.
 *
 * Handles timer expiry checks, STmin-gated CF transmission,
 * deferred FC sending, DataInd and ErrCb callbacks.
 *
 * Typical call period: 1 - 10 ms.
 */
void Cus_Cantp_MainFunction( void );


#endif /* __CANTP_FSM_H__ */
