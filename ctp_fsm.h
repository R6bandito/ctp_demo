#ifndef __CANTP_FSM_H__
#define __CANTP_FSM_H__


#include "ctp_types.h"
#include "ctp_frame.h"
#include "ctp_conn.h"


int8_t Cus_Cantp_StartTransmit( Cus_CANTP_TxConn_t *conn, const uint8_t *data, uint32_t len );

void Cus_Cantp_TxConfirm( void *bind_dev, uint32_t tag );

void Cus_Cantp_MainFunction( void );

void Cus_Cantp_FeedFrame( uint32_t canId, const uint8_t *data, uint8_t dlc );

#endif /* __CANTP_FSM_H__ */

