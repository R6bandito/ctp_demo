/**
 * @file ctp_conn.c
 * @brief CAN TP Connection Management — Implementation
 * @author R6bandito
 * @date 2026-8
 *
 * Static pool allocation, connection lifecycle, and lookup routing.
 */
#include "ctp_conn.h"
#include <string.h>


/* ***************************************************** */
static Cus_CANTP_TxConn_t txPool[CUS_CANTP_MAX_TX];
static Cus_CANTP_RxConn_t rxPool[CUS_CANTP_MAX_RX];

static Cus_CANTP_TxConn_t *AllocTxConn( void );
static Cus_CANTP_RxConn_t *AllocRxConn( void );
/* ***************************************************** */

/**
 * @brief  Initialise connection pools.
 *         Mark every slot in TxPool and RxPool as free.
 *         Must be called once before any Create/Release.
 */
void 
Cus_Cantp_ConnPoolInit( void )
{
	for( uint8_t index = 0; index < CUS_CANTP_MAX_TX; index++ )
		txPool[index].index = -1;

	for( uint8_t index = 0; index < CUS_CANTP_MAX_RX; index++ )
		rxPool[index].index = -1;
}


static Cus_CANTP_TxConn_t *
AllocTxConn( void )
{
	/* Check for a free slot; allocate one if available. */
	int8_t freeSlots = -1;
	for( uint8_t index = 0; index < CUS_CANTP_MAX_TX; index++ )
	{
		if ( txPool[index].index == -1 && txPool[index].state == CUS_CANTP_STA_IDLE )
		{
			/* Find the freeslots. Return with the relevant Addr. */
			memset(&txPool[index], 0, sizeof(txPool[index]));
			freeSlots = index;
			txPool[freeSlots].index = freeSlots;
			return &txPool[freeSlots];
		}
	}

	/* No Space. Return NULL. */
	return NULL;
}


static Cus_CANTP_RxConn_t *
AllocRxConn( void )
{
	/* Check for a free slot; allocate one if available. */
	int8_t freeSlots = -1;
	for( uint8_t index = 0; index < CUS_CANTP_MAX_RX; index++ )
	{
		if ( rxPool[index].index == -1 && rxPool[index].state == CUS_CANTP_STA_IDLE )
		{
			/* Find the freeslots. Return with the relevant Addr. */
			memset(&rxPool[index], 0, sizeof(rxPool[index]));
			freeSlots = index;
			rxPool[freeSlots].index = freeSlots;
			return &rxPool[freeSlots];
		}
	}

	/* No Space. Return NULL. */
	return NULL;
}


/**
 * @brief  Release a connection and return its slot to the pool.
 *
 * Stops all associated timers so MainFunction skip the slot,
 * then marks it as free (index = -1, state = IDLE).
 * The slot is zero-filled on the next Alloc, not here.
 *
 * @param conn  Opaque pointer to TxConn or RxConn
 * @param type  CUS_CANTP_CONN_TYPE_TX or CUS_CANTP_CONN_TYPE_RX
 *
 * @warning  Call only from task context (no ISR).  The internal
 *           assertion traps double-free or NULL-pointer bugs.
 */
void 
Cus_Cantp_ReleaseConn( void *conn, Cus_CANTP_ConnType_t type )
{
	CUS_CANTP_ASSERT( ((type == CUS_CANTP_CONN_TYPE_TX) || (type == CUS_CANTP_CONN_TYPE_RX)) );
	CUS_CANTP_ASSERT( conn != NULL );

	if ( type == CUS_CANTP_CONN_TYPE_TX )
	{
		Cus_CANTP_TxConn_t *tx = (Cus_CANTP_TxConn_t *)conn;
		tx->index = -1;
		tx->state = CUS_CANTP_STA_IDLE;
		Cus_Cantp_TimerStop(&tx->t_n_as);
		Cus_Cantp_TimerStop(&tx->t_n_bs);
		Cus_Cantp_TimerStop(&tx->t_stmin);
	}
	else if ( type == CUS_CANTP_CONN_TYPE_RX )
	{
		Cus_CANTP_RxConn_t *rx = (Cus_CANTP_RxConn_t *)conn;
		rx->index = -1;
		rx->state = CUS_CANTP_STA_IDLE;
		Cus_Cantp_TimerStop(&rx->t_n_ar);
		Cus_Cantp_TimerStop(&rx->t_n_cr);
	}
}


/**
 * @brief  Allocate and partially configure a receive connection.
 *
 * Sets the mandatory fields (channel, callbacks, device bindings).
 * The receive buffer and flow-control parameters are configured
 * separately via Cus_Cantp_RxConnBindBuf().
 * Returns NULL if no free slot is available or required params are missing.
 */
const Cus_CANTP_RxConn_t *
Cus_Cantp_CreateRxConn( const Cus_CANTP_ChannelCfg_t channel, void *bind_dev, void *user_ctx, Cus_Cantp_SendFunc_t sendFn, Cus_Cantp_ErrCb_t errFn )
{
	if ( !bind_dev || !sendFn )
		return NULL;

    Cus_CANTP_RxConn_t *newConn = AllocRxConn();
	if ( !newConn )
	/* No freeslots. Return NULL. */
		return NULL;

	newConn->bind_dev = bind_dev;
	newConn->channel = channel;
	newConn->user_ctx = user_ctx;
	newConn->send = sendFn;
	newConn->err = errFn;
	newConn->state = CUS_CANTP_STA_IDLE;

	return newConn;
}


/**
 * @brief  Bind a receive buffer and flow-control parameters to a RxConn.
 *         Must be called after Cus_Cantp_CreateRxConn().
 */
void 
Cus_Cantp_RxConnBindBuf( const Cus_CANTP_RxConn_t *conn, uint8_t *pbuf, uint32_t size, uint8_t bs, uint8_t stmin, Cus_Cantp_DataInd_t commit )
{
	if ( !conn || !pbuf || !commit || !size )
		return;

	CUS_CANTP_ASSERT(conn->index < CUS_CANTP_MAX_RX);
	Cus_CANTP_RxConn_t *rc = &rxPool[conn->index];

	rc->recv_buf = pbuf;
	rc->recv_size = size;
	rc->bs = bs;
	rc->stmin = stmin;
	rc->data_ind = commit;
}


/**
 * @brief  Allocate and configure a transmit connection.  See CreateRxConn.
 */
const Cus_CANTP_TxConn_t *
Cus_Cantp_CreateTxConn( const Cus_CANTP_ChannelCfg_t channel, void *bind_dev, void *user_ctx, Cus_Cantp_SendFunc_t sendFn, Cus_Cantp_ErrCb_t errFn )
{
	if ( !bind_dev || !sendFn )
		return NULL;

	Cus_CANTP_TxConn_t *newConn = AllocTxConn();
	if ( !newConn )
	/* No freeslots. Return NULL. */
		return NULL;

	newConn->bind_dev = bind_dev;
	newConn->send = sendFn;
	newConn->channel = channel;
	newConn->err = errFn;
	newConn->user_ctx = user_ctx;
	
	return newConn;
}


/**
 * @brief  Find a TxConn by its platform device pointer and opaque tag.
 *         Used by TxConfirm ISR routing.
 */
Cus_CANTP_TxConn_t *
Cus_Cantp_FindTxByTag( void *bind_dev, uint32_t tag )
{
	if ( !bind_dev )	
		return NULL;

	for( uint8_t index = 0; index < CUS_CANTP_MAX_TX; index++ )
	{
		if ( txPool[index].index == -1 )
		/* UnActivate entity. Skip */
			continue;

		if ( (txPool[index].bind_dev == bind_dev) && (txPool[index].tx_tag == tag) )
		{
			/* Find the uniquely identified device. */
			return &txPool[index];
		}
	}

	/* No match device. Return NULL. */
	return NULL;
}


/**
 * @brief  Find a RxConn by its platform device pointer and opaque tag.
 *         Used by TxConfirm ISR routing (FC confirmation).
 */
Cus_CANTP_RxConn_t *
Cus_Cantp_FindRxByTag( void *bind_dev, uint32_t tag )
{
	if ( !bind_dev )
		return NULL;

	for( uint8_t index = 0; index < CUS_CANTP_MAX_RX; index++ )
	{	
		if ( rxPool[index].index == -1 )
		/* UnActivate entity. Skip */
			continue;

		if ( (rxPool[index].bind_dev == bind_dev) && (rxPool[index].fc_tag == tag) )
		{
			/* Find the uniquely identified device. */
			return &rxPool[index];
		}
	}

	/* No match device. Return NULL. */
	return NULL;
}


/**
 * @brief  Find a TxConn waiting for a Flow Control frame matching the
 *         given CAN ID.  Used by FeedFrame to route incoming FC frames.
 */
Cus_CANTP_TxConn_t *
Cus_Cantp_FindTxById( uint32_t canId, uint8_t *frame )
{
	for( uint8_t index = 0; index < CUS_CANTP_MAX_TX; index++ )
	{
		if ( txPool[index].index == -1 )
		/* UnActivate entity. Skip */
			continue;

		Cus_CANTP_ChannelCfg_t *cfg = &txPool[index].channel;
		if ( cfg->addrMode == CUS_CANTP_ADDR_MODE_NORMAL )
		{
			/*
			 * FC is always a physical point‑to‑point reply.
			 * Peer puts our SA in the TA field and its own SA (our TA)
			 * in the SA field.  Compare the swapped values directly.
			 */
			uint8_t can_ta = (canId >> 1) & 0x1F;
			uint8_t can_sa = (canId >> 6) & 0x1F;

			if ( can_ta == cfg->SA && can_sa == cfg->TA )
			{
				/* Find relevant device. */
				return &txPool[index];
			}
		}
		else if ( cfg->addrMode == CUS_CANTP_ADDR_MODE_EXT )
		{
			/*
			 * EXT: SA sits in CAN ID bits 10:3,
			 * TA sits in the first data byte.
			 * FC replies: peer's SA = our TA, peer's TA = our SA.
			 */
			uint8_t can_sa = (canId >> 3) & 0xFF;

			if ( can_sa == cfg->TA && frame[0] == cfg->SA )
			{
				/* Find relevant device. */
				return &txPool[index];
			}
		}
	}

	return NULL;
}


/**
 * @brief  Find a RxConn whose addressing matches the incoming CAN ID.
 *         Used by FeedFrame to route SF/FF/CF frames.
 */
Cus_CANTP_RxConn_t *
Cus_Cantp_FindRxById( uint32_t canId, uint8_t *frame )
{
	for( uint8_t index = 0; index < CUS_CANTP_MAX_RX; index++ )
	{
		if ( rxPool[index].index == -1 )
			continue;

		Cus_CANTP_ChannelCfg_t *cfg = &rxPool[index].channel;
		if ( cfg->addrMode == CUS_CANTP_ADDR_MODE_NORMAL )
		{
			/*
			 * NORMAL: TA in bits 5:1, TA_Type in bit 0,
			 * SA in bits 10:6.
			 * If SA == 0, accept any sender (wildcard).
			 */
			uint8_t can_ta      = (canId >> 1) & 0x1F;
			uint8_t can_ta_type =  canId & 0x01;

			if ( can_ta != cfg->TA || can_ta_type != cfg->TAType )
				continue;

			if ( cfg->SA != 0 )
			{
				uint8_t can_sa = (canId >> 6) & 0x1F;
				if ( can_sa != cfg->SA )
					continue;
			}

			/* Find relevant device. */
			return &rxPool[index];
		}
		else if ( cfg->addrMode == CUS_CANTP_ADDR_MODE_EXT )
		{
			/*
			 * EXT: TA in data byte 0, SA in CAN ID bits 10:3.
			 * SA == 0, accept any sender.
			 */
			if ( frame[0] != cfg->TA )
				continue;

			if ( cfg->SA != 0 )
			{
				uint8_t can_sa = (canId >> 3) & 0xFF;
				if ( can_sa != cfg->SA )
					continue;
			}

			/* Find relevant device. */
			return &rxPool[index];
		}
	}

	return NULL;
}


Cus_CANTP_TxConn_t *
Cus_Cantp_GetTxConn( uint8_t index )
{
	return (index < CUS_CANTP_MAX_TX) ? &txPool[index] : NULL;
}


Cus_CANTP_RxConn_t *
Cus_Cantp_GetRxConn( uint8_t index )
{
	return (index < CUS_CANTP_MAX_RX) ? &rxPool[index] : NULL;
}
