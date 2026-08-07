/**
 * @file ctp_fsm.c
 * @brief CAN TP Finite State Machine — Implementation
 * @author R6bandito
 * @date 2026-8
 *
 * Core protocol engine.  Handles:
 *   - StartTransmit   (SF / FF dispatching)
 *   - FeedFrame       (ISR frame routing → SF/FF/CF/FC handlers)
 *   - TxConfirm       (ISR TX-completion → state advancement)
 *   - MainFunction    (Task-level timer expiry, STmin, deferred FC, callbacks)
 *   - SendNextCF      (CF construction and submission)
 *   - SendFC          (Flow Control frame construction and submission)
 *
 * No user callbacks are invoked from ISR context.
 * All DataInd / ErrCb calls are deferred to MainFunction.
 */
#include "ctp_fsm.h"
#include <string.h>


/* ******************************************************************************************** */
static void Cus_Cantp_SFHandler( Cus_CANTP_RxConn_t *rx, const uint8_t *data, Cus_CANTP_FrameSize_t fs );
static void Cus_Cantp_FFHandler( Cus_CANTP_RxConn_t *rx, const uint8_t *data, uint32_t canId, Cus_CANTP_FrameSize_t fs );
static void Cus_Cantp_CFHandler( Cus_CANTP_RxConn_t *rx, const uint8_t *data, Cus_CANTP_FrameSize_t fs );
static void Cus_Cantp_FCHandler( Cus_CANTP_TxConn_t *tx, const uint8_t *data );

static bool SendNextCF( Cus_CANTP_TxConn_t *tx );
static bool SendFC( Cus_CANTP_RxConn_t *rx, Cus_CANTP_FLOWState_t flow );

static void TxConnResetTx( Cus_CANTP_TxConn_t *conn );
static void RxConnResetRx( Cus_CANTP_RxConn_t *rx );

extern Cus_CANTP_TxConn_t *Cus_Cantp_GetTxConn( uint8_t index );
extern Cus_CANTP_RxConn_t *Cus_Cantp_GetRxConn( uint8_t index );

/*============================================================================
 * OS Critical-Section Stubs (weak — overridden by RTOS port)
 *============================================================================*/
#ifndef __weak
	#if defined(__GNUC__) || defined(__clang__)
		#define __weak  __attribute__((weak))
	#elif defined(__CC_ARM)
		#define __weak  __weak
	#endif
#endif

__weak void Cus_Cantp_OS_EnterCritical(void) {}
__weak void Cus_Cantp_OS_ExitCritical(void) {}


/*============================================================================
 * Deferred-Callback Helpers
 * Error / completion detected in Phase 1 (critical section) → set cb_pending.
 * The actual callback is dispatched in Phase 2 (outside critical section).
 *============================================================================*/

/**
 * @brief  Record a Tx connection error for deferred callback.
 *         Resets transient fields and returns the slot to IDLE.
 */
static void 
TxConnFlagError( Cus_CANTP_TxConn_t *tx, Cus_CANTP_ErrCode_t err )
{
	tx->cb_pending  = 1;
	tx->cb_err_code = (uint8_t)err;
	tx->state       = CUS_CANTP_STA_IDLE;
	TxConnResetTx(tx);
}

/**
 * @brief  Record an Rx connection error for deferred callback.
 *         Resets transient fields and returns the slot to IDLE.
 */
static void 
RxConnFlagError( Cus_CANTP_RxConn_t *rx, Cus_CANTP_ErrCode_t err )
{
	rx->cb_pending  = 1;    /* bit0 = err pending */
	rx->cb_err_code = (uint8_t)err;
	RxConnResetRx(rx);
	rx->state = CUS_CANTP_STA_IDLE;
}
/* ******************************************************************************************** */


static void 
Cus_Cantp_SFHandler( Cus_CANTP_RxConn_t *rx, const uint8_t *data, Cus_CANTP_FrameSize_t fs )
{
	if ( !rx || !data )
		return;

	if ( rx->state != CUS_CANTP_STA_IDLE )	
		return;

	uint8_t off = Cus_Cantp_GetPciOffset( rx->channel.addrMode );

	const uint8_t *pData;
	uint8_t pLen = 0;
	if ( !Cus_Cantp_ParseSF( data, fs, off, &pData, &pLen) )
	/* Parse Error. Return. */
		return;
	
	if ( pLen > rx->recv_size )
	{
		/* buffer overflow, drop */
		rx->state = CUS_CANTP_STA_IDLE;
		return;
	}

	/* Copy SF Data & Update state. */
	memcpy( rx->recv_buf, pData, pLen );
	rx->tot_len = pLen;
	rx->pos = pLen;

	/* MainFunction sees this → calls DataInd */
	rx->state = CUS_CANTP_STA_RX_SF;
}


static void 
Cus_Cantp_FFHandler( Cus_CANTP_RxConn_t *rx, const uint8_t *data, uint32_t canId, Cus_CANTP_FrameSize_t fs )
{
	if ( !rx || !data )
		return;

	if ( rx->state != CUS_CANTP_STA_IDLE )
		return;

	uint8_t off = Cus_Cantp_GetPciOffset( rx->channel.addrMode );

	const uint8_t *pData;
	uint8_t pLen = 0;
	uint32_t tot_len = 0;
	tot_len = Cus_Cantp_ParseFF( data, fs, off, &pData, &pLen );
	if ( !tot_len )
		return;

	/* Check if the receive buffer size is sufficient. */
	if ( tot_len > rx->recv_size )
	{
		/* Overflow! Set the OVERFLOW flow control flag and return. */
		rx->state = CUS_CANTP_STA_IDLE;
		RxConnResetRx( rx );
		rx->fc_pending = 1;
		return;
	}

	/* Copy FF payload */
	memcpy( rx->recv_buf, pData, pLen ); 

	/* Extract sender's SA for FC reply. */
	rx->peer_sa = Cus_Cantp_ExtractSA( rx->channel.addrMode, canId );

	/* Update state. */
	rx->tot_len = tot_len;
	rx->pos = pLen;
	rx->sn = 1;
	rx->bs_rem = rx->bs;

	/* Defer FC(CTS) to MainFunction. */
	rx->fc_pending = 1;
	rx->state = CUS_CANTP_STA_RX_FF;
}


static void 
Cus_Cantp_CFHandler( Cus_CANTP_RxConn_t *rx, const uint8_t *data, Cus_CANTP_FrameSize_t fs )
{
	if ( !rx || !data )
		return;

	if ( rx->state != CUS_CANTP_STA_RX_WAIT_CF )
		return;

	uint8_t off = Cus_Cantp_GetPciOffset( rx->channel.addrMode );

	const uint8_t *pData;
	uint8_t pLen = 0;
	uint8_t SN = 0;
	if ( !Cus_Cantp_ParseCF( data, fs, off, &SN, &pData, &pLen ) )
		return;

	if ( rx->sn != SN )
	{
		/* SN mismatch. */
		rx->state = CUS_CANTP_STA_IDLE;
		RxConnResetRx( rx );
		rx->fc_pending = 1;		/* MainFunction → Send OVERFLOW */
		return;
	}

	/* Copy CF payload. */
	uint32_t rem = rx->tot_len - rx->pos;
	uint8_t copyLen = (rem < pLen) ? (uint8_t)rem : pLen;
	memcpy( (rx->recv_buf + rx->pos), pData, copyLen);

	/* Update state. */
	if ( rx->bs > 0 )	rx->bs_rem--;
	rx->pos += copyLen;
	rx->sn = (rx->sn + 1) & 0x0F;

	/* Check completion / block boundary */
	if ( rx->pos == rx->tot_len )
	{
		/* All data received. */
		rx->state = CUS_CANTP_STA_RX_CF_COMPLETE;
		Cus_Cantp_TimerStop( &rx->t_n_cr );
		return;
	}
	else if ( (rx->bs > 0) && (rx->bs_rem == 0) )
	{
		rx->fc_pending = 1;		/* MainFunction → Send CTS */
	}

	Cus_Cantp_TimerStart( &rx->t_n_cr, CUS_CANTP_TIMEOUT_N_CR );

	/* else: stay in RX_WAIT_CF, N_Cr reset in MainFunction */
}


static void 
Cus_Cantp_FCHandler( Cus_CANTP_TxConn_t *tx, const uint8_t *data )
{
	if ( !tx || !data )
		return;

	if ( tx->state != CUS_CANTP_STA_TX_WAIT_FC )
		return;

	/* Received the FC. Stop the Timer. */
	Cus_Cantp_TimerStop(&tx->t_n_bs);

	uint8_t off = Cus_Cantp_GetPciOffset( tx->channel.addrMode );

	Cus_CANTP_FLOWState_t flowState;
	uint8_t BS = 0;
	uint8_t STmin = 0;
	if ( !Cus_Cantp_ParseFC( data, CUS_CANTP_SIZE_8, off, &flowState, &BS, &STmin ) )
		return;

	if ( flowState == CUS_CANTP_FLOW_OVERFLOW )
	{
		/* Received OVERFLOW. */
		tx->state = CUS_CANTP_STA_IDLE;
		return;
	}

	/* Update state. */
	tx->bs = BS;
	tx->bs_rem = (BS == 0) ? 0xFFFF : BS;	/* BS==0 means no limit. */
	tx->stmin = STmin;

	tx->state = CUS_CANTP_STA_TX_CF;
}


static bool 
SendNextCF( Cus_CANTP_TxConn_t *tx )
{
	if ( !tx || (tx->state != CUS_CANTP_STA_TX_CF) )
		return false;

	/*
	 * Gather addressing parameters once.
	 * pciOffset and CAN ID are constant for the entire multi-frame session.
	 */
	uint8_t off = Cus_Cantp_GetPciOffset( tx->channel.addrMode );
	uint8_t pLen = 0;
	uint32_t rem = tx->tot_len - tx->pos;
	uint8_t frame[CUS_CANTP_SIZE_64] = { 0 };

	/*
	 * Build the CF, convert the returned frame-size to a hardware DLC,
	 * and submit via the user's SendFunc callback.
	 */
	Cus_Cantp_WriteAddrPrefix( frame, &tx->channel );
	uint8_t fs = Cus_Cantp_BuildCF( frame, tx->channel.fSize, off, (tx->tx_data + tx->pos), rem, tx->sn, &pLen );
	uint8_t dlc_hw = Cus_Cantp_SizeToLinkLayerDLC( (Cus_CANTP_FrameSize_t)fs );
	uint32_t id = Cus_Cantp_GenerateId( tx->channel.addrMode, tx->channel.TA, tx->channel.SA, CUS_CANTP_TA_TYPE_PHYSICAL, 0 );
	CUS_CANTP_ASSERT( id != 0 );

	int8_t sret = tx->send( tx->user_ctx, id, frame, dlc_hw );
	if ( sret < 0 )
		return false;
	
	/*
	 * Frame accepted by hardware.
	 * Advance the send position and SN for the next CF,
	 * update the block counter, and start both the send-confirmation
	 * timer (N_As) and the separation-time timer (STmin).
	 * MainFunction will trigger the next CF once STmin expires.
	 */
	tx->tx_tag = (uint32_t)sret;
	tx->pos += pLen;
	tx->sn = (tx->sn + 1) & 0x0F;

	if ( tx->bs > 0 )	tx->bs_rem--;
	Cus_Cantp_TimerStart( &tx->t_n_as, CUS_CANTP_TIMEOUT_N_AS );
	Cus_Cantp_TimerStart( &tx->t_stmin, tx->stmin );	

	return true;
}


static bool 
SendFC( Cus_CANTP_RxConn_t *rx, Cus_CANTP_FLOWState_t flow )
{
	if ( !rx || ((rx->state != CUS_CANTP_STA_RX_FF) && (rx->state != CUS_CANTP_STA_RX_WAIT_CF)) )
		return false;

	uint8_t off = Cus_Cantp_GetPciOffset( rx->channel.addrMode );

	/* FC always classic CAN, 8 bytes */
	uint8_t frame[CUS_CANTP_SIZE_8] = { 0 };

	if ( rx->channel.addrMode == CUS_CANTP_ADDR_MODE_EXT )
		frame[0] = rx->peer_sa;   
	
	uint8_t fs = Cus_Cantp_BuildFC( frame, CUS_CANTP_SIZE_8, off, flow, rx->bs, rx->stmin );
	if ( !fs )	
		return false;

	/* FC CAN ID: TA=peer_sa, SA=my own SA */
	uint32_t id = Cus_Cantp_GenerateId( rx->channel.addrMode, rx->peer_sa, rx->channel.TA, CUS_CANTP_TA_TYPE_PHYSICAL, 0 );
	int8_t sret = rx->send( rx->user_ctx, id, frame, 8 );
	if ( sret < 0 )
		return false;

	/*
	 * Store the hardware tag so TxConfirm can match this FC frame,
	 * start the send-confirmation timer (N_Ar), and move to TX_FC.
	 */
	rx->fc_tag = (uint32_t)sret;
	rx->bs_rem = rx->bs; 
	Cus_Cantp_TimerStart( &rx->t_n_ar, CUS_CANTP_TIMEOUT_N_AR );
	rx->state = CUS_CANTP_STA_TX_FC;

	return true;
}


/**
 * @brief  Reset transient Tx fields after a transmission completes.
 *         Keeps channel config and callbacks intact.
 */
static void TxConnResetTx( Cus_CANTP_TxConn_t *conn )
{
	conn->tx_tag  = 0;
	conn->tx_data = NULL;
	conn->tot_len = 0;
	conn->pos     = 0;
	conn->bs      = 0;
	conn->stmin   = 0;
	conn->sn      = 0;
	conn->bs_rem  = 0;
	Cus_Cantp_TimerStop( &conn->t_n_as );
    Cus_Cantp_TimerStop( &conn->t_n_bs );
	Cus_Cantp_TimerStop( &conn->t_stmin );
}


static void RxConnResetRx( Cus_CANTP_RxConn_t *rx )
{
	rx->sn         = 0;
	rx->tot_len    = 0;
	rx->pos        = 0;
	rx->bs_rem     = 0;
	rx->fc_pending = 0;
	rx->fc_tag     = 0;
	rx->peer_sa    = 0;
	Cus_Cantp_TimerStop( &rx->t_n_ar );
	Cus_Cantp_TimerStop( &rx->t_n_cr );
}

/**
 * @brief  Initiate a CAN TP transmission on the given connection.
 *
 * Automatically chooses Single Frame (len ≤ max SF payload) or
 * First Frame (len > max SF payload, up to 4095 bytes).
 *
 * For SF: the frame is submitted to hardware immediately via SendFunc.
 *         Returns 1 on success, 0 if SendFunc fails (mailbox full).
 *
 * For FF: the first frame is submitted and the connection advances to
 *         TX_FF state.  Remaining data is sent by MainFunction.
 *
 * @param conn   TxConn created by Cus_Cantp_CreateTxConn()
 * @param data   source data (must remain valid until transmission finishes)
 * @param len    total byte count (1 ~ 4095)
 *
 * @return 1 = request submitted, 0 = failure (see return codes below)
 *
 * @retval  1  transmission started (SF or FF accepted by hardware)
 * @retval  0  invalid argument (conn/data NULL, len == 0, len > 4095)
 * @retval -1  connection busy (state != IDLE)
 * @retval -2  SendFunc failed (no mailbox available, try again later)
 * @retval -3  channel configuration invalid (address out of range)
 * @retval -4  the Functional TAType is not support FF frame.
 */
int8_t 
Cus_Cantp_StartTransmit( Cus_CANTP_TxConn_t *conn, const uint8_t *data, uint32_t len )
{
	if ( !conn || !data || len == 0 || len > 4095 )
		return 0;

	if ( conn->state != CUS_CANTP_STA_IDLE )
		return -1;

	uint8_t offset = Cus_Cantp_GetPciOffset( conn->channel.addrMode );
	uint32_t id = Cus_Cantp_GenerateId( conn->channel.addrMode, conn->channel.TA, conn->channel.SA, conn->channel.TAType, conn->channel.funcId );
	if ( !id )
		return -3;

	uint8_t frame[CUS_CANTP_SIZE_64] = { 0 };
	Cus_Cantp_WriteAddrPrefix( frame, &conn->channel );

	/* Try Single Frame first — BuildSF internally decides classic vs FD PCI. */
	uint8_t fs = Cus_Cantp_BuildSF( frame, conn->channel.fSize, offset, data, len );
	if ( fs )
	{
		/* Single Frame suffices for this communication.  */
		uint8_t dlc_hw = Cus_Cantp_SizeToLinkLayerDLC( (Cus_CANTP_FrameSize_t)fs );
		conn->state = CUS_CANTP_STA_TX_SF;
		int8_t sret = conn->send( conn->user_ctx, id, frame, dlc_hw );
		if ( sret < 0 )
			return -2;
		conn->tx_tag = (uint32_t)sret;

		conn->sn = 0;
		conn->pos = len;
	}
	else
	{
		if ( conn->channel.TAType == CUS_CANTP_TA_TYPE_FUNCTIONAL )
		{
			/* Functional addressing supports Single Frame only. 
			   Multi-frame (FF/CF) is not allowed. */
			return -4;
		}

		/* SF doesn't fit — go First Frame. */
		fs = Cus_Cantp_BuildFF( frame, conn->channel.fSize, offset, data, len );
		CUS_CANTP_ASSERT( fs != 0 );

		uint8_t dlc_hw = Cus_Cantp_SizeToLinkLayerDLC((Cus_CANTP_FrameSize_t)fs);

		conn->state = CUS_CANTP_STA_TX_FF;
		int8_t sret = conn->send( conn->user_ctx, id, frame, dlc_hw );
		if ( sret < 0 )
			return -2;
		conn->tx_tag = (uint32_t)sret;

		uint8_t ffPayload = conn->channel.fSize - offset - 2;   /* FF PCI = 2 bytes */
		conn->pos = ffPayload;
		conn->sn = 1;
	}

	conn->tot_len = len;
	conn->tx_data = data;
	Cus_Cantp_TimerStart( &conn->t_n_as, CUS_CANTP_TIMEOUT_N_AS );

	return 1;
}


/**
 * @brief  CAN TX confirmation callback — called from CAN TX completion ISR.
 *
 * The user's ISR reads the completed mailbox number and passes it here
 * together with the CAN device handle.  CANTP locates the matching
 * TxConn or RxConn and advances its state machine.
 *
 * @param bind_dev   CAN device handle (same as conn->bind_dev)
 * @param tag        Opaque tag returned by SendFunc (mailbox number)
 */
void 
Cus_Cantp_TxConfirm( void *bind_dev, uint32_t tag )
{
	/* Try TxConn first. */
	Cus_CANTP_TxConn_t *tx = Cus_Cantp_FindTxByTag(bind_dev, tag);
	if ( tx )
	{
		/* Find relevant connection. */
		switch (tx->state)
		{
			case CUS_CANTP_STA_TX_SF:
			{
				/* Single Frame confirmed — transmission complete. */
				tx->state = CUS_CANTP_STA_IDLE;
				TxConnResetTx( tx );
				Cus_Cantp_TimerStop( &tx->t_n_as );
				tx->tx_tag = 0;
				break;
			}	

			case CUS_CANTP_STA_TX_FF:
			{
				/* First Frame confirmed — now wait for Flow Control. */
				Cus_Cantp_TimerStop( &tx->t_n_as );
				Cus_Cantp_TimerStart( &tx->t_n_bs, CUS_CANTP_TIMEOUT_N_BS );
				tx->state = CUS_CANTP_STA_TX_WAIT_FC;
				tx->tx_tag = 0;
				break;
			}

			case CUS_CANTP_STA_TX_CF:
			{
				/* Consecutive Frame confirmed. */
				Cus_Cantp_TimerStop(&tx->t_n_as);
				uint32_t rem = tx->tot_len - tx->pos;
				if ( rem == 0 )
				{
					/* All data transmit OK! */
					tx->state = CUS_CANTP_STA_IDLE;
					TxConnResetTx(tx);
				}
				else if ( (tx->bs) > 0 && (tx->bs_rem == 0) )
				{
					/* Block exhausted — wait for next FC. */
					Cus_Cantp_TimerStart( &tx->t_n_bs, CUS_CANTP_TIMEOUT_N_BS );
					tx->state = CUS_CANTP_STA_TX_WAIT_FC;
				}
				/* else: stay in TX_CF — MainFunction will trigger next CF. */

				tx->tx_tag = 0;
				break;
			}
			
			default:	break;
		}
		return;
	}

	/* Then, try RxConn.(FC frame was sent, waiting for confirmation) */
	Cus_CANTP_RxConn_t *rx = Cus_Cantp_FindRxByTag( bind_dev, tag );
	if ( rx && rx->state == CUS_CANTP_STA_TX_FC )
	{
		/* Find relevant connection. */
		Cus_Cantp_TimerStop( &rx->t_n_ar );
		Cus_Cantp_TimerStart( &rx->t_n_cr, CUS_CANTP_TIMEOUT_N_CR );
		rx->state = CUS_CANTP_STA_RX_WAIT_CF;
		rx->fc_tag = 0;
	}
}


/**
 * @brief  Feed a received CAN frame into the CANTP stack.
 *
 * Called from the CAN RX interrupt handler.  The DLC value from the
 * CAN hardware register is converted internally to a frame byte
 * count (classic CAN DLC 0-8 → 0-8 bytes, FD DLC → via lookup).
 *
 * SF/FF/CF frames are routed to a matching RxConn; FC frames are
 * routed to a waiting TxConn.  Processing is done inline within the
 * ISR (copy, state update), but user callbacks (DataInd, ErrCb) are
 * deferred to Cus_Cantp_MainFunction().
 *
 * @param canId   CAN identifier (11-bit or 29-bit)
 * @param data    Raw CAN data field (up to 64 bytes)
 * @param dlc     Data Length Code from the hardware register (0-15)
 */
void 
Cus_Cantp_FeedFrame( uint32_t canId, const uint8_t *data, uint8_t dlc )
{
	if ( !data || !canId )
		return;

	/* ── RxConn: SF / FF / CF ── */
	Cus_CANTP_FrameSize_t fs = Cus_Cantp_LinkLayerDLCToSize( dlc );
	Cus_CANTP_RxConn_t *rx = Cus_Cantp_FindRxById( canId, (uint8_t *)data );
	if ( rx )
	{
		/* Find relevant connnection. */
		uint8_t pciOffset = Cus_Cantp_GetPciOffset( rx->channel.addrMode );
		Cus_CANTP_PCIType_t frameType = Cus_Cantp_GetPciType( data, pciOffset );
		switch (frameType)
		{
			case CUS_CANTP_PCI_SF:
			{
				Cus_Cantp_SFHandler( rx, data, fs );
				break;
			}

			case CUS_CANTP_PCI_CF:
			{
				Cus_Cantp_CFHandler( rx, data, fs );
				break;
			}

			case CUS_CANTP_PCI_FF:
			{
				Cus_Cantp_FFHandler( rx, data, canId, fs );
				break;
			}
			
			default:	break;
		}
	}
	else 
	{
		/* ── RxConn: Functional broadcast ──
		* No physical match → this may be a functional frame.
		* Scan all RxConns; dispatch SF to every conn whose funcId matches.
		* FF / CF on functional are silently dropped (ISO 15765-2: SF only). */
		for ( uint8_t i = 0; i < CUS_CANTP_MAX_RX; i++ )
		{
			Cus_CANTP_RxConn_t *frx = Cus_Cantp_GetRxConn( i );
			if ( frx->index == -1 )
				continue;

			/* Match: functional addressing + same funcId */
			if ( frx->channel.TAType != CUS_CANTP_TA_TYPE_FUNCTIONAL )
				continue;
			if ( frx->channel.funcId != canId )
				continue;

			uint8_t foff = Cus_Cantp_GetPciOffset( frx->channel.addrMode );
			Cus_CANTP_PCIType_t ftype = Cus_Cantp_GetPciType( data, foff );

			if ( ftype == CUS_CANTP_PCI_SF )
				Cus_Cantp_SFHandler( frx, data, fs );
			/* else: FF / CF / FC on functional — discard */
		}
	}

	/* ── TxConn: FC ── */
	/* Route to TxConn waiting for FC */
	Cus_CANTP_TxConn_t *tx = Cus_Cantp_FindTxById( canId, (uint8_t *)data );
	if ( tx && tx->state == CUS_CANTP_STA_TX_WAIT_FC )
	{
		uint8_t off = Cus_Cantp_GetPciOffset( tx->channel.addrMode );
		if ( Cus_Cantp_GetPciType( data, off ) == CUS_CANTP_PCI_FC )
			Cus_Cantp_FCHandler( tx, data );
	}
}


/**
 * @brief  CAN TP main state machine driver.
 *
 * Must be called periodically from a task context (RTOS thread or bare-metal
 * super-loop).  Typical call period: 1–10 ms.
 *
 * This function performs all deferred work that cannot run inside ISRs:
 *   - Timer expiry checks (N_As, N_Bs, N_Cr, N_Ar)
 *   - STmin-gated consecutive frame transmission
 *   - Deferred Flow Control frame transmission (fc_pending)
 *   - Receive-completion callbacks (DataInd)
 *   - Error callbacks (ErrCb)
 *
 * @note  Callbacks are invoked in the caller's context — ensure stack and
 *        RTOS primitives are available before calling this function.
 */
void 
Cus_Cantp_MainFunction( void )
{
	/* =================================================================
	 * Phase 1 — Critical section
	 *   - Timer expiry   → flag error (cb_pending), reset connection
	 *   - STmin / CF     → SendNextCF (mailbox write only)
	 *   - fc_pending     → SendFC (mailbox write only)
	 *   - Rx completion  → flag DataInd (cb_pending)
	 * No user callbacks are invoked inside the critical section.
	 * ================================================================= */
	Cus_Cantp_OS_EnterCritical();

	/* ── TxConn Pool ── */
	for ( uint8_t index = 0; index < CUS_CANTP_MAX_TX; index++ )
	{
		Cus_CANTP_TxConn_t *tx = Cus_Cantp_GetTxConn( index );
		if ( tx->index == -1 )
			continue;

		/* N_Bs timeout — peer did not send FC in time */
		if ( Cus_Cantp_TimerExpired( &tx->t_n_bs ) && tx->state == CUS_CANTP_STA_TX_WAIT_FC )
		{
			TxConnFlagError( tx, CUS_CANTP_ERR_NBS_TIMEOUT );
			continue;
		}

		/* N_As timeout — hardware did not confirm the last TX */
		if ( Cus_Cantp_TimerExpired( &tx->t_n_as ) && Cus_Cantp_TimerActive( &tx->t_n_as ) )
		{
			TxConnFlagError( tx, CUS_CANTP_ERR_NAS_TIMEOUT );
			continue;
		}

		/* Overflow during multi-frame transfer (peer sent OVERFLOW FC) */
		if ( (tx->state == CUS_CANTP_STA_IDLE) && (tx->tot_len > 0) && (tx->pos != tx->tot_len) )
		{
			TxConnFlagError( tx, CUS_CANTP_ERR_FLOW_OVFLW );
			continue;
		}

		/* STmin-gated consecutive frame transmission */
		if ( (tx->state == CUS_CANTP_STA_TX_CF) 
				&& !Cus_Cantp_TimerActive( &tx->t_n_as ) 
				&& ( !Cus_Cantp_TimerActive( &tx->t_stmin ) || Cus_Cantp_TimerExpired( &tx->t_stmin ) ) )
		{
			while ( SendNextCF( tx ) )
			{
				if ( Cus_Cantp_TimerActive( &tx->t_n_as ) )		break;
				if ( tx->pos == tx->tot_len )					break;
				if ( (tx->bs > 0) && (tx->bs_rem == 0) )		break;
				if ( (tx->stmin > 0) && Cus_Cantp_TimerActive( &tx->t_stmin ) )	break;
			}
		}
	}

	/* ── RxConn Pool ── */
	for ( uint8_t index = 0; index < CUS_CANTP_MAX_RX; index++ )
	{
		Cus_CANTP_RxConn_t *rx = Cus_Cantp_GetRxConn( index );
		if ( rx->index == -1 )
			continue;

		/* N_Ar timeout — FC frame TX was not confirmed */
		if ( Cus_Cantp_TimerExpired( &rx->t_n_ar ) && (rx->state == CUS_CANTP_STA_TX_FC) )
		{
			RxConnFlagError( rx, CUS_CANTP_ERR_NAS_TIMEOUT );
			continue;
		}

		/* N_Cr timeout — peer stopped sending CFs */
		if ( Cus_Cantp_TimerExpired( &rx->t_n_cr ) && (rx->state == CUS_CANTP_STA_RX_WAIT_CF) )
		{
			RxConnFlagError( rx, CUS_CANTP_ERR_NCR_TIMEOUT );
			continue;
		}

		/* Deferred Flow Control frame */
		if ( rx->fc_pending )
		{
			if ( (rx->state == CUS_CANTP_STA_RX_FF) || (rx->state == CUS_CANTP_STA_RX_WAIT_CF) )
			{
				SendFC( rx, CUS_CANTP_FLOW_CTS );
			}
			else if ( rx->state == CUS_CANTP_STA_IDLE )
			{
				SendFC( rx, CUS_CANTP_FLOW_OVERFLOW );
			}
			rx->fc_pending = 0;
		}

		/* Multi-frame or single-frame receive complete — flag DataInd for Phase 2 */
		if ( (rx->state == CUS_CANTP_STA_RX_CF_COMPLETE) || (rx->state == CUS_CANTP_STA_RX_SF) )
		{
			rx->cb_pending = 2;    /* bit1 = DataInd pending */
		}
	}

	Cus_Cantp_OS_ExitCritical();

	/* =================================================================
	 * Phase 2 — User callbacks (outside critical section)
	 *   - err()      for all connections flagged in Phase 1
	 *   - data_ind() for Rx connections with completed reassembly
	 * ================================================================= */

	/* ── TxConn: error callbacks ── */
	for ( uint8_t index = 0; index < CUS_CANTP_MAX_TX; index++ )
	{
		Cus_CANTP_TxConn_t *tx = Cus_Cantp_GetTxConn( index );
		if ( tx->index == -1 )
			continue;
		if ( !tx->cb_pending )
			continue;

		if ( tx->err )
			tx->err( (void *)tx, (Cus_CANTP_ErrCode_t)tx->cb_err_code );

		tx->cb_pending  = 0;
		tx->cb_err_code = 0;
	}

	/* ── RxConn: error & DataInd callbacks ── */
	for ( uint8_t index = 0; index < CUS_CANTP_MAX_RX; index++ )
	{
		Cus_CANTP_RxConn_t *rx = Cus_Cantp_GetRxConn( index );
		if ( rx->index == -1 )
			continue;
		if ( !rx->cb_pending )
			continue;

		/* bit0: error callback */
		if ( rx->cb_pending & 1 )
		{
			if ( rx->err )
				rx->err( (void *)rx, (Cus_CANTP_ErrCode_t)rx->cb_err_code );
		}

		/* bit1: DataInd callback (reassembly complete) */
		if ( rx->cb_pending & 2 )
		{
			if ( rx->data_ind )
				rx->data_ind( (void *)rx, rx->recv_buf, rx->tot_len );

			RxConnResetRx( rx );
			rx->state = CUS_CANTP_STA_IDLE;
		}

		rx->cb_pending  = 0;
		rx->cb_err_code = 0;
	}
}

