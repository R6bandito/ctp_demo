#include "ctp_os.h"



#if defined(CUS_CANTP_OS)

/* ************************************* */
static const uint32_t mbItemSize       = sizeof( Cus_Cantp_OSReq_t );
static const uint32_t mbTimeout_noBlock = 0u;
static void          *pMailBox;
static void __cantp_kernel_thread( void *parameter );
/* ************************************* */


void 
Cus_CANTP_OS_Init( void )
{
	int8_t res = -1;
	res = Cus_Cantp_OS_MailboxCreate( CUS_CANTP_MAILBOX_LENGTH, mbItemSize, &pMailBox );
	CUS_CANTP_ASSERT( res >= 0 );

	res = Cus_Cantp_OS_ThreadNew( (Cus_CANTP_Thread_t)__cantp_kernel_thread, NULL,
	                               CUS_CANTP_MAILBOX_STACK_SIZE, CUS_CANTP_MAILBOX_PRIORITY );
	CUS_CANTP_ASSERT( res >= 0 );
}


static void 
__cantp_kernel_thread( void *parameter )
{
	(void)parameter;
	Cus_Cantp_OSReq_t Req;

	while (1)
	{
		while ( Cus_Cantp_OS_MailboxFetch( pMailBox, &Req, mbTimeout_noBlock ) == 0 )
		{
			switch ( Req.type )
			{
				case CUS_CANTP_REQ_SEND:	
					Cus_Cantp_StartTransmit( Req.u.send.conn,
					                          Req.u.send.data,
					                          Req.u.send.len );
					break;

				case CUS_CANTP_REQ_CREATE_RX:
					*Req.u.create_rx.out_conn = Cus_Cantp_CreateRxConn(
						Req.u.create_rx.channel, 
						Req.u.create_rx.bind_dev, 
						Req.u.create_rx.user_ctx, 
						Req.u.create_rx.send_fn, 
						Req.u.create_rx.err_fn );

					Cus_Cantp_RxConnBindBuf( *Req.u.create_rx.out_conn, 
					                          Req.u.create_rx.pbuf, 
					                          Req.u.create_rx.size, 
					                          Req.u.create_rx.bs, 
					                          Req.u.create_rx.stmin, 
					                          Req.u.create_rx.recv_fn );

					*Req.p_done = 1;	
					break;

				case CUS_CANTP_REQ_CREATE_TX:
					*Req.u.create_tx.out_conn = Cus_Cantp_CreateTxConn(
						Req.u.create_tx.channel, 
						Req.u.create_tx.bind_dev, 
						Req.u.create_tx.user_ctx, 
						Req.u.create_tx.send_fn, 
						Req.u.create_tx.err_fn );
					*Req.p_done = 1;
					break;

				case CUS_CANTP_REQ_RELEASE:
					Cus_Cantp_ReleaseConn( Req.u.release.conn,
					                        Req.u.release.conn_type );
					*Req.p_done = 1;
					break;
				
				default:	break;
			}

			Cus_Cantp_MainFunction();
		}

		Cus_Cantp_MainFunction();

		Cus_Cantp_OS_MailboxFetch( pMailBox, &Req, 1u );
	}
}


/* ── Internal: submit a sync request and poll for TP-thread completion ── */
static int8_t __submit_and_wait( Cus_Cantp_OSReq_t *req, uint32_t timeoutMs )
{
	if ( Cus_Cantp_OS_MailboxSend( pMailBox, req, timeoutMs ) != 0 )
		return -1;

	uint32_t elapsed = 0;
	while ( !*req->p_done )
	{
		if ( elapsed >= timeoutMs )
			return -3;   /* operation timed out */
		Cus_Cantp_OS_Delay( 1 );
		elapsed++;
	}
	return 0;
}


int8_t 
Cus_Cantp_OS_StartTransmit( Cus_CANTP_TxConn_t *conn, const uint8_t *data,
                             uint32_t len, uint32_t timeoutMs )
{
	if ( !conn || !data || len == 0 )
		return -2;

	Cus_Cantp_OSReq_t Req = {
		.type   = CUS_CANTP_REQ_SEND,
		.p_done = NULL,
		.u      = { .send = { .conn = conn, .data = data, .len = len } }
	};

	return Cus_Cantp_OS_MailboxSend( pMailBox, &Req, timeoutMs );
}


int8_t 
Cus_Cantp_OS_CreateRxConn_Bind(
	Cus_CANTP_ChannelCfg_t  channel,
	void                   *bind_dev,
	void                   *user_ctx,
	Cus_Cantp_SendFunc_t    send_fn,
	Cus_Cantp_ErrCb_t       err_fn,
	Cus_Cantp_DataInd_t     recv_fn,
	uint8_t                *pbuf,
	uint32_t                size,
	uint8_t                 bs,
	uint8_t                 stmin,
	uint32_t                timeoutMs,
	const Cus_CANTP_RxConn_t **out_conn )
{
	if ( !bind_dev || !pbuf || !out_conn )
		return -2;
	if ( !send_fn || !recv_fn )
		return -2;

	volatile uint8_t flag = 0;

	Cus_Cantp_OSReq_t Req = {
		.type   = CUS_CANTP_REQ_CREATE_RX,
		.p_done = (volatile uint8_t *)&flag,
		.u      = { .create_rx = {
			.channel  = channel,
			.bind_dev = bind_dev,
			.user_ctx = user_ctx,
			.send_fn  = send_fn,
			.err_fn   = err_fn,
			.recv_fn  = recv_fn,
			.pbuf     = pbuf,
			.size     = size,
			.bs       = bs,
			.stmin    = stmin,
			.out_conn = out_conn
		}}
	};

	return __submit_and_wait( &Req, timeoutMs );
}


int8_t 
Cus_Cantp_OS_CreateTxConn(
	Cus_CANTP_ChannelCfg_t  channel,
	void                   *bind_dev,
	void                   *user_ctx,
	Cus_Cantp_SendFunc_t    send_fn,
	Cus_Cantp_ErrCb_t       err_fn,
	uint32_t                timeoutMs,
	const Cus_CANTP_TxConn_t **out_conn )
{
	if ( !bind_dev || !out_conn || !send_fn )
		return -2;

	volatile uint8_t flag = 0;

	Cus_Cantp_OSReq_t Req = {
		.type   = CUS_CANTP_REQ_CREATE_TX,
		.p_done = (volatile uint8_t *)&flag,
		.u      = { .create_tx = {
			.channel  = channel,
			.bind_dev = bind_dev,
			.user_ctx = user_ctx,
			.send_fn  = send_fn,
			.err_fn   = err_fn,
			.out_conn = out_conn
		}}
	};

	return __submit_and_wait( &Req, timeoutMs );
}


int8_t 
Cus_Cantp_OS_ReleaseConn( void *conn, Cus_CANTP_ConnType_t conn_type, uint32_t timeoutMs )
{
	if ( !conn )
		return -2;

	volatile uint8_t flag = 0;

	Cus_Cantp_OSReq_t Req = 
	{
		.type   = CUS_CANTP_REQ_RELEASE,
		.p_done = (volatile uint8_t *)&flag,
		.u      = { .release = { .conn = conn, .conn_type = conn_type } }
	};

	return __submit_and_wait( &Req, timeoutMs );
}


#endif /* CUS_CANTP_OS */
