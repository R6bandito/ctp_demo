#ifndef __CANTP_OS_H__
#define __CANTP_OS_H__

#include <stdint.h>

#if (1)

	#include "ctp.h"

	#define CUS_CANTP_OS

	#ifndef CUS_CANTP_MAILBOX_LENGTH
		#define CUS_CANTP_MAILBOX_LENGTH		(8u)
	#endif /* CUS_CANTP_MAILBOX_LENGTH */

	#ifndef CUS_CANTP_MAILBOX_STACK_SIZE		
		#define CUS_CANTP_MAILBOX_STACK_SIZE	(512u)
	#endif /* CUS_CANTP_MAILBOX_STACK_SIZE */

	#ifndef CUS_CANTP_MAILBOX_PRIORITY
		#define CUS_CANTP_MAILBOX_PRIORITY		(3u)
	#endif /* CUS_CANTP_MAILBOX_PRIORITY */

	typedef struct Cus_CANTP_TxConn  Cus_CANTP_TxConn_t;

    typedef enum 
	{
        CUS_CANTP_REQ_SEND,        /* StartTransmit                    */
        CUS_CANTP_REQ_CREATE_RX,   /* CreateRxConn + RxConnBindBuf    */
        CUS_CANTP_REQ_CREATE_TX,   /* CreateTxConn                     */
        CUS_CANTP_REQ_RELEASE,     /* ReleaseConn                      */

    } Cus_Cantp_ReqType_t;

	typedef struct 
	{
		Cus_Cantp_ReqType_t  type;

		/*
         * Common completion flag — set by TP thread, polled by caller.
         *  0 = pending, 1 = done.
         * For CREATE requests the result pointer is also valid once done == 1.*/
        volatile uint8_t * p_done;

		union {
			/* ── SEND ─────────────────────────────────────── */
			struct 
			{
				uint32_t len;
				const uint8_t *data;
				Cus_CANTP_TxConn_t *conn;
			} send;

			/* ── CREATE_RX ────────────────────────────────── */
			struct 
			{
				Cus_CANTP_ChannelCfg_t channel;
				void *bind_dev;
				void *user_ctx;
				Cus_Cantp_SendFunc_t send_fn;
				Cus_Cantp_DataInd_t recv_fn;
				Cus_Cantp_ErrCb_t err_fn;
				uint8_t *pbuf;
				uint32_t size;
				uint8_t bs;
				uint8_t stmin;
				/* out: TP thread writes the created conn here */
				const Cus_CANTP_RxConn_t **out_conn;
			} create_rx;

			/* ── CREATE_TX ────────────────────────────────── */
			struct 
			{
				Cus_CANTP_ChannelCfg_t channel;
				void *bind_dev;
				void *user_ctx;
				Cus_Cantp_SendFunc_t send_fn;
				Cus_Cantp_ErrCb_t err_fn;
				const Cus_CANTP_TxConn_t **out_conn;
			} create_tx;

			/* ── RELEASE ──────────────────────────────────── */
			struct 
			{
				void *conn;
				Cus_CANTP_ConnType_t conn_type;
			} release;
		} u;

	} Cus_Cantp_OSReq_t;


	typedef void (*Cus_CANTP_Thread_t)(void *arg);
	extern int8_t Cus_Cantp_OS_ThreadNew( Cus_CANTP_Thread_t pThread, void *pArg, uint32_t stackSize, uint32_t priority );
	extern int8_t Cus_Cantp_OS_MailboxCreate( uint32_t itemCount, uint32_t itemSize, void **pBox );
	extern int8_t Cus_Cantp_OS_MailboxSend( void *mailBox, const void *item, uint32_t timeoutMs );
	extern int8_t Cus_Cantp_OS_MailboxFetch( void *mailBox, void *item, uint32_t timeoutMs );
	extern void Cus_Cantp_OS_Delay( uint32_t delayMs );
	extern void Cus_Cantp_OS_EnterCritical( void );
	extern void Cus_Cantp_OS_ExitCritical( void );

	void Cus_CANTP_OS_Init( void );

	/*============================================================================
     * In an OS environment (task context), 
	 * use the following OS-safe APIs instead of directly touching Cus_Cantp_SendReq_t.
     *============================================================================*/
    
    /**
     * @brief  Fire-and-forget send. Returns immediately; result via err callback.
     * @retval  0  request queued
     * @retval -1  mailbox full
	 * @retval -2  param wrong
     */
    int8_t Cus_Cantp_OS_StartTransmit( Cus_CANTP_TxConn_t *conn, const uint8_t *data, uint32_t len, uint32_t timeoutMs );

	/**
	* @brief  Synchronous create + bind an Rx connection. Blocks caller.
	* @param out_conn  [out] created RxConn (valid when return == 0)
	* @retval  0  success
	* @retval -1  mailbox full
	* @retval -2  param wrong
	*/
	int8_t Cus_Cantp_OS_CreateRxConn_Bind(
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
	uint32_t 				timeoutMs,
	const Cus_CANTP_RxConn_t **out_conn );

	/**
	* @brief  Synchronous create a Tx connection. Blocks caller.
	* @param timeoutMs  max wait time (ms)
	* @param out_conn  [out] created TxConn (valid when return == 0)
	* @retval  0  success
	* @retval -1  mailbox full
	* @retval -2  param wrong
	* @retval -3  operation timeout
	*/
	int8_t Cus_Cantp_OS_CreateTxConn(
	Cus_CANTP_ChannelCfg_t  channel,
	void                   *bind_dev,
	void                   *user_ctx,
	Cus_Cantp_SendFunc_t    send_fn,
	Cus_Cantp_ErrCb_t       err_fn,
	uint32_t                timeoutMs,
	const Cus_CANTP_TxConn_t **out_conn );

	/**
	* @brief  Synchronous release a connection. Blocks caller.
	* @param timeoutMs  max wait time (ms)
	* @retval  0  success
	* @retval -1  mailbox full
	* @retval -2  param wrong
	* @retval -3  operation timeout
	*/
	int8_t Cus_Cantp_OS_ReleaseConn( void *conn, Cus_CANTP_ConnType_t conn_type, uint32_t timeoutMs );

#endif 


#endif /* __CANTP_OS_H__ */
