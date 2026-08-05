/**
 * @file CANTP_types.h
 * @brief CAN TP (ISO 15765-2) Lightweight Implementation - Type Definitions
 * @author R6bandito
 * @date 2026-8
 *
 * This header defines all type definitions, enumerations, and callback
 * prototypes used by the CAN TP protocol stack.
 */

#ifndef __CANTP_TYPES_H__
#define __CANTP_TYPES_H__


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/*============================================================================
 * Debug & Assert
 *============================================================================*/
/**
 * @brief  CANTP library assertion macro.
 *
 * Checks condition `expr` at runtime.  On failure:
 *   - Calls CUS_CANTP_ASSERT_HOOK() (user-overridable debug print).
 *   - Triggers a debugger breakpoint (BKPT on Cortex-M).
 *   - Enters an infinite loop.
 *
 * To provide a custom debug print, #define CUS_CANTP_ASSERT_HOOK(msg)
 * before including this header.  The default hook is a no-op.
 *
 * Usage example with printf:
 * @code
 *   #define CUS_CANTP_ASSERT_HOOK(msg)  printf("[%s:%d] %s\n", __FILE__, __LINE__, msg)
 *   #include "ctp_types.h"
 * @endcode
 */
#ifndef CUS_CANTP_ASSERT_HOOK
  #define CUS_CANTP_ASSERT_HOOK(msg)  ((void)0)
#endif

#define CUS_CANTP_ASSERT(expr)                                           \
      do {                                                                   \
          if (!(expr)) {                                                    \
              CUS_CANTP_ASSERT_HOOK("CANTP ASSERT: " #expr);               \
              __asm volatile ("bkpt 0");  /* ARM Cortex-M breakpoint */      \
              for (;;) { __asm volatile ("nop"); }                           \
          }                                                                  \
      } while (0)


/* Forward declarations for connection objects */
typedef struct	Cus_CANTP_TxConn	Cus_CANTP_TxConn_t;		/**< Transmit connection context */
typedef struct	Cus_CANTP_RxConn	Cus_CANTP_RxConn_t;		/**< Receive connection context */


/**
 * @brief CAN TP protocol state machine states
 *
 * Defines all possible states for a CAN TP connection (both Tx and Rx).
 * States are grouped by direction (TX vs RX) and sub-state (SF/FF/CF/FC).
 */
typedef enum 
{
	CUS_CANTP_STA_IDLE = 0,		/**< Idle state: no active transmission */

	/* ----- Transmit side states ----- */
	CUS_CANTP_STA_TX_SF,			/**< Transmitting Single Frame */
	CUS_CANTP_STA_TX_FF,			/**< Transmitting First Frame (multi-frame start) */
	CUS_CANTP_STA_TX_CF,			/**< Transmitting Consecutive Frame (data continuation) */
	CUS_CANTP_STA_TX_WAIT_FC,		/**< Waiting for Flow Control frame from receiver */

	/* ----- Receive side states ----- */
	CUS_CANTP_STA_RX_SF,			/**< Receiving Single Frame (<= 8 bytes) */
	CUS_CANTP_STA_RX_FF,			/**< Receiving First Frame (multi-frame start) */
	CUS_CANTP_STA_RX_CF_COMPLETE,	/**< CF reception complete, MainFunction → DataInd */
	CUS_CANTP_STA_RX_WAIT_CF,		/**< Waiting for next Consecutive Frame */
	CUS_CANTP_STA_TX_FC,			/**< Transmitting Flow Control frame (receiver side) */

} Cus_CANTP_State_t;


/**
 * @brief CAN TP error codes
 *
 * Error conditions that can be reported via the error callback.
 * Timeout errors correspond to N_As/N_Ar/N_Bs timers as per ISO 15765-2.
 */
typedef enum {
	CUS_CANTP_ERR_NONE = 0,
	CUS_CANTP_ERR_NBS_TIMEOUT,
	CUS_CANTP_ERR_NCR_TIMEOUT,
	CUS_CANTP_ERR_NAS_TIMEOUT,
	CUS_CANTP_ERR_FLOW_OVFLW,
	CUS_CANTP_ERR_SN_MISMATCH,
	CUS_CANTP_ERR_TX_FAILED,
	CUS_CANTP_ERR_RX_BUFFER_FULL,

} Cus_CANTP_ErrCode_t;


/**
 * @brief Flow Control status values (ISO 15765-2 FS field)
 *
 * Sent by receiver in FC frames to control sender's transmission pace.
 *
 * @note This implementation is a simplified subset of the ISO 15765-2 standard.
 *       The WAIT state is NOT implemented. Receiving a WAIT flow control frame
 *       will be treated as OVERFLOW, causing the sender to abort the ongoing
 *       transmission immediately.
 */
typedef enum 
{
	CUS_CANTP_FLOW_CTS = 0,
	/* CUS_CANTP_FLOW_WAIT = 1, */		/* NOT implemented! */
	CUS_CANTP_FLOW_OVERFLOW = 2,

} Cus_CANTP_FLOWState_t;


/**
 * @brief CAN TP addressing modes (ISO 15765-2)
 *
 * Defines how the CAN identifier (CAN ID) is interpreted and used
 * to distinguish between different connections or nodes.
 *
 * @note Currently only NORMAL mode is implemented. EXT and MIX modes
 *       are reserved for future expansion.
 */
typedef enum 
{
	CUS_CANTP_ADDR_MODE_NORMAL = 0,
	CUS_CANTP_ADDR_MODE_EXT	   = 1,
	/* CUS_CANTP_ADDR_MODE_MIX  NOT implemented! */

} Cus_CANTP_AddrMode_t;


/**
 * @brief CAN TP Target Address (TA) type
 *
 * Defines whether the target node is addressed individually or as a group.
 * This field is used in the N_AI (Address Information) to select the
 * appropriate routing behavior.
 *
 * - Physical: Point-to-point communication with a single specific ECU.
 * - Functional: Broadcast/multicast communication to a group of ECUs
 *   (e.g., all nodes on the network or a specific functional class).
 */
typedef enum 
{
	CUS_CANTP_TA_TYPE_PHYSICAL = 0,
	CUS_CANTP_TA_TYPE_FUNCTIONAL = 1,

} Cus_CANTP_TAType_t;


/**
 * @brief Protocol Control Information (PCI) types
 *
 * Identifies the type of CAN TP frame based on the first byte's high nibble.
 * Used to decode the incoming CAN message and route to the correct state handler.
 */
typedef enum 
{
	CUS_CANTP_PCI_SF = 0,
	CUS_CANTP_PCI_FF = 1,
	CUS_CANTP_PCI_CF = 2,
	CUS_CANTP_PCI_FC = 3,
	CUS_CANTP_PCI_UNKNOWN = 0xFF,

} Cus_CANTP_PCIType_t;


/**
 * @brief CAN TP frame sizes (classic CAN and CAN FD)
 *
 * Defines the byte-length values that a CAN frame can carry,
 * covering both classic CAN (8 bytes) and CAN FD (12–64 bytes).
 * Used in channel configuration and DLC ↔ frame-size conversion.
 */
typedef enum 
{
	CUS_CANTP_SIZE_8  = 8,
	CUS_CANTP_SIZE_12 = 12,
	CUS_CANTP_SIZE_16 = 16,
	CUS_CANTP_SIZE_20 = 20,
	CUS_CANTP_SIZE_24 = 24,
	CUS_CANTP_SIZE_32 = 32,
	CUS_CANTP_SIZE_48 = 48,
	CUS_CANTP_SIZE_64 = 64,

} Cus_CANTP_FrameSize_t;


/**
 * @brief Connection type discriminator
 *
 * Used by Cus_Cantp_ReleaseConn() to determine which pool
 * (TxPool or RxPool) the connection belongs to.
 */
typedef enum 
{
	CUS_CANTP_CONN_TYPE_TX,
	CUS_CANTP_CONN_TYPE_RX,

} Cus_CANTP_ConnType_t;


/*============================================================================
* Protocol Timing Constants (ISO 15765-2)
*============================================================================*/
#ifndef CUS_CANTP_TIMEOUT_N_AS
	#define CUS_CANTP_TIMEOUT_N_AS   1000U   /* send confirmation timeout (ms) */
#endif
#ifndef CUS_CANTP_TIMEOUT_N_BS
	#define CUS_CANTP_TIMEOUT_N_BS   5000U   /* wait for Flow Control (ms)     */
#endif
#ifndef CUS_CANTP_TIMEOUT_N_AR
	#define CUS_CANTP_TIMEOUT_N_AR   1000U   /* FC send confirmation (ms)      */
#endif
#ifndef CUS_CANTP_TIMEOUT_N_CR
	#define CUS_CANTP_TIMEOUT_N_CR   1000U   /* wait for Consecutive Frame (ms)*/
#endif


/*============================================================================
 * Callback Function Types
 *============================================================================*/
/**
 * @brief CAN frame transmission callback
 *
 * Called by CANTP when a frame needs to be transmitted over CAN.
 * The implementation must write the frame to a hardware mailbox or
 * queue and return an opaque tag that identifies this transmission.
 *
 * @param userCtx   User context (passed through from the connection)
 * @param canId     CAN identifier (11-bit or 29-bit)
 * @param data       Pointer to data buffer to send
 * @param dlc        Data length code
 * @retval  -1       Transmit failed (mailbox full or hardware error)
 * @retval >= 0      Opaque tag identifying this transmission (e.g.
 *                   CAN mailbox number on STM32).  The same value is
 *                   passed back via Cus_Cantp_TxConfirm() to match
 *                   the completion event to this connection.
 */
typedef int8_t ( *Cus_Cantp_SendFunc_t )( void *userCtx, uint32_t canId, const uint8_t *data, uint8_t dlc );


/**
 * @brief Data indication callback (called when a complete message is reassembled)
 *
 * @param rxConn    Pointer to the receive connection object
 * @param rxData    Pointer to the reassembled complete message data
 * @param dataLen    Length of the received data in bytes
 */
typedef void ( *Cus_Cantp_DataInd_t )( void *rxConn, const uint8_t *rxData, uint32_t dataLen );


/**
 * @brief Error notification callback
 *
 * Called when a protocol error occurs (timeout, overflow, sequence mismatch, etc.)
 * The connection may be automatically reset or kept alive depending on error severity.
 *
 * @param conn       Pointer to the connection object (Tx or Rx)
 * @param err        Error code indicating the failure type
 */
typedef void ( *Cus_Cantp_ErrCb_t )( void *conn, Cus_CANTP_ErrCode_t err );


#endif /* __CANTP_TYPES_H__ */


