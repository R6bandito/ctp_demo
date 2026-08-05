/**
 * @file ctp.h
 * @brief CAN TP (ISO 15765-2) Lightweight Protocol Stack — Aggregate Header
 * @author R6bandito
 * @date 2026-8
 *
 * Include this single header to pull in the entire CAN TP stack.
 * All configuration macros (CUS_CANTP_MAX_TX, CUS_CANTP_MAX_RX, etc.)
 * should be defined before including this file, or the defaults are used.
 *
 * Usage:
 * @code
 *   #define CUS_CANTP_MAX_TX   4
 *   #define CUS_CANTP_MAX_RX   4
 *   #include "ctp.h"
 * @endcode
 */
#ifndef __CANTP_H__
#define __CANTP_H__


#include "ctp_types.h"
#include "ctp_frame.h"
#include "ctp_timer.h"
#include "ctp_channel.h"
#include "ctp_conn.h"
#include "ctp_fsm.h"


#endif /* __CANTP_H__ */
