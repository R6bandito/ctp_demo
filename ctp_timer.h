/**
 * @file ctp_timer.h
 * @brief CAN TP Timer Module - Interface
 * @author R6bandito
 * @date 2026-8
 *
 * This module provides a software timer service for the CAN TP protocol
 * stack. It implements a tick-based counter with start/stop/expiry checking.
 *
 * @note Timers are one-shot and must be re-started after expiry.
 *       All functions are thread-safe if Cus_Cantp_TimerTickInc() is called
 *       from a single ISR context.
 */

#ifndef __CANTP_TIMER_H__
#define __CANTP_TIMER_H__


#include <stdint.h>
#include <stdbool.h>


typedef     uint32_t    Cus_CANTP_TickType_t;


/**
 * @brief Timer object structure
 *
 * Holds the absolute tick value when the timer will expire.
 * A deadline of 0 indicates the timer is inactive (stopped).
 */
typedef struct 
{
    Cus_CANTP_TickType_t deadLineTick;

} Cus_CANTP_Timer_t;


void Cus_Cantp_TimerInit( void );
void Cus_Cantp_TimerTickInc( void );
Cus_CANTP_TickType_t Cus_Cantp_TimerNow( void );

void Cus_Cantp_TimerStart( Cus_CANTP_Timer_t *timer, Cus_CANTP_TickType_t ms );
void Cus_Cantp_TimerStop( Cus_CANTP_Timer_t *timer );
bool Cus_Cantp_TimerExpired( Cus_CANTP_Timer_t *timer );
bool Cus_Cantp_TimerActive( const Cus_CANTP_Timer_t *t );


#endif /* __CANTP_TIMER_H__ */

