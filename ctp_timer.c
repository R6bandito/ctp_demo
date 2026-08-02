/**
 * @file ctp_timer.c
 * @brief CAN TP Timer Module - Implementation
 * @author R6bandito
 * @date 2026-8
 *
 * Provides a simple software timer service based on a monotonically
 * increasing tick counter. All timers are one-shot and non-blocking.
 * The timer resolution is determined by the caller's tick interrupt
 * (usually 1ms).
 *
 * @note This is a lightweight implementation without dynamic memory
 *       allocation or linked lists. Each timer is a standalone object
 *       managed by the caller.
 */


#include "ctp_timer.h"


/**
 * @brief Global tick counter (monotonically increasing)
 *
 * Incremented by Cus_Cantp_TimerTickInc() which should be called from
 * the system tick interrupt (e.g., SysTick_Handler) at a fixed interval.
 * Rolling over is handled gracefully in Expired() via signed comparison.
 */
static volatile Cus_CANTP_TickType_t gs_Tick;


/**
 * @brief Get the current tick value
 *
 * @return Current system tick count
 */
Cus_CANTP_TickType_t 
Cus_Cantp_TimerNow( void )
{
    return gs_Tick;
}


/**
 * @brief Increment the system tick counter
 *
 * This function must be called regularly (e.g., from the SysTick ISR)
 * to advance the time base. The tick interval defines the timer resolution.
 */
void 
Cus_Cantp_TimerTickInc( void )
{
    gs_Tick++;
}


/**
 * @brief Start a timer with a given timeout
 *
 * Sets the timer's deadline to (current_tick + timeout_ms).
 * If the timer was previously running, it is restarted with the new timeout.
 *
 * @param timer  Pointer to the timer object to start
 * @param ms     Timeout value in tick units (milliseconds if tick = 1ms)
 */
void 
Cus_Cantp_TimerStart( Cus_CANTP_Timer_t *timer, Cus_CANTP_TickType_t ms )
{
    timer->deadLineTick = gs_Tick + ms;
}


/**
 * @brief Stop a running timer
 *
 * Disables the timer by setting its deadline to 0.
 * The timer can be restarted later with a new timeout.
 *
 * @param timer  Pointer to the timer object to stop
 */
void 
Cus_Cantp_TimerStop( Cus_CANTP_Timer_t *timer )
{
    timer->deadLineTick = 0;
}


/**
 * @brief Initialize the timer module
 *
 * Resets the global tick counter to 0. Should be called before any other
 * timer functions, typically at system boot.
 */
void 
Cus_Cantp_TimerInit( void )
{
    gs_Tick = 0;
}


bool 
Cus_Cantp_TimerActive( const Cus_CANTP_Timer_t *t ) 
{
    return t->deadLineTick != 0;
}


/**
 * @brief Check if a timer has expired
 *
 * A timer is considered expired when (current_tick - deadline_tick) >= 0.
 * The signed comparison handles tick counter wrap-around correctly.
 * A timer with deadline == 0 is considered inactive and never expires.
 *
 * @param timer  Pointer to the timer object to check
 * @return true if the timer has expired, false otherwise
 */
bool 
Cus_Cantp_TimerExpired( Cus_CANTP_Timer_t *timer )
{
    if ( timer->deadLineTick == 0 )
        /* This timer does not been enabled. */
        return false;

    return ((int32_t)(gs_Tick - timer->deadLineTick) >= 0);
}


