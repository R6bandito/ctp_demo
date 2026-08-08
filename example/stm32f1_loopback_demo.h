/**
 * @file    stm32f1_loopback_demo.h
 * @brief   EasyCantp — STM32F1 CAN Loopback Demo Header
 * @author  R6bandito
 * @date    2026-08
 *
 * Platform: STM32F103XE, CAN1 (PA11=RX, PA12=TX), 500 kbps
 * Dependencies: ctp.h, stm32f1xx_hal.h
 * Test coverage: loopback, Normal / EXT addressing, SF / FF (up to 4095B), BS flow control
 */
#ifndef __STM32F1_LOOP_H__
#define __STM32F1_LOOP_H__


/* ********************************* */
#define DEMO_CAN_PORT            (GPIOA)
#define DEMO_CAN_TX              (GPIO_PIN_12)
#define DEMO_CAN_RX              (GPIO_PIN_11)
#define DEMO_CAN_CLK_EN()        do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while(0)


void test_env_init( void );
void test_run_all( void );
void test_normal_sf_7b( void );
void test_normal_ff_35b_bs0( void );
void test_normal_ff_35b_bs2( void );
void test_normal_ff_4095( void );
void test_ext_ff_4095b( void );
void test_ext_sf_6b( void );
void test_func_sf_broadcast( void );
/* ********************************* */

#endif /* __STM32F1_LOOP_H__ */
