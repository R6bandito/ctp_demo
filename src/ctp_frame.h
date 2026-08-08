/**
 * @file ctp_frame.h
 * @brief CAN TP Frame Assembly & Parsing — Interface
 * @author R6bandito
 * @date 2026-8
 *
 * Pure functions for building and parsing ISO 15765-2 frames (SF/FF/CF/FC).
 * No global state, no hardware dependencies.  All addressing-mode variance
 * is captured by the @a pciOffset parameter.
 */
#ifndef __CANTP_FRAME_H__
#define __CANTP_FRAME_H__


#include "ctp_types.h"


/*============================================================================
 * Build — CANTP → raw 8‑byte / FD frame
 *============================================================================*/
uint8_t Cus_Cantp_BuildSF( uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                           const uint8_t *data, uint32_t len );

uint8_t Cus_Cantp_BuildFF( uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                           const uint8_t *data, uint32_t totLen );

uint8_t Cus_Cantp_BuildCF( uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                           const uint8_t *data, uint32_t remaining,
                           uint8_t snCode, uint8_t *pCopyLen );

uint8_t Cus_Cantp_BuildFC( uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                           Cus_CANTP_FLOWState_t flowState, uint8_t bs, uint8_t stmin );


/*============================================================================
 * Parse — raw frame → CANTP fields
 *============================================================================*/
Cus_CANTP_PCIType_t Cus_Cantp_GetPciType( const uint8_t *frame, uint8_t pciOffset );

uint8_t  Cus_Cantp_ParseSF( const uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                            const uint8_t **pData, uint8_t *pLen );

uint32_t Cus_Cantp_ParseFF( const uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                            const uint8_t **pData, uint8_t *pLen );

uint8_t  Cus_Cantp_ParseCF( const uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                            uint8_t *pSN, const uint8_t **pData, uint8_t *pLen );

uint8_t  Cus_Cantp_ParseFC( const uint8_t *frame, Cus_CANTP_FrameSize_t frameSize, uint8_t pciOffset,
                            Cus_CANTP_FLOWState_t *pFlowState, uint8_t *pBS, uint8_t *pSTmin );


#endif /* __CANTP_FRAME_H__ */
