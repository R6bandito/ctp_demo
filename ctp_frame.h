#ifndef __CANTP_FRAME_H__
#define __CANTP_FRAME_H__


#include "ctp_types.h"


uint8_t Cus_Cantp_BuildSF( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t *data, uint8_t len );
uint8_t Cus_Cantp_BuildFF( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t *data, uint32_t totLen );
uint8_t Cus_Cantp_BuildCF( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t *data, uint32_t remaining, uint8_t snCode, uint8_t *pCopyLen );
uint8_t Cus_Cantp_BuildFC( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, uint8_t flowState, uint8_t bs, uint8_t stmin );

Cus_CANTP_PCIType_t Cus_Cantp_GetPciType( const uint8_t *frame, uint8_t pciOffset );
uint8_t Cus_Cantp_ParseSF( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t **pData, uint8_t *pLen );
uint32_t Cus_Cantp_ParseFF( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t **pData, uint8_t *pLen );
uint8_t Cus_Cantp_ParseCF( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, uint8_t *pSN, const uint8_t **pData, uint8_t *pLen );
uint8_t Cus_Cantp_ParseFC( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, uint8_t *pFlowState, uint8_t *pBS, uint8_t *pSTmin );


#endif /* __CANTP_FRAME_H__ */
