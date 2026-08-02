/**
 * @file ctp_channel.h
 * @brief CAN TP Channel Configuration & Addressing — Interface
 * @author R6bandito
 * @date 2026-8
 *
 * Channel config structure, CAN-ID generation, SA extraction,
 * DLC ↔ frame-size conversion, and address-prefix helpers.
 */
#ifndef __CANTP_CHANNEL_H__
#define __CANTP_CHANNEL_H__


#include "ctp_types.h"


typedef struct 
{
	Cus_CANTP_AddrMode_t addrMode;
	uint8_t SA;
	uint8_t TA;
	Cus_CANTP_TAType_t TAType;
	Cus_CANTP_FrameSize_t fSize;
	uint32_t funcId;

} Cus_CANTP_ChannelCfg_t;


static inline uint8_t Cus_Cantp_GetPciOffset( Cus_CANTP_AddrMode_t mode ) {	return (mode == CUS_CANTP_ADDR_MODE_EXT) ? 1 : 0; }

uint32_t Cus_Cantp_GenerateId( Cus_CANTP_AddrMode_t addrMode, uint8_t ta, uint8_t sa, uint8_t taType, uint32_t funcID );

uint8_t Cus_Cantp_SizeToLinkLayerDLC( Cus_CANTP_FrameSize_t fSize );

void  Cus_Cantp_WriteAddrPrefix( uint8_t *frame, const Cus_CANTP_ChannelCfg_t *cfg );

Cus_CANTP_FrameSize_t Cus_Cantp_LinkLayerDLCToSize( uint8_t DLC );

uint8_t Cus_Cantp_ExtractSA( Cus_CANTP_AddrMode_t addrMode, uint32_t canId );


#endif /* __CANTP_CHANNEL_H__ */
