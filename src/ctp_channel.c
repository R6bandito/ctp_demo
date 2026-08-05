/**
 * @file ctp_channel.c
 * @brief CAN TP Channel Configuration & Addressing — Implementation
 * @author R6bandito
 * @date 2026-8
 *
 * DLC conversion tables, CAN-ID bit packing, and address-prefix insertion.
 */
#include "ctp_channel.h"


uint8_t 
Cus_Cantp_SizeToLinkLayerDLC( Cus_CANTP_FrameSize_t fSize )
{
	if ( fSize == CUS_CANTP_SIZE_8 )
	/* Classic CAN. TX_DL = 8 */
		return 8;

	/* CAN_FD case. Mapping fSize to TX_DLC */
	switch (fSize)
	{
		case CUS_CANTP_SIZE_12:  return 9;		
		case CUS_CANTP_SIZE_16:  return 10;		
		case CUS_CANTP_SIZE_20:  return 11;		
		case CUS_CANTP_SIZE_24:  return 12;		
		case CUS_CANTP_SIZE_32:  return 13;		
		case CUS_CANTP_SIZE_48:  return 14;		
		case CUS_CANTP_SIZE_64:  return 15;		
		
		default:	break;
	}

	return 8;
}


Cus_CANTP_FrameSize_t 
Cus_Cantp_LinkLayerDLCToSize( uint8_t DLC )
{
	switch (DLC)
	{
		case 8:		return CUS_CANTP_SIZE_8;	
		case 9:		return CUS_CANTP_SIZE_12;	
		case 10:	return CUS_CANTP_SIZE_16;
		case 11:	return CUS_CANTP_SIZE_20;
		case 12:	return CUS_CANTP_SIZE_24;
		case 13:	return CUS_CANTP_SIZE_32;
		case 14:	return CUS_CANTP_SIZE_48;
		case 15:	return CUS_CANTP_SIZE_64;
		
		default:	break;
	}

	
	return (Cus_CANTP_FrameSize_t)DLC;
}


void  
Cus_Cantp_WriteAddrPrefix( uint8_t *frame, const Cus_CANTP_ChannelCfg_t *cfg )
{
	if ( !frame || !cfg )	return;

	switch (cfg->addrMode)
	{
		/* Normal addressing mode: N_AI not present, no modification needed, return directly. */
		case CUS_CANTP_ADDR_MODE_NORMAL:  break;

		/* Extended addressing: TA occupies first data byte, PCI/payload shift to byte 1. */
		case CUS_CANTP_ADDR_MODE_EXT:	frame[0] = cfg->TA;		break;		
			
		default:	break;
	}
}


uint32_t 
Cus_Cantp_GenerateId( Cus_CANTP_AddrMode_t addrMode, uint8_t ta, uint8_t sa, uint8_t taType, uint32_t funcID )
{
	uint32_t CAN_id = 0;

	if ( taType == CUS_CANTP_TA_TYPE_FUNCTIONAL )
	{
		/* Functional addressing: target is a group of nodes, so CAN ID is the predefined functional identifier directly. */
		if ( funcID > 0x7FF )
			return 0;
		CAN_id = funcID;
	}
	else 
	{
		switch (addrMode)
		{
			case CUS_CANTP_ADDR_MODE_NORMAL:
			{
				/* Normal addressing mode: CAN ID is composed of SA (5 bits), TA (5 bits),
				and TAType (1 bit) shifted to fit 11-bit Standard CAN ID. */
				if ( (sa > 0x1F) || (ta > 0x1F) )
					return 0;  
				CAN_id = ((sa << 6) | (ta << 1) | (taType)) & 0x7FF;
				break;
			}
	
			case CUS_CANTP_ADDR_MODE_EXT:
			{
				/* Extended addressing mode: TA occupies the first data byte (frame[0]), so the CAN ID only encodes SA and TAType.
				ID layout: [ SA (bits 10-3) | TAType (bit 2) ].
				SA must fit within 8 bits (0x00-0xFF), as it is shifted to the high byte of the 11-bit ID. */
				if (sa > 0xFF)
					return 0;
				CAN_id = ((sa << 3) | (taType << 2)) & 0x7FF;
				break;
			}
	
			default:	break;
		}
	}

	return CAN_id;
}


uint8_t 
Cus_Cantp_ExtractSA( Cus_CANTP_AddrMode_t addrMode, uint32_t canId )
{
	uint8_t oSA = 0;
	switch (addrMode)
	{
		case CUS_CANTP_ADDR_MODE_NORMAL:
		{
			oSA = (canId >> 6) & 0x1F;
			break;
		}	

		case CUS_CANTP_ADDR_MODE_EXT:
		{
			oSA = (canId >> 3) & 0xFF;
			break;
		}
		
		default:	break;
	}

	return oSA;
}
