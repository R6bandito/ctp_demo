/**
 * @file ctp_frame.c
 * @brief CAN TP Frame Assembly & Parsing — Implementation
 * @author R6bandito
 * @date 2026-8
 *
 * ISO 15765-2 frame construction / deconstruction.
 * All functions are stateless and operate purely on the supplied buffers.
 */
#include "ctp_frame.h"
#include <string.h>


uint8_t 
Cus_Cantp_BuildSF( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t *data, uint8_t len )
{
	if ( !frame || !data )
		return 0;

	uint8_t pciBytes = (len <= 7) ? 1 : 2;
	uint8_t maxPayload = frameSize - pciOffset - pciBytes;
	if ( len > maxPayload )
	{
		/* Error. The data size is too large. Cant transmit with a single frame. */
		return 0;
	}

	if ( len <= 7 )
	{
		/* bxCAN 2.0 case. */
		/* Build PCI Byte. */
		uint8_t pciH4Bit = CUS_CANTP_PCI_SF;
		uint8_t pciL4Bit_SFDL = len; 
		frame[pciOffset] = ((pciH4Bit << 4) | pciL4Bit_SFDL & 0x0F);
	}
	else 
	{
		/* CAN_FD case. */
		frame[pciOffset] = 0;
		frame[pciOffset + 1] = len;
	}

	/* Copy Data. */
	memcpy(&frame[pciOffset + pciBytes], data, len);

	/* Padding to DLC=8 CAN frame. */
	uint8_t padLen = maxPayload - len;
	if ( padLen > 0 )
		memset(&frame[pciOffset + pciBytes + len], 0, padLen);

	return frameSize;
}


uint8_t 
Cus_Cantp_BuildFF( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t *data, uint32_t totLen )
{
	if ( !frame || !data )
		return 0;

	uint8_t maxPayLoad = frameSize - pciOffset - 2;
	uint8_t sfMAX = frameSize - pciOffset - 1;

	if ( (totLen <= sfMAX) || (totLen > 4095) )
	{
		/* Error. Can use a single frame to carry these data. */
		/* No Need to segment. */
		/* Or the totLen greater than the limit. */
		return 0;
	}

	/* Build PCI Byte. */
    uint16_t pciWord = (uint16_t)((CUS_CANTP_PCI_FF << 12) | (totLen & 0x0FFF));
    frame[pciOffset] = (uint8_t)((pciWord >> 8) & 0xFF);
    frame[pciOffset + 1] = (uint8_t)(pciWord & 0xFF);

	/* Copy Data. */
	memcpy(&frame[pciOffset + 2], data, maxPayLoad);

	return frameSize;
}


uint8_t 
Cus_Cantp_BuildCF( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t *data, uint32_t remaining, uint8_t snCode, uint8_t *pCopyLen )
{
	if ( !frame || !data || !pCopyLen )
		return 0;

	uint8_t maxPayload = frameSize - pciOffset - 1;

	/* Build PCI Byte. */
	uint8_t pciByte = (uint8_t)((CUS_CANTP_PCI_CF << 4) | (snCode & 0x0F));
	frame[pciOffset] = pciByte;

	/* Copy Data. */
	uint8_t copyLen = (remaining < maxPayload) ? (uint8_t)remaining : maxPayload;
	*pCopyLen = copyLen;
	memcpy(&frame[pciOffset + 1], data, copyLen);

	/* The last one of the CF? */
	uint8_t padLen = maxPayload - copyLen;
	if ( padLen > 0 )
		/* Last CF. We pad it to DLC=8. */
		memset(&frame[pciOffset + 1 + copyLen], 0, padLen);

	return frameSize;
}


uint8_t 
Cus_Cantp_BuildFC( uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, uint8_t flowState, uint8_t bs, uint8_t stmin )
{
	if ( !frame )
		return 0;

	(void)frameSize;

	uint8_t maxPayload = 8 - pciOffset - 3;

	/* Build PCI Byte. */
	uint8_t pciByte = ((CUS_CANTP_PCI_FC << 4) | (flowState & 0x0F));
	frame[pciOffset] = pciByte;

	/* Fill BS & STmin. */
	frame[pciOffset + 1] = bs;
	frame[pciOffset + 2] = stmin;

	/* The FC's payload cant carry any data.We Set it to 0. */
	memset(&frame[pciOffset + 3], 0, maxPayload);

	return 8;
}



/* ************************************************************************  */
/*  Parse                                                                    */
/* ************************************************************************  */
Cus_CANTP_PCIType_t 
Cus_Cantp_GetPciType( const uint8_t *frame, uint8_t pciOffset )
{
	if ( !frame )
		return CUS_CANTP_PCI_UNKNOWN;
	
	uint8_t nibble = ((frame[pciOffset] >> 4) & 0x0F);
	if ( nibble > CUS_CANTP_PCI_FC )
		return CUS_CANTP_PCI_UNKNOWN;

	return (Cus_CANTP_PCIType_t)nibble;
}


uint8_t 
Cus_Cantp_ParseSF( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t **pData, uint8_t *pLen )
{
	if ( !frame || !pData || !pLen )
		return 0;

	uint8_t sf_dl = frame[pciOffset] & 0x0F;
	uint8_t pciBytes = 0;
	if ( !sf_dl && frameSize > 8 )
	{
		/* CAN_FD case. */
		sf_dl = frame[pciOffset + 1];
		pciBytes = 2;
	}
	else 
	{
		pciBytes = 1;
	}
	
	/* Get transmition data length. */
	*pLen = sf_dl;

	/* pData -> data. No copy. */
	*pData = &frame[pciOffset + pciBytes];

	return 1;
}


uint32_t 
Cus_Cantp_ParseFF( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, const uint8_t **pData, uint8_t *pLen )
{
	if ( !frame || !pData || !pLen )
		return 0;

	uint8_t maxPayLoad = frameSize - pciOffset - 2;

	if ( (frame[pciOffset] & 0x0F) == 0 && frame[pciOffset + 1] == 0 )
	{
		/* Message larger than 4095 Bytes. Not Supposed! */
		return 0;
	}

	/* Get total length. */
	uint32_t totLen = ((uint32_t)(frame[pciOffset] & 0x0F) << 8) | frame[pciOffset + 1];

	/* Get data len in FF. */
	*pLen = maxPayLoad;

	/*c. */
	*pData = &frame[pciOffset + 2];

	return totLen;
}


uint8_t 
Cus_Cantp_ParseCF( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, uint8_t *pSN, const uint8_t **pData, uint8_t *pLen )
{
	if ( !frame || !pSN || !pData || !pLen )
		return 0;

	uint8_t maxPayLoad = frameSize - pciOffset  - 1;
	
	/* Get SNCode. */
	*pSN = frame[pciOffset] & 0x0F;

	/* The data len equal maxPayLoad. */
	*pLen = maxPayLoad;

	/* pData -> data. */
	*pData = &frame[pciOffset + 1];

	return 1;
}


uint8_t 
Cus_Cantp_ParseFC( const uint8_t *frame, uint8_t frameSize, uint8_t pciOffset, uint8_t *pFlowState, uint8_t *pBS, uint8_t *pSTmin )
{
	if ( !frame || !pFlowState || !pBS || !pSTmin )
		return 0;

	(void)frameSize;

	/* Get Flowstate. */
	*pFlowState = frame[pciOffset] & 0x0F;

	/* Get BS & STmin. */
	*pBS = frame[pciOffset + 1];
	*pSTmin = frame[pciOffset + 2];

	return 1;
}
