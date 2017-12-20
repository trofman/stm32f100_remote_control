#include "registers.h"

uint16_t usRegHoldingUserBuf[REG_HOLDING_USER_NREGS] = {0};

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if ((usAddress >= REG_HOLDING_USER_START) &&
    	(usAddress + usNRegs <= REG_HOLDING_USER_START + REG_HOLDING_USER_NREGS))
    {
    	switch ( eMode ) {
    	case MB_REG_READ:
    		iRegIndex = ( int )( usAddress - REG_HOLDING_USER_START );
    		while ( usNRegs > 0 ) {
    			*pucRegBuffer++ = ( unsigned char )( usRegHoldingUserBuf[iRegIndex] >> 8 );
    			*pucRegBuffer++ = ( unsigned char )( usRegHoldingUserBuf[iRegIndex] & 0xFF );
    			iRegIndex++;
    			usNRegs--;
    		}
    		break;
    	case MB_REG_WRITE:
    		iRegIndex = ( int )( usAddress - REG_HOLDING_USER_START );
    		while ( usNRegs > 0 ) {
    			usRegHoldingUserBuf[iRegIndex] = (uint16_t)(*pucRegBuffer++) << 8;
    			usRegHoldingUserBuf[iRegIndex] += (uint16_t)(*pucRegBuffer++);
    			iRegIndex++;
    			usNRegs--;
    		}
    		break;
    	}
    }
    else {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    return MB_ENOREG;
}
