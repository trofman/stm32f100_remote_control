#ifndef _REGISTERS_H
#define _REGISTERS_H

#include "stdint.h"

#include "mb.h"

/*
 * Holding Registers for Modbus RTU
 */


/* Registers for user-mode configuration */

#define REG_HOLDING_USER_START					((uint32_t) 0)
#define REG_HOLDING_USER_NREGS					((uint32_t) 26)

extern uint16_t usRegHoldingUserBuf[];

#define VALUE_REG								(usRegHoldingUserBuf[1])


#endif /* _REGISTERS_H */
