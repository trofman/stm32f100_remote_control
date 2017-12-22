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

#define BUTTON_VALUE							(usRegHoldingUserBuf[1])
#define LED_1									(usRegHoldingUserBuf[2])
#define LED_2									(usRegHoldingUserBuf[3])
#define LED_3									(usRegHoldingUserBuf[4])
#define PWM_VALUE								(usRegHoldingUserBuf[5])
#define ANALOG_VALUE							(usRegHoldingUserBuf[6])

#endif /* _REGISTERS_H */
